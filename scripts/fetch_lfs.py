#!/usr/bin/env python3
"""Fetch git-lfs objects for downloaded repos and replace pointer files in place.

Downloads from GitHub media endpoint (media.githubusercontent.com), dedupes by
sha256 oid, verifies content hash before writing.
"""
import concurrent.futures as cf
import hashlib
import os
import re
import sys
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPOS = os.path.join(ROOT, "repos")
REPO_BRANCH = {"engineai_rl_lab": "main", "engineai_navigation": "master"}
WORKERS = 8


def find_pointers():
    objs = {}
    for repo in REPO_BRANCH:
        base = os.path.join(REPOS, repo)
        for dp, _dn, fn in os.walk(base):
            for f in fn:
                p = os.path.join(dp, f)
                try:
                    with open(p, "rb") as fh:
                        head = fh.read(160)
                except OSError:
                    continue
                if not head.startswith(b"version https://git-lfs.github.com/spec"):
                    continue
                txt = head.decode("utf-8", "replace")
                oid = re.search(r"oid sha256:([0-9a-f]{64})", txt)
                size = re.search(r"size (\d+)", txt)
                if oid and size:
                    objs.setdefault(oid.group(1), {"size": int(size.group(1)), "paths": []})
                    objs[oid.group(1)]["paths"].append((repo, os.path.relpath(p, base)))
    return objs


def media_url(repo, rel):
    return f"https://media.githubusercontent.com/media/engineai-robotics/{repo}/{REPO_BRANCH[repo]}/{rel}"


def fetch_one(oid, info):
    repo, rel = info["paths"][0]
    url = media_url(repo, rel)
    h = hashlib.sha256()
    buf = []
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "kb-lfs"})
        with urllib.request.urlopen(req, timeout=300) as r:
            while True:
                chunk = r.read(1 << 20)
                if not chunk:
                    break
                h.update(chunk)
                buf.append(chunk)
    except Exception as e:  # noqa: BLE001
        return oid, None, f"download error: {e}"
    data = b"".join(buf)
    if h.hexdigest() != oid:
        return oid, None, f"sha256 mismatch: got {h.hexdigest()}"
    if len(data) != info["size"]:
        return oid, None, f"size mismatch: got {len(data)} want {info['size']}"
    for repo, rel in info["paths"]:
        with open(os.path.join(REPOS, repo, rel), "wb") as f:
            f.write(data)
    return oid, len(data), None


def main():
    objs = find_pointers()
    total = sum(v["size"] for v in objs.values())
    print(f"{len(objs)} unique LFS objects, {total/1048576:.1f} MB total")
    ok = 0
    done_bytes = 0
    with cf.ThreadPoolExecutor(WORKERS) as ex:
        futs = {ex.submit(fetch_one, oid, info): (oid, info) for oid, info in objs.items()}
        for fut in cf.as_completed(futs):
            oid, n, err = fut.result()
            info = futs[fut][1]
            if err:
                print(f"FAIL {oid[:12]} ({info['size']/1048576:.1f}MB): {err}")
            else:
                ok += 1
                done_bytes += n
                print(f"OK   {oid[:12]} {n/1048576:7.1f}MB -> {info['paths'][0][1][:70]}")
    print(f"DONE {ok}/{len(objs)} objects, {done_bytes/1048576:.1f} MB written")
    return 0 if ok == len(objs) else 1


if __name__ == "__main__":
    sys.exit(main())
