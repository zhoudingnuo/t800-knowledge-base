#!/usr/bin/env python3
"""Fetch remaining git-lfs objects using parallel HTTP Range requests.

Same as fetch_lfs.py but splits each large object into N chunks fetched
concurrently (media.githubusercontent.com supports Range/206).
"""
import concurrent.futures as cf
import hashlib
import os
import re
import sys
import time
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPOS = os.path.join(ROOT, "repos")
REPO_BRANCH = {"engineai_rl_lab": "main", "engineai_navigation": "master"}
CHUNKS = 8
RETRIES = 4


def find_pointers():
    objs = {}
    for repo in REPO_BRANCH:
        base = os.path.join(REPOS, repo)
        for dp, _dn, fn in os.walk(base):
            for f in fn:
                p = os.path.join(dp, f)
                try:
                    with open(p, "rb") as fh:
                        head = fh.read(200)
                except OSError:
                    continue
                if not head.startswith(b"version https://git-lfs.github.com/spec"):
                    continue
                t = head.decode("utf-8", "replace")
                oid = re.search(r"oid sha256:([0-9a-f]{64})", t)
                size = re.search(r"size (\d+)", t)
                if oid and size:
                    objs.setdefault(oid.group(1), {"size": int(size.group(1)), "paths": []})
                    objs[oid.group(1)]["paths"].append((repo, os.path.relpath(p, base)))
    return objs


def media_url(repo, rel):
    return f"https://media.githubusercontent.com/media/engineai-robotics/{repo}/{REPO_BRANCH[repo]}/{rel}"


def get_range(url, start, end):
    last = None
    for i in range(RETRIES):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "kb-lfs", "Range": f"bytes={start}-{end}"})
            with urllib.request.urlopen(req, timeout=180) as r:
                data = r.read()
            if len(data) != end - start + 1:
                raise IOError(f"short read {len(data)} != {end - start + 1}")
            return data
        except Exception as e:  # noqa: BLE001
            last = e
            time.sleep(1.5 * (i + 1))
    raise last


def fetch_one(oid, info):
    repo, rel = info["paths"][0]
    url = media_url(repo, rel)
    size = info["size"]
    step = (size + CHUNKS - 1) // CHUNKS
    ranges = [(i * step, min(size - 1, (i + 1) * step - 1)) for i in range(CHUNKS) if i * step < size]
    try:
        with cf.ThreadPoolExecutor(len(ranges)) as ex:
            parts = list(ex.map(lambda r: get_range(url, r[0], r[1]), ranges))
    except Exception as e:  # noqa: BLE001
        return oid, None, f"download error: {e}"
    data = b"".join(parts)
    if len(data) != size:
        return oid, None, f"size mismatch: got {len(data)} want {size}"
    dig = hashlib.sha256(data).hexdigest()
    if dig != oid:
        return oid, None, f"sha256 mismatch: got {dig}"
    for repo, rel in info["paths"]:
        with open(os.path.join(REPOS, repo, rel), "wb") as f:
            f.write(data)
    return oid, size, None


def main():
    objs = find_pointers()
    total = sum(v["size"] for v in objs.values())
    print(f"{len(objs)} unique LFS objects remaining, {total/1048576:.1f} MB", flush=True)
    ok = 0
    with cf.ThreadPoolExecutor(min(4, len(objs) or 1)) as ex:
        futs = {ex.submit(fetch_one, oid, info): (oid, info) for oid, info in objs.items()}
        for fut in cf.as_completed(futs):
            oid, n, err = fut.result()
            info = futs[fut][1]
            if err:
                print(f"FAIL {oid[:12]} ({info['size']/1048576:.1f}MB): {err}", flush=True)
            else:
                ok += 1
                print(f"OK   {oid[:12]} {n/1048576:7.1f}MB -> {info['paths'][0][1][:70]}", flush=True)
    print(f"DONE {ok}/{len(objs)} objects", flush=True)
    return 0 if ok == len(objs) else 1


if __name__ == "__main__":
    sys.exit(main())
