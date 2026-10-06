#!/usr/bin/env python3
"""Download remaining git-lfs objects with small, checkpointed chunks.

Chunks (default 2 MiB) are written to .kb_tmp/lfs_chunks/<oid>/ as they finish,
so progress survives restarts and a stalled chunk only costs 2 MiB.
"""
import concurrent.futures as cf
import hashlib
import os
import re
import sys
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPOS = os.path.join(ROOT, "repos")
CHUNK_DIR = os.path.join(ROOT, ".kb_tmp", "lfs_chunks")
REPO_BRANCH = {"engineai_rl_lab": "main", "engineai_navigation": "master"}
CHUNK = 2 << 20
WORKERS = 12
RETRIES = 3


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


def fetch_chunk(url, idx, start, end, dest):
    tmp = dest + ".part"
    if os.path.exists(dest) and os.path.getsize(dest) == end - start + 1:
        return idx, True, "cached"
    for i in range(RETRIES):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "kb-lfs", "Range": f"bytes={start}-{end}"})
            with urllib.request.urlopen(req, timeout=90) as r:
                data = r.read()
            if len(data) != end - start + 1:
                raise IOError(f"short read {len(data)}")
            with open(tmp, "wb") as f:
                f.write(data)
            os.replace(tmp, dest)
            return idx, True, "ok"
        except Exception as e:  # noqa: BLE001
            if i == RETRIES - 1:
                return idx, False, str(e)
    return idx, False, "unreachable"


def main():
    objs = find_pointers()
    if not objs:
        print("no LFS pointers left")
        return 0
    rc = 0
    for oid, info in objs.items():
        repo, rel = info["paths"][0]
        url = f"https://media.githubusercontent.com/media/engineai-robotics/{REPO_BRANCH[repo]}/{rel}".replace(
            f"/{REPO_BRANCH[repo]}/", f"/{REPO_BRANCH[repo]}/")
        url = f"https://media.githubusercontent.com/media/engineai-robotics/{repo}/{REPO_BRANCH[repo]}/{rel}"
        size = info["size"]
        d = os.path.join(CHUNK_DIR, oid)
        os.makedirs(d, exist_ok=True)
        ranges = [(i * CHUNK, min(size - 1, (i + 1) * CHUNK - 1)) for i in range((size + CHUNK - 1) // CHUNK)]
        done = sum(1 for i, s, e in [(i, s, e) for i, (s, e) in enumerate(ranges)]
                   if os.path.exists(os.path.join(d, f"{i:05d}")) and os.path.getsize(os.path.join(d, f"{i:05d}")) == e - s + 1)
        print(f"{oid[:12]} size={size/1048576:.1f}MB chunks={len(ranges)} cached={done}", flush=True)
        with cf.ThreadPoolExecutor(WORKERS) as ex:
            futs = [ex.submit(fetch_chunk, url, i, s, e, os.path.join(d, f"{i:05d}")) for i, (s, e) in enumerate(ranges)]
            for fut in cf.as_completed(futs):
                idx, ok, msg = fut.result()
                if not ok:
                    print(f"  chunk {idx} FAILED: {msg}", flush=True)
        # assemble
        missing = [i for i in range(len(ranges)) if not os.path.exists(os.path.join(d, f"{i:05d}"))]
        if missing:
            print(f"{oid[:12]}: {len(missing)} chunks missing, rerun to resume", flush=True)
            rc = 1
            continue
        h = hashlib.sha256()
        with open(os.path.join(d, "assembled"), "wb") as out:
            for i in range(len(ranges)):
                with open(os.path.join(d, f"{i:05d}"), "rb") as f:
                    data = f.read()
                h.update(data)
                out.write(data)
        if h.hexdigest() != oid:
            print(f"{oid[:12]}: sha256 MISMATCH, not written", flush=True)
            rc = 1
            continue
        with open(os.path.join(d, "assembled"), "rb") as f:
            data = f.read()
        for repo, rel in info["paths"]:
            with open(os.path.join(REPOS, repo, rel), "wb") as f:
                f.write(data)
        print(f"{oid[:12]}: OK, sha256 verified, written to {len(info['paths'])} path(s)", flush=True)
    return rc


if __name__ == "__main__":
    sys.exit(main())
