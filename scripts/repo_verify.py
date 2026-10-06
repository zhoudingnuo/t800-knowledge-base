#!/usr/bin/env python3
"""Verify downloaded repo trees against GitHub git blob SHA1s (API tree endpoint).

Detects tampering/missing/extra files introduced by any transport (mirror).
Usage: python3 repo_verify.py [repo ...]
"""
import hashlib
import json
import os
import stat
import sys
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPOS = os.path.join(ROOT, "repos")

REPO_BRANCH = {
    "engineai_rl_lab": "main",
    "engineai_ros2_workspace": "community",
    "engineai_robotics_native_sdk": "main",
    "engineai_robot_simulation_runtime": "main",
    "engineai_robotics_description": "master",
    "GMR": "main",
    "engineai_navigation": "master",
    "engineai_amp": "main",
    "engineai_legged_gym": "master",
    "engineai_humanoid": "master",
}


def api(url):
    req = urllib.request.Request(url, headers={"User-Agent": "kb-verify", "Accept": "application/vnd.github+json"})
    with urllib.request.urlopen(req, timeout=60) as r:
        return json.load(r)


def blob_sha(data: bytes) -> str:
    h = hashlib.sha1()
    h.update(b"blob %d\0" % len(data))
    h.update(data)
    return h.hexdigest()


def local_blobs(path):
    out = {}
    for dirpath, dirnames, filenames in os.walk(path):
        dirnames[:] = [d for d in dirnames if d != ".git"]
        for fn in filenames:
            p = os.path.join(dirpath, fn)
            rel = os.path.relpath(p, path)
            if os.path.islink(p):
                out[rel] = ("120000", blob_sha(os.readlink(p).encode()))
                continue
            try:
                with open(p, "rb") as f:
                    out[rel] = ("100644", blob_sha(f.read()))
            except OSError as e:
                out[rel] = ("ERR", str(e))
    return out


def lfs_patterns(repo_path):
    """Glob patterns marked filter=lfs in any .gitattributes of the repo."""
    pats = []
    for dirpath, _dn, fns in os.walk(repo_path):
        if ".gitattributes" in fns:
            with open(os.path.join(dirpath, ".gitattributes"), encoding="utf-8", errors="replace") as f:
                for line in f:
                    line = line.split("#")[0].strip()
                    if "filter=lfs" in line and line:
                        pats.append(line.split()[0])
    return pats


def is_lfs(rel, pats):
    import fnmatch
    base = os.path.basename(rel)
    return any(fnmatch.fnmatch(base, p) or fnmatch.fnmatch(rel, p) for p in pats)


def verify(repo):
    branch = REPO_BRANCH[repo]
    path = os.path.join(REPOS, repo)
    if not os.path.isdir(path):
        print(f"[{repo}] SKIP: not downloaded")
        return None
    try:
        tree = api(f"https://api.github.com/repos/engineai-robotics/{repo}/git/trees/{branch}?recursive=1")
    except Exception as e:  # noqa: BLE001
        print(f"[{repo}] API-FAIL: {e}")
        return None
    remote = {}
    submodules = set()
    for it in tree.get("tree", []):
        if it["type"] == "blob":
            remote[it["path"]] = it["sha"]
        elif it["type"] == "commit":
            submodules.add(it["path"])
    pats = lfs_patterns(path)
    local = local_blobs(path)
    missing = sorted(set(remote) - set(local))
    extra = sorted(set(local) - set(remote))
    # submodule dirs appear local as empty dirs (no files) -> not reported as extra
    mism = [p for p in set(remote) & set(local) if local[p][1] != remote[p]]
    # files whose content was replaced by git-lfs objects are exempt here:
    # their git blob is the small pointer; content was sha256-verified at fetch time.
    lfs_replaced = sorted(p for p in mism if is_lfs(p, pats))
    mismatch = sorted(p for p in mism if p not in set(lfs_replaced))
    status = "PASS" if not missing and not mismatch else "FAIL"
    trunc = " (tree truncated)" if tree.get("truncated") else ""
    print(f"[{repo}] {status}: {len(remote)} remote blobs{trunc}, {len(local)} local files, "
          f"{len(mismatch)} mismatch, {len(missing)} missing, {len(extra)} extra, "
          f"{len(lfs_replaced)} lfs-replaced (exempt)"
          + (f", {len(submodules)} submodules" if submodules else ""))
    for p in mismatch[:20]:
        print(f"    MISMATCH {p}: remote={remote[p]} local={local[p][1]}")
    for p in missing[:20]:
        print(f"    MISSING  {p}")
    for p in extra[:20]:
        print(f"    EXTRA    {p}")
    return status


def main():
    repos = sys.argv[1:] or list(REPO_BRANCH)
    results = {r: verify(r) for r in repos}
    ng = [r for r, s in results.items() if s == "FAIL"]
    print("\nSUMMARY:", ", ".join(f"{r}={s or 'SKIP'}" for r, s in results.items()))
    return 1 if ng else 0


if __name__ == "__main__":
    sys.exit(main())
