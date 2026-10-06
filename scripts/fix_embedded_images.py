#!/usr/bin/env python3
"""Decode inline base64 data-URI images in crawled docs into real files.

Rewrites the markdown to reference docs/assets/<page>-<n>.<ext>.
"""
import base64
import glob
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCS = os.path.join(ROOT, "docs", "t800-online")
ASSETS = os.path.join(ROOT, "docs", "assets")
PAT = re.compile(r"IMG::data:image/(png|jpeg|jpg|gif|webp|svg\+xml);base64,([A-Za-z0-9+/=\s]+)")

EXT = {"jpeg": "jpg", "svg+xml": "svg"}


def main():
    os.makedirs(ASSETS, exist_ok=True)
    total = 0
    for md in glob.glob(os.path.join(DOCS, "**", "*.md"), recursive=True):
        s = open(md, encoding="utf-8").read()
        rdir = os.path.relpath(os.path.dirname(md), DOCS).replace(os.sep, "_")
        page = f"{rdir}_{os.path.splitext(os.path.basename(md))[0]}"
        rel_assets = os.path.relpath(ASSETS, os.path.dirname(md))
        n = 0

        def repl(m):
            nonlocal n
            n += 1
            ext = EXT.get(m.group(1), m.group(1))
            data = base64.b64decode(re.sub(r"\s+", "", m.group(2)))
            name = f"{page}-{n:02d}.{ext}"
            with open(os.path.join(ASSETS, name), "wb") as f:
                f.write(data)
            return f"{rel_assets}/{name}"

        s2 = PAT.sub(repl, s)
        # any remaining IMG:: markers are non-data refs the crawler missed
        leftover = re.findall(r"IMG::[^\s)\"']+", s2)
        s2 = re.sub(r"IMG::", "", s2)
        if s2 != s:
            open(md, "w", encoding="utf-8").write(s2)
            total += n
            print(f"{os.path.relpath(md, ROOT)}: {n} images extracted ({len(data) if False else ''})")
        if leftover:
            print(f"  leftover refs: {leftover[:5]}")
    print(f"TOTAL {total} embedded images")


if __name__ == "__main__":
    main()
