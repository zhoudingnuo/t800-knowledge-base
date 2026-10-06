#!/usr/bin/env python3
"""Crawl EngineAI T800 online docs (engineai.com.cn/open/docs/t800) into Markdown.

No external deps: uses urllib + html.parser. Images are downloaded to
docs/assets/ and Markdown image links are rewritten to local relative paths.
"""
import html
import os
import re
import sys
import time
import urllib.request
from html.parser import HTMLParser

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASSETS = os.path.join(ROOT, "docs", "assets")
BASE = "https://engineai.com.cn"

PRODUCTS = {
    "t800": {
        "out": os.path.join(ROOT, "docs", "t800-online"),
        "sections": {
            "t800-developer": [f"{i:02d}" for i in range(1, 8)],
            "ros2-interface": [f"{i:02d}" for i in range(1, 11)],
            "native-sdk": ["01", "02", "3", "4"],
            "t800-pro": [f"{i:02d}" for i in range(1, 9)],
            "support": [f"{i:02d}" for i in range(1, 4)],
        },
    },
    "pm01": {
        "out": os.path.join(ROOT, "docs", "pm01-online"),
        "sections": {
            "pm01": [f"{i:02d}" for i in range(1, 6)],
            "ros2-interface": [f"{i:02d}" for i in range(1, 11)],
            "native-sdk": ["01", "02", "3", "4"],
            "support": [f"{i:02d}" for i in range(1, 4)],
        },
    },
}

BLOCK = {"p", "div", "section", "h1", "h2", "h3", "h4", "h5", "h6",
         "ul", "ol", "li", "pre", "table", "tr", "blockquote", "hr", "br"}


class Md(HTMLParser):
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.out = []
        self.stack = []
        self.pre = 0
        self.href = None
        self.img_srcs = []
        self.skip = 0

    def handle_starttag(self, tag, attrs):
        a = dict(attrs)
        if tag in ("script", "style"):
            self.skip += 1
            return
        if tag == "pre":
            self.pre += 1
            self.out.append("\n```\n")
        elif tag == "code" and not self.pre:
            self.out.append("`")
        elif tag in ("h1", "h2", "h3", "h4", "h5", "h6"):
            self.out.append("\n\n" + "#" * int(tag[1]) + " ")
        elif tag == "p":
            self.out.append("\n\n")
        elif tag == "br":
            self.out.append("\n")
        elif tag == "hr":
            self.out.append("\n\n---\n\n")
        elif tag == "li":
            self.out.append("\n- ")
        elif tag in ("ul", "ol"):
            self.out.append("\n")
        elif tag == "blockquote":
            self.out.append("\n\n> ")
        elif tag == "strong" or tag == "b":
            self.out.append("**")
        elif tag == "em" or tag == "i":
            self.out.append("*")
        elif tag == "a":
            self.href = a.get("href")
            self.out.append("[")
        elif tag == "img":
            src = a.get("src", "")
            alt = a.get("alt", "") or "image"
            if src:
                self.img_srcs.append(src)
                self.out.append(f"\n\n![{alt}](IMG::{src})\n\n")
        elif tag == "td" or tag == "th":
            self.out.append(" | ")
        elif tag == "tr":
            self.out.append("\n")
        self.stack.append(tag)

    def handle_endtag(self, tag):
        if tag in ("script", "style"):
            self.skip = max(0, self.skip - 1)
            return
        if tag == "pre":
            self.pre = max(0, self.pre - 1)
            self.out.append("\n```\n")
        elif tag == "code" and not self.pre:
            self.out.append("`")
        elif tag in ("h1", "h2", "h3", "h4", "h5", "h6", "p", "blockquote"):
            self.out.append("\n")
        elif tag in ("strong", "b"):
            self.out.append("**")
        elif tag in ("em", "i"):
            self.out.append("*")
        elif tag == "a":
            href = self.href or ""
            self.href = None
            self.out.append(f"]({href})")
        if self.stack and self.stack[-1] == tag:
            self.stack.pop()

    def handle_data(self, data):
        if self.skip:
            return
        if self.pre:
            self.out.append(data)
        else:
            self.out.append(re.sub(r"\s+", " ", data))

    def text(self):
        s = "".join(self.out)
        s = re.sub(r"[ \t]+\n", "\n", s)
        s = re.sub(r"\n{3,}", "\n\n", s)
        return s.strip()


def fetch(url, binary=False, tries=3):
    last = None
    for i in range(tries):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0 (kb-crawler)"})
            with urllib.request.urlopen(req, timeout=45) as r:
                data = r.read()
            return data if binary else data.decode("utf-8", "replace")
        except Exception as e:  # noqa: BLE001
            last = e
            time.sleep(2 * (i + 1))
    raise last


def slugify(t):
    t = re.sub(r"^[\d\.\s]+", "", t)
    t = re.sub(r"[\\/:*?\"<>|\s]+", "-", t).strip("-")
    return t[:60] or "page"


def main():
    import sys as _sys
    wanted = _sys.argv[1:] or list(PRODUCTS)
    os.makedirs(ASSETS, exist_ok=True)
    manifest = []
    for product in wanted:
        cfg = PRODUCTS[product]
        OUT = cfg["out"]
        for sec, pages in cfg["sections"].items():
            d = os.path.join(OUT, sec)
            os.makedirs(d, exist_ok=True)
            for p in pages:
                url = f"{BASE}/open/docs/{product}/{sec}/{p}?product={product}"
                try:
                    h = fetch(url)
                except Exception as e:  # noqa: BLE001
                    print(f"FAIL {product}/{sec}/{p}: {e}")
                    continue
                title = re.search(r"<title>(.*?)</title>", h, re.S)
                title = html.unescape(title.group(1)).split("|")[0].strip() if title else f"{sec}-{p}"
                m = re.search(r'<article class="docContent">(.*?)</article>', h, re.S)
                body = m.group(1) if m else ""
                conv = Md()
                conv.feed(body)
                conv.close()
                md = conv.text()
                # download images
                for i, src in enumerate(conv.img_srcs, 1):
                    if not src.startswith("/"):
                        continue
                    name = re.sub(r"[^A-Za-z0-9._-]+", "_", urllib.request.unquote(src.split("?")[0]).rsplit("/", 1)[-1])[:80]
                    local = "assets/" + name
                    path = os.path.join(ASSETS, name)
                    if not os.path.exists(path):
                        try:
                            data = fetch(BASE + src, binary=True)
                            with open(path, "wb") as f:
                                f.write(data)
                        except Exception as e:  # noqa: BLE001
                            print(f"  img fail {src}: {e}")
                    md = md.replace(f"IMG::{src}", "../../" + local)
                fname = f"{p}-{slugify(title)}.md"
                with open(os.path.join(d, fname), "w", encoding="utf-8") as f:
                    f.write(f"<!-- 来源: {url} -->\n\n# {title}\n\n{md}\n")
                manifest.append((product, sec, p, title, fname, len(md)))
                print(f"OK {product}/{sec}/{p} {title} ({len(md)} chars)")
    with open(os.path.join(ROOT, "docs", "manifest.tsv"), "w", encoding="utf-8") as f:
        for row in manifest:
            f.write("\t".join(str(x) for x in row) + "\n")
    print(f"TOTAL {len(manifest)} pages")


if __name__ == "__main__":
    sys.exit(main())
