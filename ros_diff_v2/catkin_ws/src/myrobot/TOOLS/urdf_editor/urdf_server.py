#!/usr/bin/env python3
"""
URDF Editor Server.

提供静态页面与受限的 GET/POST 接口，用于读取和回写 myrobot 包内的 xacro 与 yaml 文件。
路径必须落在 PACKAGE_ROOT 内（用 resolve + relative_to 校验），越界返回 403。

Usage:
    python urdf_server.py [--port 8766] [--bind 127.0.0.1]
"""

import argparse
import http.server
import json
import os
import posixpath
import shutil
import socketserver
import sys
import tempfile
import urllib.parse
import xml.etree.ElementTree as ET
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_ROOT = SCRIPT_DIR.parent.parent.resolve()
DEFAULT_URDF = PACKAGE_ROOT / "my_urdf" / "robot.urdf.xacro"
DEFAULT_PORT = 8766

# 文件大小限制，避免误读超大文件
MAX_BYTES = 1 << 20  # 1 MiB
BACKUP_COUNT = 3


class Handler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, fmt, *args):
        sys.stderr.write("[urdf_editor] " + (fmt % args) + "\n")

    # 静态文件服务：仅服务本目录
    def translate_path(self, path):
        path = path.split("?", 1)[0].split("#", 1)[0]
        path = posixpath.normpath(urllib.parse.unquote(path))
        words = [w for w in path.split("/") if w]
        full = SCRIPT_DIR
        for w in words:
            if os.path.dirname(w) or w in (os.curdir, os.pardir):
                continue
            full = full / w
        return str(full)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/urdf":
            return self._handle_read(parsed.query, suffixes=(".xacro", ".urdf"))
        if parsed.path == "/api/config":
            return self._handle_read(parsed.query, suffixes=(".yaml", ".yml"))
        if parsed.path == "/api/defaults":
            return self._handle_defaults()
        return super().do_GET()

    def do_POST(self):
        if not self._origin_allowed():
            self.send_error(403, "Cross-origin write rejected")
            return
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/urdf":
            return self._handle_write(suffixes=(".xacro", ".urdf"))
        if parsed.path == "/api/config":
            return self._handle_write(suffixes=(".yaml", ".yml"))
        self.send_error(404, "Unknown endpoint")

    def _resolve(self, rel, allowed_suffixes):
        if not rel:
            return None
        rel = rel.lstrip("/\\")
        candidate = (PACKAGE_ROOT / rel).resolve()
        try:
            candidate.relative_to(PACKAGE_ROOT)
        except ValueError:
            return None
        if not candidate.is_file():
            return None
        if allowed_suffixes and candidate.suffix.lower() not in allowed_suffixes:
            return None
        return candidate

    def _send_json(self, payload, status=200):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def _origin_allowed(self):
        """Allow command-line clients and same-origin browser writes only."""
        origin = self.headers.get("Origin")
        if not origin:
            return True
        parsed = urllib.parse.urlparse(origin)
        return parsed.scheme in ("http", "https") and parsed.netloc == self.headers.get("Host")

    def _handle_defaults(self):
        self._send_json({
            "package_root": str(PACKAGE_ROOT),
            "urdf_path": str(DEFAULT_URDF),
        })

    def _handle_read(self, query, suffixes):
        params = urllib.parse.parse_qs(query)
        rel = params.get("path", [None])[0]
        target = self._resolve(rel or "my_urdf/robot.urdf.xacro", suffixes)
        if target is None:
            self.send_error(403, "Path not allowed")
            return
        try:
            data = target.read_bytes()
        except FileNotFoundError:
            self.send_error(404, "File not found")
            return
        if len(data) > MAX_BYTES:
            self.send_error(413, "File too large")
            return
        self._send_json({
            "path": str(target),
            "content": data.decode("utf-8", errors="replace"),
        })

    def _handle_write(self, suffixes):
        try:
            length = int(self.headers.get("Content-Length", 0))
        except (TypeError, ValueError):
            self.send_error(400, "Invalid Content-Length")
            return
        if length <= 0 or length > MAX_BYTES:
            self.send_error(400, "Empty or oversized payload")
            return
        raw = self.rfile.read(length)
        try:
            data = json.loads(raw.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            self.send_error(400, "Invalid JSON")
            return
        target = self._resolve(data.get("path", ""), suffixes)
        if target is None:
            self.send_error(403, "Path not allowed")
            return
        content = data.get("content", "")
        if not isinstance(content, str):
            self.send_error(400, "content must be a string")
            return
        if len(content.encode("utf-8")) > MAX_BYTES:
            self.send_error(413, "File too large")
            return

        validation_error = self._validate_content(target, content)
        if validation_error:
            self.send_error(400, validation_error)
            return

        backup_path = None
        if target.exists():
            backup_path = target.with_suffix(target.suffix + ".bak")
            try:
                self._rotate_backups(backup_path)
                shutil.copy2(target, backup_path)
            except OSError as e:
                self.send_error(500, "Backup failed: " + str(e))
                return

        temp_name = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                newline="",
                dir=str(target.parent),
                prefix=target.name + ".",
                suffix=".tmp",
                delete=False,
            ) as temp_file:
                temp_name = temp_file.name
                temp_file.write(content)
                temp_file.flush()
                os.fsync(temp_file.fileno())
            os.replace(temp_name, target)
        except OSError as e:
            if temp_name:
                try:
                    os.unlink(temp_name)
                except OSError:
                    pass
            self.send_error(500, "Write failed: " + str(e))
            return
        self._send_json({
            "ok": True,
            "path": str(target),
            "backup": str(backup_path) if backup_path else None,
        })

    @staticmethod
    def _validate_content(target, content):
        if "\x00" in content:
            return "NUL bytes are not allowed"
        if target.suffix.lower() in (".xacro", ".urdf"):
            try:
                ET.fromstring(content)
            except ET.ParseError as exc:
                return "Invalid XML/Xacro: " + str(exc)
        return None

    @staticmethod
    def _rotate_backups(backup_path):
        oldest = Path(str(backup_path) + f".{BACKUP_COUNT - 1}")
        if oldest.exists():
            oldest.unlink()
        for index in range(BACKUP_COUNT - 2, 0, -1):
            older = Path(str(backup_path) + f".{index}")
            newer = Path(str(backup_path) + f".{index + 1}")
            if older.exists():
                os.replace(older, newer)
        if backup_path.exists():
            os.replace(backup_path, Path(str(backup_path) + ".1"))


def main():
    parser = argparse.ArgumentParser(description="URDF Editor Server")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--bind", default="127.0.0.1")
    args = parser.parse_args()

    os.chdir(SCRIPT_DIR)
    socketserver.TCPServer.allow_reuse_address = True
    url = f"http://{args.bind}:{args.port}/"
    with socketserver.TCPServer((args.bind, args.port), Handler) as httpd:
        print("URDF editor server")
        print(f"  URL:        {url}")
        print(f"  Web root:   {SCRIPT_DIR}")
        print(f"  Package:    {PACKAGE_ROOT}")
        print(f"  Default:    {DEFAULT_URDF}")
        print("  Press Ctrl+C to stop.")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
