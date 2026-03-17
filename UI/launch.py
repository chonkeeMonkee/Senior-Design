#!/usr/bin/env python3
"""
ESP32 Submarine Dashboard — Native GUI Launcher
------------------------------------------------
Starts a local HTTP server on port 8080 and opens the dashboard
in a native OS window (via pywebview) instead of a browser tab.

Requirements:
    pip install pywebview

Usage:
    python3 launch.py
    OR  double-click this file in Finder (if .py is associated)
"""

import http.server
import os
import sys
import threading
import time

# ── Configuration ─────────────────────────────────────────────
PORT        = 8080
TITLE       = "ESP32 Submarine Dashboard"
WIDTH       = 1440
HEIGHT      = 860
DASHBOARD   = os.path.dirname(os.path.abspath(__file__))
URL         = f"http://localhost:{PORT}"

# ── Kill any process already holding the port ─────────────────
def _free_port():
    import subprocess
    try:
        pids = subprocess.check_output(
            ["lsof", "-ti", f"tcp:{PORT}"],
            stderr=subprocess.DEVNULL
        ).decode().split()
        for pid in pids:
            subprocess.call(["kill", "-9", pid],
                            stderr=subprocess.DEVNULL)
        time.sleep(0.3)
    except Exception:
        pass

# ── HTTP server thread ─────────────────────────────────────────
def _start_server():
    handler = http.server.SimpleHTTPRequestHandler
    handler.log_message = lambda *a: None  # suppress access logs

    os.chdir(DASHBOARD)
    with http.server.HTTPServer(("", PORT), handler) as httpd:
        httpd.serve_forever()

def start_server():
    _free_port()
    t = threading.Thread(target=_start_server, daemon=True)
    t.start()
    # Give the server a moment to be ready before opening the window
    time.sleep(0.6)

# ── Main ───────────────────────────────────────────────────────
def main():
    try:
        import webview
    except ImportError:
        print("=" * 60)
        print("  pywebview not found. Install it with:")
        print("    pip install pywebview")
        print("=" * 60)
        # Fallback: open in default browser
        import webbrowser
        start_server()
        print(f"Opening dashboard in browser at {URL}")
        webbrowser.open(URL)
        print("Press Ctrl+C to stop the server.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        return

    start_server()

    # ── Set macOS dock icon ───────────────────────────────────
    icon_path = os.path.join(DASHBOARD, "Sub_image_transparent.png")
    try:
        from AppKit import NSApplication, NSImage
        ns_app = NSApplication.sharedApplication()
        ns_image = NSImage.alloc().initWithContentsOfFile_(icon_path)
        if ns_image:
            ns_app.setApplicationIconImage_(ns_image)
    except Exception as e:
        print(f"[warn] Could not set dock icon: {e}")

    window = webview.create_window(
        title       = TITLE,
        url         = URL,
        width       = WIDTH,
        height      = HEIGHT,
        resizable   = True,
        min_size    = (900, 600),
        background_color="#0a0e1a",   # match dashboard background
    )

    webview.start(debug=False)

if __name__ == "__main__":
    main()
