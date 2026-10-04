"""AC8 MouseFlight live telemetry: receives the mod's per-frame UDP datagrams on
127.0.0.1, records every session to recordings/, and serves the reconstruction page.

    python dev/telemetry/server.py [--udp 49731] [--http 8731] [--no-browser]

The mod sends only when config.ini [control] has telemetry_port set (e.g. 49731).
Python standard library only.
"""
import argparse
import collections
import datetime
import json
import os
import re
import socket
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
RECORDINGS = os.path.join(REPO, "recordings")
SESSION_GAP = 5.0          # seconds of silence that end a recording
LIVE_SECONDS = 120         # frames kept in memory for a page that connects mid-flight


class Hub:
    """Frames received so far, the open recording, and waiting page streams."""

    def __init__(self):
        self.lock = threading.Condition()
        self.frames = collections.deque()    # (seq, receive time, json text)
        self.seq = 0
        self.last_receive = 0.0
        self.recording = None
        self.recording_name = None
        self.received = 0

    def add(self, text):
        now = time.time()
        with self.lock:
            if self.recording is None or now - self.last_receive > SESSION_GAP:
                self._start_recording()
                self.frames.clear()
            self.last_receive = now
            self.seq += 1
            self.received += 1
            self.frames.append((self.seq, now, text))
            while self.frames and now - self.frames[0][1] > LIVE_SECONDS:
                self.frames.popleft()
            self.recording.write(text + "\n")
            self.lock.notify_all()

    def _start_recording(self):
        if self.recording:
            self.recording.close()
        os.makedirs(RECORDINGS, exist_ok=True)
        self.recording_name = datetime.datetime.now().strftime("flight-%Y%m%d-%H%M%S.jsonl")
        self.recording = open(os.path.join(RECORDINGS, self.recording_name), "a", encoding="utf-8", buffering=1)
        print(f"recording {self.recording_name}")

    def flush_idle(self):
        with self.lock:
            if self.recording and time.time() - self.last_receive > SESSION_GAP:
                self.recording.close()
                self.recording = None
                print(f"saved {self.recording_name}")


HUB = Hub()


class Server(ThreadingHTTPServer):
    daemon_threads = True

    def handle_error(self, request, client_address):
        # A page reload drops its open connections; that is not worth a traceback.
        import sys
        if isinstance(sys.exc_info()[1], (ConnectionAbortedError, ConnectionResetError, BrokenPipeError)):
            return
        super().handle_error(request, client_address)


def receive(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", port))
    sock.settimeout(1.0)
    print(f"listening for telemetry on udp 127.0.0.1:{port}")
    while True:
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            HUB.flush_idle()
            continue
        text = data.decode("utf-8", "replace").strip()
        if text.startswith("{") and text.endswith("}"):
            HUB.add(text)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def send_body(self, body, content_type, status=200):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            with open(os.path.join(HERE, "index.html"), "rb") as f:
                self.send_body(f.read(), "text/html; charset=utf-8")
        elif path == "/stream":
            self.stream()
        elif path == "/status":
            with HUB.lock:
                status = {"recording": HUB.recording_name if HUB.recording else None,
                          "received": HUB.received, "lastReceive": HUB.last_receive, "now": time.time()}
            self.send_body(json.dumps(status).encode(), "application/json")
        elif path == "/recordings":
            items = []
            if os.path.isdir(RECORDINGS):
                for name in sorted(os.listdir(RECORDINGS), reverse=True):
                    if name.endswith(".jsonl"):
                        full = os.path.join(RECORDINGS, name)
                        items.append({"name": name, "bytes": os.path.getsize(full), "modified": os.path.getmtime(full)})
            self.send_body(json.dumps(items).encode(), "application/json")
        elif re.fullmatch(r"/recordings/[\w.-]+\.jsonl", path):
            full = os.path.join(RECORDINGS, os.path.basename(path))
            if not os.path.isfile(full):
                self.send_body(b"not found", "text/plain", 404)
                return
            with open(full, "rb") as f:
                self.send_body(f.read(), "application/x-ndjson")
        else:
            self.send_body(b"not found", "text/plain", 404)

    def stream(self):
        """Server-sent events: the recent past first, then new frames in small batches."""
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        seen = 0
        try:
            with HUB.lock:
                backlog = [text for _, _, text in HUB.frames]
                seen = HUB.seq
            self.wfile.write(("event: backlog\ndata: [" + ",".join(backlog) + "]\n\n").encode())
            self.wfile.flush()
            while True:
                with HUB.lock:
                    HUB.lock.wait_for(lambda: HUB.seq > seen, timeout=1.0)
                    fresh = [text for seq, _, text in HUB.frames if seq > seen]
                    seen = HUB.seq
                if fresh:
                    self.wfile.write(("data: [" + ",".join(fresh) + "]\n\n").encode())
                else:
                    self.wfile.write(b": idle\n\n")
                self.wfile.flush()
                time.sleep(0.03)   # ~30 updates a second is plenty for the page
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--udp", type=int, default=49731, help="telemetry port (config.ini telemetry_port)")
    parser.add_argument("--http", type=int, default=8731, help="page port")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    threading.Thread(target=receive, args=(args.udp,), daemon=True).start()
    server = Server(("127.0.0.1", args.http), Handler)
    url = f"http://127.0.0.1:{args.http}/"
    print(f"open {url}   (Ctrl+C to stop)")
    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
