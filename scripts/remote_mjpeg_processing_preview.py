#!/usr/bin/env python3
import argparse
import signal
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2


class FrameStore:
    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg = b""
        self.seq = 0

    def update(self, jpeg: bytes):
        with self.cond:
            self.jpeg = jpeg
            self.seq += 1
            self.cond.notify_all()

    def wait_next(self, last_seq: int, timeout: float):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq, timeout=timeout)
            return self.seq, self.jpeg


def process_frame(frame):
    # Replace this block with the algorithm that should be shown in the 8080 UI.
    result = frame.copy()
    cv2.putText(
        result,
        "11 device processing",
        (24, 48),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    return result


def capture_loop(args, store: FrameStore, stop_event: threading.Event):
    delay = 0.0 if args.max_fps <= 0.0 else 1.0 / args.max_fps
    while not stop_event.is_set():
        cap = cv2.VideoCapture(args.input)
        if not cap.isOpened():
            print(f"failed to open input stream: {args.input}", flush=True)
            time.sleep(1.0)
            continue

        print(f"reading input stream: {args.input}", flush=True)
        while not stop_event.is_set():
            started = time.monotonic()
            ok, frame = cap.read()
            if not ok or frame is None:
                print("input stream ended, reconnecting", flush=True)
                break

            result = process_frame(frame)
            ok, encoded = cv2.imencode(
                ".jpg",
                result,
                [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality],
            )
            if ok:
                store.update(encoded.tobytes())

            if delay > 0.0:
                elapsed = time.monotonic() - started
                if elapsed < delay:
                    time.sleep(delay - elapsed)

        cap.release()
        time.sleep(0.2)


def make_handler(store: FrameStore, stream_path: str):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            print(f"{self.client_address[0]} - {fmt % args}", flush=True)

        def do_GET(self):
            if self.path == "/":
                body = (
                    "<!doctype html><html><head><title>Remote Processing</title></head>"
                    f"<body style='margin:0;background:#050608'><img src='{stream_path}' "
                    "style='width:100vw;height:100vh;object-fit:contain'></body></html>"
                ).encode()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path != stream_path:
                self.send_error(HTTPStatus.NOT_FOUND)
                return

            self.send_response(HTTPStatus.OK)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Pragma", "no-cache")
            self.send_header("Connection", "close")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            last_seq = 0
            while True:
                seq, jpeg = store.wait_next(last_seq, timeout=2.0)
                if seq == last_seq or not jpeg:
                    continue
                last_seq = seq
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    return

    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input",
        default="http://192.168.127.10:8080/raw.mjpg",
        help="MJPEG input URL from the 10 device web UI",
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--path", default="/processed.mjpg")
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--max-fps", type=float, default=15.0)
    args = parser.parse_args()
    args.jpeg_quality = max(1, min(100, args.jpeg_quality))

    stop_event = threading.Event()
    store = FrameStore()
    capture_thread = threading.Thread(
        target=capture_loop,
        args=(args, store, stop_event),
        daemon=True,
    )
    capture_thread.start()

    server = ThreadingHTTPServer((args.host, args.port), make_handler(store, args.path))

    def stop(signum, frame):
        del signum, frame
        stop_event.set()
        server.shutdown()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    print(f"serving processed stream: http://{args.host}:{args.port}{args.path}", flush=True)
    server.serve_forever()
    server.server_close()
    stop_event.set()


if __name__ == "__main__":
    main()
