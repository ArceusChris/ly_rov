#!/usr/bin/env python3
import argparse
import json
import signal
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np


class StreamStore:
    def __init__(self):
        self.cond = threading.Condition()
        self.frames = {
            "/processed.mjpg": b"",
            "/mask.mjpg": b"",
        }
        self.seq = {
            "/processed.mjpg": 0,
            "/mask.mjpg": 0,
        }
        self.metadata = {}

    def update(self, processed_jpeg: bytes, mask_jpeg: bytes, metadata: dict):
        with self.cond:
            self.frames["/processed.mjpg"] = processed_jpeg
            self.frames["/mask.mjpg"] = mask_jpeg
            self.seq["/processed.mjpg"] += 1
            self.seq["/mask.mjpg"] += 1
            self.metadata = metadata
            self.cond.notify_all()

    def wait_next(self, path: str, last_seq: int, timeout: float):
        with self.cond:
            self.cond.wait_for(lambda: self.seq[path] != last_seq, timeout=timeout)
            return self.seq[path], self.frames[path]

    def metadata_json(self):
        with self.cond:
            return json.dumps(self.metadata, separators=(",", ":")).encode()


def segment_white_pipe(frame, args):
    blue, green, red = cv2.split(frame)
    red_ok = red > args.min_red
    green_ok = green > args.min_green
    blue_ok = blue > args.min_blue

    max_channel = np.maximum(np.maximum(red, green), blue)
    min_channel = np.minimum(np.minimum(red, green), blue)
    neutral_ok = (max_channel.astype(np.int16) - min_channel.astype(np.int16)) <= args.max_channel_diff
    not_wood = (red.astype(np.int16) - blue.astype(np.int16)) <= args.max_red_blue_diff

    mask = (red_ok & green_ok & blue_ok & neutral_ok & not_wood).astype(np.uint8) * 255
    threshold_pixels = int(cv2.countNonZero(mask))

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (9, 25)),
    )
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (5, 9)),
    )
    morph_pixels = int(cv2.countNonZero(mask))

    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    best_label = 0
    best_score = 0.0
    largest = {
        "x": 0,
        "y": 0,
        "w": 0,
        "h": 0,
        "area": 0,
        "aspect": 0.0,
    }
    rejected_area = 0
    rejected_min_aspect = 0
    rejected_max_aspect = 0
    height, width = frame.shape[:2]

    for label in range(1, count):
        x, y, w, h, area = [int(v) for v in stats[label, :5]]
        aspect = float(h) / max(1, w)
        if area > largest["area"]:
            largest = {
                "x": x,
                "y": y,
                "w": w,
                "h": h,
                "area": area,
                "aspect": aspect,
            }
        if area < args.min_area or h < height * args.min_height_ratio or w < 8:
            rejected_area += 1
            continue
        if aspect < args.min_aspect:
            rejected_min_aspect += 1
            continue
        if args.max_aspect > 0.0 and aspect > args.max_aspect:
            rejected_max_aspect += 1
            continue

        center = x + w * 0.5
        center_bias = 1.0 - abs(center - width * 0.5) / max(1.0, width * 0.5)
        score = area * (0.8 + min(aspect, 5.0) * 0.2) * (0.7 + max(0.0, center_bias) * 0.3)
        if score > best_score:
            best_score = score
            best_label = label

    best = np.zeros(mask.shape, dtype=np.uint8)
    if best_label != 0:
        best[labels == best_label] = 255
        best //= 255

    selected_pixels = int(cv2.countNonZero(best))
    if args.erode_kernel_size > 1 and selected_pixels > 0:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (args.erode_kernel_size, args.erode_kernel_size),
        )
        best = cv2.erode(best, kernel)
    output_pixels = int(cv2.countNonZero(best))

    metadata = {
        "width": width,
        "height": height,
        "threshold_pixels": threshold_pixels,
        "morph_pixels": morph_pixels,
        "components": max(0, count - 1),
        "rejected_area": rejected_area,
        "rejected_min_aspect": rejected_min_aspect,
        "rejected_max_aspect": rejected_max_aspect,
        "largest": largest,
        "best_label": best_label,
        "selected_pixels": selected_pixels,
        "output_pixels": output_pixels,
        "timestamp": time.time(),
    }
    return best, metadata


def draw_overlay(frame, mask):
    binary = mask > 0
    tint = frame.copy()
    tint[binary] = (0, 255, 80)
    overlay = cv2.addWeighted(frame, 0.68, tint, 0.32, 0.0)
    contours, _ = cv2.findContours((binary.astype(np.uint8) * 255), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay, contours, -1, (0, 0, 255), 2)
    return overlay


def encode_jpeg(image, quality):
    ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        return b""
    return encoded.tobytes()


def capture_loop(args, store: StreamStore, stop_event: threading.Event):
    delay = 0.0 if args.max_fps <= 0.0 else 1.0 / args.max_fps
    while not stop_event.is_set():
        cap = cv2.VideoCapture(args.input)
        if not cap.isOpened():
            print(f"failed to open input stream: {args.input}", flush=True)
            time.sleep(args.reconnect_delay)
            continue

        print(f"reading input stream: {args.input}", flush=True)
        while not stop_event.is_set():
            started = time.monotonic()
            ok, frame = cap.read()
            if not ok or frame is None:
                print("input stream ended, reconnecting", flush=True)
                break

            mask, metadata = segment_white_pipe(frame, args)
            overlay = draw_overlay(frame, mask)
            processed_jpeg = encode_jpeg(overlay, args.jpeg_quality)
            mask_jpeg = encode_jpeg(mask * 255, args.mask_jpeg_quality)
            if processed_jpeg and mask_jpeg:
                store.update(processed_jpeg, mask_jpeg, metadata)

            if delay > 0.0:
                elapsed = time.monotonic() - started
                if elapsed < delay:
                    time.sleep(delay - elapsed)

        cap.release()
        time.sleep(args.reconnect_delay)


def make_handler(store: StreamStore):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            print(f"{self.client_address[0]} - {fmt % args}", flush=True)

        def do_GET(self):
            if self.path == "/":
                body = (
                    "<!doctype html><html><head><title>Remote Segmentation</title></head>"
                    "<body style='margin:0;background:#050608;display:grid;grid-template-columns:1fr 1fr;height:100vh'>"
                    "<img src='/processed.mjpg' style='width:100%;height:100%;object-fit:contain'>"
                    "<img src='/mask.mjpg' style='width:100%;height:100%;object-fit:contain'>"
                    "</body></html>"
                ).encode()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path == "/metadata.json":
                body = store.metadata_json()
                self.send_response(HTTPStatus.OK)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path not in store.frames:
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
                seq, jpeg = store.wait_next(self.path, last_seq, timeout=2.0)
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
    parser.add_argument("--input", default="http://192.168.127.10:8080/raw.mjpg")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--mask-jpeg-quality", type=int, default=95)
    parser.add_argument("--max-fps", type=float, default=15.0)
    parser.add_argument("--reconnect-delay", type=float, default=1.0)
    parser.add_argument("--min-red", type=int, default=150)
    parser.add_argument("--min-green", type=int, default=150)
    parser.add_argument("--min-blue", type=int, default=150)
    parser.add_argument("--max-channel-diff", type=int, default=45)
    parser.add_argument("--max-red-blue-diff", type=int, default=55)
    parser.add_argument("--min-area", type=int, default=3000)
    parser.add_argument("--min-height-ratio", type=float, default=0.18)
    parser.add_argument("--min-aspect", type=float, default=0.5)
    parser.add_argument("--max-aspect", type=float, default=2.0)
    parser.add_argument("--erode-kernel-size", type=int, default=1)
    args = parser.parse_args()
    args.jpeg_quality = max(1, min(100, args.jpeg_quality))
    args.mask_jpeg_quality = max(1, min(100, args.mask_jpeg_quality))
    args.erode_kernel_size = max(1, args.erode_kernel_size)

    stop_event = threading.Event()
    store = StreamStore()
    capture_thread = threading.Thread(target=capture_loop, args=(args, store, stop_event), daemon=True)
    capture_thread.start()

    server = ThreadingHTTPServer((args.host, args.port), make_handler(store))

    def stop(signum, frame):
        del signum, frame
        stop_event.set()
        server.shutdown()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    print(f"serving remote segmentation on http://{args.host}:{args.port}", flush=True)
    server.serve_forever()
    server.server_close()
    stop_event.set()


if __name__ == "__main__":
    main()
