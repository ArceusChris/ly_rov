#!/usr/bin/env python3
"""Segment the white pipe in pipe2.mp4 and overlay rov_pipe_tracker commands."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Observation:
    valid: bool = False
    cx: float = 0.0
    cy: float = 0.0
    angle_rad: float = 0.0
    width: int = 0
    height: int = 0
    pixels: int = 0


@dataclass
class Command:
    x: int = 0
    y: int = 0
    z: int = 500
    r: int = 0
    x_error: float = 0.0
    yaw_error: float = 0.0


def clamp_axis(value: float) -> int:
    return int(max(-1000, min(1000, round(value))))


def wrap_half_pi(angle: float) -> float:
    half_pi = math.pi * 0.5
    while angle > half_pi:
        angle -= math.pi
    while angle < -half_pi:
        angle += math.pi
    return angle


def segment_white_pipe(
    frame: np.ndarray,
    min_area: int,
    min_red: int,
    min_green: int,
    min_blue: int,
    max_channel_diff: int,
    max_red_blue_diff: int,
) -> np.ndarray:
    blue = frame[:, :, 0]
    green = frame[:, :, 1]
    red = frame[:, :, 2]
    channel_diff = np.maximum.reduce([red, green, blue]) - np.minimum.reduce([red, green, blue])
    red_blue_diff = red.astype(np.int16) - blue.astype(np.int16)

    mask = (
        (red > min_red)
        & (green > min_green)
        & (blue > min_blue)
        & (channel_diff <= max_channel_diff)
        & (red_blue_diff <= max_red_blue_diff)
    ).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (9, 25)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 9)))

    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    best_label = 0
    best_score = 0.0
    height, width = mask.shape
    for label in range(1, count):
        x, y, w, h, area = stats[label]
        if area < min_area or h < height * 0.18 or w < 8:
            continue
        aspect = h / max(1, w)
        if aspect < 1.0:
            continue
        center_bias = 1.0 - abs((x + w * 0.5) - width * 0.5) / max(1.0, width * 0.5)
        score = area * (0.8 + min(aspect, 5.0) * 0.2) * (0.7 + max(0.0, center_bias) * 0.3)
        if score > best_score:
            best_score = score
            best_label = label

    if best_label == 0:
        return np.zeros_like(mask)
    return np.where(labels == best_label, 255, 0).astype(np.uint8)


def observe(mask: np.ndarray, min_pixels: int) -> Observation:
    height, width = mask.shape
    ys, xs = np.nonzero(mask)
    count = int(xs.size)
    obs = Observation(width=width, height=height, pixels=count)
    if count < min_pixels:
        return obs

    obs.cx = float(xs.mean())
    obs.cy = float(ys.mean())
    dx = xs.astype(np.float64) - obs.cx
    dy = ys.astype(np.float64) - obs.cy
    xx = float(np.dot(dx, dx))
    yy = float(np.dot(dy, dy))
    xy = float(np.dot(dx, dy))
    obs.angle_rad = 0.5 * math.atan2(2.0 * xy, xx - yy)
    obs.valid = True
    return obs


def command_from_observation(args: argparse.Namespace, obs: Observation) -> Command:
    if not obs.valid:
        return Command()

    x_error = (obs.cx - (obs.width - 1) * 0.5) / max(1.0, obs.width * 0.5)
    desired = math.radians(args.desired_angle_deg)
    yaw_error = wrap_half_pi(obs.angle_rad - desired) / (math.pi * 0.5)
    return Command(
        x=clamp_axis(args.forward_axis),
        y=clamp_axis(args.lateral_sign * args.lateral_gain * x_error),
        z=int(max(0, min(1000, args.z_axis))),
        r=clamp_axis(args.yaw_sign * args.yaw_gain * yaw_error),
        x_error=x_error,
        yaw_error=yaw_error,
    )


def draw_bar(frame: np.ndarray, name: str, value: int, y: int, z_axis: bool = False) -> None:
    x0, w, h = 18, 180, 12
    cv2.putText(frame, name, (x0, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    bx = x0 + 34
    cv2.rectangle(frame, (bx, y), (bx + w, y + h), (45, 45, 45), -1)
    if z_axis:
        fill = int(w * max(0, min(1000, value)) / 1000)
        cv2.rectangle(frame, (bx, y), (bx + fill, y + h), (0, 210, 255), -1)
    else:
        mid = bx + w // 2
        end = mid + int((w // 2) * max(-1000, min(1000, value)) / 1000)
        cv2.line(frame, (mid, y), (mid, y + h), (230, 230, 230), 1)
        cv2.rectangle(frame, (min(mid, end), y), (max(mid, end), y + h), (0, 210, 255), -1)
    cv2.rectangle(frame, (bx, y), (bx + w, y + h), (220, 220, 220), 1)
    cv2.putText(frame, f"{value:4d}", (bx + w + 8, y + 11), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)


def draw_overlay(frame: np.ndarray, mask: np.ndarray, obs: Observation, cmd: Command) -> np.ndarray:
    out = frame.copy()
    tint = np.zeros_like(out)
    tint[:, :, 1] = mask
    out = cv2.addWeighted(out, 1.0, tint, 0.28, 0)

    cv2.line(out, (obs.width // 2, 0), (obs.width // 2, obs.height), (180, 180, 180), 1)
    if obs.valid:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(out, contours, -1, (0, 255, 0), 2)
        center = (int(round(obs.cx)), int(round(obs.cy)))
        cv2.circle(out, center, 5, (0, 255, 255), -1)
        length = 90
        vx = math.cos(obs.angle_rad) * length
        vy = math.sin(obs.angle_rad) * length
        p1 = (int(round(obs.cx - vx)), int(round(obs.cy - vy)))
        p2 = (int(round(obs.cx + vx)), int(round(obs.cy + vy)))
        cv2.line(out, p1, p2, (255, 255, 0), 3)
        state = f"LOCK  pixels={obs.pixels}  angle={math.degrees(obs.angle_rad):.1f} deg"
    else:
        state = "LOST"

    cv2.rectangle(out, (8, 8), (330, 118), (0, 0, 0), -1)
    cv2.putText(out, state, (18, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(
        out,
        f"manual_control  x={cmd.x}  y={cmd.y}  z={cmd.z}  r={cmd.r}",
        (18, 56),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        out,
        f"x_error={cmd.x_error:+.3f}  yaw_error={cmd.yaw_error:+.3f}",
        (18, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    draw_bar(out, "X", cmd.x, 134)
    draw_bar(out, "Y", cmd.y, 154)
    draw_bar(out, "Z", cmd.z, 174, True)
    draw_bar(out, "R", cmd.r, 194)
    return out


def self_test() -> None:
    assert clamp_axis(1200) == 1000
    assert clamp_axis(-1200) == -1000
    assert abs(wrap_half_pi(math.pi) - 0.0) < 1e-9
    mask = np.zeros((80, 80), np.uint8)
    cv2.rectangle(mask, (35, 5), (45, 75), 255, -1)
    obs = observe(mask, 10)
    assert obs.valid
    assert abs(obs.cx - 40.0) < 0.1
    assert abs(abs(obs.angle_rad) - math.pi * 0.5) < 0.05


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default="pipe2.mp4")
    parser.add_argument("--output", default="pipe2_tracked.mp4")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--max-frames", type=int, default=0, help="0 means all frames")
    parser.add_argument("--min-area", type=int, default=1800)
    parser.add_argument("--min-pixels", type=int, default=80)
    parser.add_argument("--min-red", type=int, default=150)
    parser.add_argument("--min-green", type=int, default=150)
    parser.add_argument("--min-blue", type=int, default=150)
    parser.add_argument("--max-channel-diff", type=int, default=45)
    parser.add_argument("--max-red-blue-diff", type=int, default=55)
    parser.add_argument("--desired-angle-deg", type=float, default=90.0)
    parser.add_argument("--forward-axis", type=int, default=250)
    parser.add_argument("--z-axis", type=int, default=500)
    parser.add_argument("--lateral-gain", type=float, default=450.0)
    parser.add_argument("--yaw-gain", type=float, default=450.0)
    parser.add_argument("--lateral-sign", type=float, default=1.0)
    parser.add_argument("--yaw-sign", type=float, default=1.0)
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test:
        self_test()
        print("self-test ok")
        return 0

    cap = cv2.VideoCapture(args.input)
    if not cap.isOpened():
        raise SystemExit(f"failed to open {args.input}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if args.start_frame:
        cap.set(cv2.CAP_PROP_POS_FRAMES, args.start_frame)

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(args.output, fourcc, fps, (width, height))
    if not writer.isOpened():
        raise SystemExit(f"failed to create {args.output}")

    written = 0
    frame_index = args.start_frame
    while True:
        if args.max_frames and written >= args.max_frames:
            break
        ok, frame = cap.read()
        if not ok:
            break

        mask = segment_white_pipe(
            frame,
            args.min_area,
            args.min_red,
            args.min_green,
            args.min_blue,
            args.max_channel_diff,
            args.max_red_blue_diff,
        )
        obs = observe(mask, args.min_pixels)
        cmd = command_from_observation(args, obs)
        out = draw_overlay(frame, mask, obs, cmd)
        writer.write(out)

        written += 1
        frame_index += 1
        if written % 300 == 0:
            print(f"processed {written} frames ({frame_index}/{total})", flush=True)
        if args.display:
            cv2.imshow("pipe tracker", out)
            if cv2.waitKey(1) in (27, ord("q")):
                break

    cap.release()
    writer.release()
    if args.display:
        cv2.destroyAllWindows()
    print(f"wrote {args.output} ({written} frames)", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
