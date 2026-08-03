#!/usr/bin/env python3
"""HTTP smoke test for the MuJoCo VLA joint chunk API."""

import argparse
import json
import sys
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


ACTION_DIM = 16


def post_json(url, body, timeout):
    data = json.dumps(body).encode("utf-8")
    request = Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def linear_chunk(start, target, steps):
    return [
        [a + (b - a) * step / steps for a, b in zip(start, target)]
        for step in range(1, steps + 1)
    ]


def main():
    parser = argparse.ArgumentParser(description="Smoke test MuJoCo VLA HTTP API.")
    parser.add_argument("--base-url", default="http://127.0.0.1:8765")
    parser.add_argument("--arms", choices=("both", "left", "right"), default="right")
    parser.add_argument("--joint-index", type=int, default=8)
    parser.add_argument("--target-rad", type=float, default=0.01)
    parser.add_argument("--steps", type=int, default=5)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()

    if not 0 <= args.joint_index < ACTION_DIM:
        print("FAILED: joint-index must be 0..15", file=sys.stderr)
        return 2

    start = [0.0] * ACTION_DIM
    target = list(start)
    target[args.joint_index] = float(args.target_rad)
    body = {
        "command_type": "joint_chunk",
        "unit": "rad",
        "arms": args.arms,
        "fps": args.fps,
        "execute": bool(args.execute),
        "actions": linear_chunk(start, target, args.steps),
        "meta": {"source": "tests/mujoco_vla_http_smoke.py"},
    }
    url = args.base_url.rstrip("/") + "/api/mujoco/vla_joint_chunk"
    try:
        result = post_json(url, body, args.timeout)
    except HTTPError as exc:
        print(exc.read().decode("utf-8", errors="replace"), file=sys.stderr)
        return 1
    except (URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1

    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get("ok") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
