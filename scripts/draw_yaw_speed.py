import argparse
import time
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np


def load_yaw_speed_csv(path: str, gimbal_id: Optional[int]):
    t_s = []
    target = []
    actual = []
    give_current = []

    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) != 5:
                continue
            try:
                t = float(parts[0])
                gid = int(parts[1])
                target_speed = float(parts[2])
                actual_speed = float(parts[3])
                current = int(parts[4])
            except ValueError:
                continue

            if gimbal_id is not None and gid != gimbal_id:
                continue

            t_s.append(t)
            target.append(target_speed)
            actual.append(actual_speed)
            give_current.append(current)

    return np.array(t_s), np.array(target), np.array(actual), np.array(give_current)


def clear_log_file(path: str):
    Path(path).write_text("", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(
        description="Plot yaw speed target/actual/current from log/yaw_speed.txt"
    )
    parser.add_argument("--file", default="../log/yaw_speed.txt", help="path to yaw speed log file")
    parser.add_argument("--id", type=int, default=None, help="filter by gimbal id (optional)")
    parser.add_argument("--deg", action="store_true", help="plot deg/s (default: rad/s)")
    args = parser.parse_args()

    try:
        t_s, target, actual, give_current = load_yaw_speed_csv(args.file, args.id)
    except FileNotFoundError:
        print(f"错误：未找到日志文件 {args.file}")
        return 1

    if t_s.size == 0:
        if args.id is None:
            print(f"未在日志文件 {args.file} 中检测到任何有效数据。")
        else:
            print(f"未在日志文件 {args.file} 中检测到任何有效数据 (id={args.id})。")
        return 1

    if args.deg:
        target = target * 180.0 / np.pi
        actual = actual * 180.0 / np.pi
        speed_unit = "deg/s"
    else:
        speed_unit = "rad/s"

    title = f"Yaw Speed Plot ({args.file})"
    if args.id is not None:
        title += f" [id={args.id}]"

    fig, (ax_speed, ax_current) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    fig.suptitle(title)

    ax_speed.plot(t_s, target, label=f"target ({speed_unit})", color="tab:red", alpha=0.9)
    ax_speed.plot(t_s, actual, label=f"actual ({speed_unit})", color="tab:blue", alpha=0.9)
    ax_speed.set_ylabel(f"speed ({speed_unit})")
    ax_speed.set_title("Yaw Speed Target vs Actual")
    ax_speed.legend(loc="upper right")
    ax_speed.grid(True)

    ax_current.plot(t_s, give_current, label="give_current", color="tab:green", alpha=0.9)
    ax_current.set_xlabel("t (s)")
    ax_current.set_ylabel("current")
    ax_current.set_title("Yaw Motor Give Current")
    ax_current.legend(loc="upper right")
    ax_current.grid(True)

    fig.tight_layout(rect=[0, 0, 1, 0.96])

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = f"yaw_speed_plot_{timestamp}.png"
    fig.savefig(filename, bbox_inches="tight", dpi=300)
    clear_log_file(args.file)
    print(f"\n✅ 图表已成功保存为本地文件: {filename}")
    print(f"🧹 已清空日志文件: {args.file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
