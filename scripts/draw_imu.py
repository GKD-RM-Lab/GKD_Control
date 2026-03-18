import argparse
import time
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np


def load_imu_csv(path: str, gimbal_id: Optional[int]):
    t_s = []
    pitch_set = []
    yaw_set = []
    pitch = []
    yaw = []
    pitch_rate = []
    yaw_rate = []
    has_set = False
    has_rate = False

    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) not in (4, 6, 8):
                continue
            try:
                t = float(parts[0])
                gid = int(parts[1])
                if len(parts) == 8:
                    ps = float(parts[2])
                    ys = float(parts[3])
                    p = float(parts[4])
                    y = float(parts[5])
                    pr = float(parts[6])
                    yr = float(parts[7])
                else:
                    p = float(parts[2])
                    y = float(parts[3])
                if len(parts) == 6:
                    pr = float(parts[4])
                    yr = float(parts[5])
            except ValueError:
                continue

            if gimbal_id is not None and gid != gimbal_id:
                continue

            t_s.append(t)
            pitch.append(p)
            yaw.append(y)
            if len(parts) == 8:
                if not has_set and len(t_s) > 1:
                    pitch_set.extend([np.nan] * (len(t_s) - 1))
                    yaw_set.extend([np.nan] * (len(t_s) - 1))
                pitch_set.append(ps)
                yaw_set.append(ys)
                has_set = True
            elif has_set:
                pitch_set.append(np.nan)
                yaw_set.append(np.nan)

            if len(parts) in (6, 8):
                if not has_rate and len(t_s) > 1:
                    pitch_rate.extend([np.nan] * (len(t_s) - 1))
                    yaw_rate.extend([np.nan] * (len(t_s) - 1))
                pitch_rate.append(pr)
                yaw_rate.append(yr)
                has_rate = True
            elif has_rate:
                pitch_rate.append(np.nan)
                yaw_rate.append(np.nan)

    pitch_set_arr = np.array(pitch_set) if has_set else None
    yaw_set_arr = np.array(yaw_set) if has_set else None
    if has_rate:
        return (
            np.array(t_s),
            pitch_set_arr,
            yaw_set_arr,
            np.array(pitch),
            np.array(yaw),
            np.array(pitch_rate),
            np.array(yaw_rate),
            has_set,
            True,
        )
    return np.array(t_s), pitch_set_arr, yaw_set_arr, np.array(pitch), np.array(yaw), None, None, has_set, False


def clear_log_file(path: str):
    Path(path).write_text("", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(
        description="Plot IMU pitch/yaw, setpoint and rate from log/imu.txt"
    )
    parser.add_argument("--file", default="../log/imu.txt", help="path to imu log file")
    parser.add_argument("--id", type=int, default=None, help="filter by gimbal id (optional)")
    parser.add_argument("--deg", action="store_true", help="plot degrees (default: radians)")
    args = parser.parse_args()

    try:
        (
            t_s,
            pitch_set,
            yaw_set,
            pitch,
            yaw,
            pitch_rate,
            yaw_rate,
            has_set,
            has_rate,
        ) = load_imu_csv(args.file, args.id)
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
        if has_set:
            pitch_set = pitch_set * 180.0 / np.pi
            yaw_set = yaw_set * 180.0 / np.pi
        pitch = pitch * 180.0 / np.pi
        yaw = yaw * 180.0 / np.pi
        if has_rate:
            pitch_rate = pitch_rate * 180.0 / np.pi
            yaw_rate = yaw_rate * 180.0 / np.pi
        unit = "deg"
        rate_unit = "deg/s"
    else:
        unit = "rad"
        rate_unit = "rad/s"

    title = f"IMU Plot ({args.file})"
    if args.id is not None:
        title += f" [id={args.id}]"

    if has_set and has_rate:
        fig, (ax_pitch, ax_yaw, ax_rate) = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    elif has_set:
        fig, (ax_pitch, ax_yaw) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        ax_rate = None
    elif has_rate:
        fig, (ax_angle, ax_rate) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    else:
        fig = plt.figure(figsize=(10, 5))
        ax_angle = fig.add_subplot(1, 1, 1)
        ax_rate = None

    fig.suptitle(title)

    if has_set:
        ax_pitch.plot(t_s, pitch_set, label=f"pitch_set ({unit})", color="tab:green", alpha=0.9)
        ax_pitch.plot(t_s, pitch, label=f"pitch ({unit})", color="tab:blue", alpha=0.9)
        ax_pitch.set_ylabel(f"pitch ({unit})")
        ax_pitch.set_title("Pitch vs Pitch Setpoint")
        ax_pitch.legend(loc="upper right")
        ax_pitch.grid(True)

        ax_yaw.plot(t_s, yaw_set, label=f"yaw_set ({unit})", color="tab:red", alpha=0.9)
        ax_yaw.plot(t_s, yaw, label=f"yaw ({unit})", color="tab:orange", alpha=0.9)
        ax_yaw.set_xlabel("t (s)")
        ax_yaw.set_ylabel(f"yaw ({unit})")
        ax_yaw.set_title("Yaw vs Yaw Setpoint")
        ax_yaw.legend(loc="upper right")
        ax_yaw.grid(True)
    else:
        ax_angle.plot(t_s, pitch, label=f"pitch ({unit})", color="tab:blue", alpha=0.9)
        ax_angle.plot(t_s, yaw, label=f"yaw ({unit})", color="tab:orange", alpha=0.9)
        ax_angle.set_xlabel("t (s)")
        ax_angle.set_ylabel(f"angle ({unit})")
        ax_angle.legend(loc="upper right")
        ax_angle.grid(True)

    if has_rate and ax_rate is not None:
        ax_rate.plot(t_s, pitch_rate, label=f"pitch_rate ({rate_unit})", color="tab:green", alpha=0.9)
        ax_rate.plot(t_s, yaw_rate, label=f"yaw_rate ({rate_unit})", color="tab:red", alpha=0.9)
        ax_rate.set_title("IMU Pitch/Yaw Rate")
        ax_rate.set_xlabel("t (s)")
        ax_rate.set_ylabel(f"rate ({rate_unit})")
        ax_rate.legend(loc="upper right")
        ax_rate.grid(True)

    fig.tight_layout(rect=[0, 0, 1, 0.96])

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = f"imu_plot_{timestamp}.png"
    fig.savefig(filename, bbox_inches="tight", dpi=300)
    clear_log_file(args.file)
    print(f"\n✅ 图表已成功保存为本地文件: {filename}")
    print(f"🧹 已清空日志文件: {args.file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
