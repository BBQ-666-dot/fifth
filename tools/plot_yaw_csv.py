# -*- coding: utf-8 -*-
"""
Read a yaw_delta CSV produced by swd_yaw_plot.py and draw the history.

Run in PyCharm with the Anaconda interpreter (numpy + matplotlib).

Default:
    python plot_yaw_csv.py
    -> uses the newest csv in tools/logs/yaw_delta_*.csv

Examples:
    python plot_yaw_csv.py tools/logs/yaw_delta_20261024_191031.csv
    python plot_yaw_csv.py --start 2 --end 10
    python plot_yaw_csv.py --png history.png --no-show
    python plot_yaw_csv.py --png none
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent

try:
    import numpy as np
    import matplotlib

    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
except ImportError:
    print("缺少 Python 依赖，请先安装: pip install numpy matplotlib")
    raise


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Plot a yaw_delta history CSV written by swd_yaw_plot.py")
    parser.add_argument("csv", nargs="?", default=None,
                        help="CSV path; default: newest tools/logs/yaw_delta_*.csv")
    parser.add_argument("--start", type=float, default=None,
                        help="start time in seconds")
    parser.add_argument("--end", type=float, default=None,
                        help="end time in seconds")
    parser.add_argument("--figsize", default="20x8",
                        help="figure size in inches (default 20x8)")
    parser.add_argument("--dpi", type=int, default=160,
                        help="PNG resolution (default 160)")
    parser.add_argument("--png", default="auto",
                        help="'auto' saves next to the CSV, 'none' disables, "
                             "or a custom PNG path")
    parser.add_argument("--no-show", action="store_true",
                        help="do not open the matplotlib window")
    return parser.parse_args(argv)


def newest_csv():
    logs = TOOLS_DIR / "logs"
    files = sorted(logs.glob("yaw_delta_*.csv"),
                   key=lambda path: path.stat().st_mtime,
                   reverse=True)
    if not files:
        raise SystemExit(f"没有找到 CSV: {logs}\\yaw_delta_*.csv")
    return files[0]


def read_csv(path):
    with open(path, "r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
    if not rows:
        raise SystemExit(f"CSV 是空的: {path}")

    def column(*names):
        for name in names:
            if name in rows[0]:
                values = []
                for row in rows:
                    text = row.get(name, "")
                    try:
                        values.append(float(text) if text != "" else np.nan)
                    except ValueError:
                        values.append(np.nan)
                return np.asarray(values, dtype=float)
        return None

    data = {
        "time_s": column("time_s", "time"),
        "yaw_delta_deg": column("yaw_delta_deg", "yaw_delta", "imu_yaw_deg"),
        "gimbal_wz_rad_s": column("gimbal_wz_rad_s"),
        "gimbal_cmd_rad_s": column("gimbal_cmd_rad_s"),
        "chassis_wz_ref_rad_s": column("chassis_wz_ref_rad_s", "wz_rad_s"),
        "chassis_wz_est_rad_s": column("chassis_wz_est_rad_s"),
    }
    if data["time_s"] is None:
        raise SystemExit("CSV 里没有 time_s 列")
    return data


def finite(values):
    if values is None:
        return None
    values = np.asarray(values, dtype=float)
    return values[np.isfinite(values)]


def summary(name, values, unit):
    values = finite(values)
    if values is None or len(values) == 0:
        return
    print(f"{name:20s} min={values.min():8.3f}  max={values.max():8.3f}  "
          f"mean={values.mean():8.3f} {unit}")


def main(argv=None):
    args = parse_args(argv)
    try:
        fig_w, fig_h = (float(part) for part in args.figsize.lower().split("x", 1))
    except ValueError:
        raise SystemExit("--figsize 格式应为 宽x高，例如 20x8")
    if fig_w <= 0 or fig_h <= 0:
        raise SystemExit("--figsize 必须大于 0")
    if args.dpi <= 0:
        raise SystemExit("--dpi 必须大于 0")

    path = Path(args.csv) if args.csv else newest_csv()
    if not path.exists():
        raise SystemExit(f"CSV 不存在: {path}")

    data = read_csv(path)
    t = data["time_s"]
    mask = np.isfinite(t)
    if args.start is not None:
        mask &= t >= args.start
    if args.end is not None:
        mask &= t <= args.end
    if not np.any(mask):
        raise SystemExit("时间过滤后没有数据")

    t = t[mask]
    yaw = data["yaw_delta_deg"][mask] if data["yaw_delta_deg"] is not None else None
    if yaw is None:
        raise SystemExit("CSV 里没有 yaw_delta 列")
    gimbal = data["gimbal_wz_rad_s"][mask] if data["gimbal_wz_rad_s"] is not None else None
    gimbal_cmd = data["gimbal_cmd_rad_s"][mask] if data["gimbal_cmd_rad_s"] is not None else None
    chassis_ref = data["chassis_wz_ref_rad_s"][mask] if data["chassis_wz_ref_rad_s"] is not None else None
    chassis_est = data["chassis_wz_est_rad_s"][mask] if data["chassis_wz_est_rad_s"] is not None else None

    print(f"CSV: {path}")
    print(f"samples: {len(t)}  duration: {t[-1] - t[0]:.3f} s")
    summary("yaw_delta", yaw, "deg")
    summary("gimbal_wz", gimbal, "rad/s")
    summary("gimbal_cmd", gimbal_cmd, "rad/s")
    summary("chassis_wz_ref", chassis_ref, "rad/s")
    summary("chassis_wz_est", chassis_est, "rad/s")

    if np.any(np.isfinite(yaw)):
        index = int(np.nanargmax(np.abs(yaw)))
        print(f"max |yaw_delta|: {yaw[index]:.3f} deg at t={t[index]:.3f} s")

    fig, (ax_angle, ax_rate) = plt.subplots(
        2, 1, sharex=True, figsize=(fig_w, fig_h))
    fig.canvas.manager.set_window_title(path.name)

    ax_angle.plot(t, yaw, color="#2f86cf", linewidth=1.4,
                  label="yaw_delta (deg)")
    ax_angle.axhline(0.0, color="k", linewidth=0.6, alpha=0.4)
    ax_angle.set_ylabel("yaw_delta (deg)")
    ax_angle.grid(True, alpha=0.35)
    ax_angle.legend(loc="upper left", fontsize=8)

    if gimbal is not None:
        ax_rate.plot(t, gimbal, color="#2dd4a7", linewidth=1.2,
                     label="gimbal actual")
    if gimbal_cmd is not None:
        ax_rate.plot(t, gimbal_cmd, color="#2dd4a7", linewidth=1.0,
                     linestyle="--", alpha=0.8, label="gimbal cmd")
    if chassis_ref is not None:
        ax_rate.plot(t, chassis_ref, color="#f4c542", linewidth=1.2,
                     label="chassis cmd")
    if chassis_est is not None:
        ax_rate.plot(t, chassis_est, color="#f2555a", linewidth=1.2,
                     label="chassis est")
    ax_rate.axhline(0.0, color="k", linewidth=0.6, alpha=0.4)
    ax_rate.set_xlabel("time (s)")
    ax_rate.set_ylabel("yaw rate (rad/s)")
    ax_rate.grid(True, alpha=0.35)
    ax_rate.legend(loc="upper right", fontsize=8)

    ax_angle.set_title(f"{path.name}  ({len(t)} samples)")
    fig.tight_layout()

    png_path = None
    if args.png.lower() != "none":
        png_path = (path.with_suffix(".png") if args.png == "auto"
                    else Path(args.png))
        png_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(png_path, dpi=args.dpi)
        print(f"PNG 已保存: {png_path}")

    if not args.no_show:
        plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())
