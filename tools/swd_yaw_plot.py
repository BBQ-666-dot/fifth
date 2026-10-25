# -*- coding: utf-8 -*-
"""
SWD live plot for chassis.yaw_delta and yaw rates (DAPLink / OpenOCD TCL).

Run this file directly in PyCharm (select an interpreter that has numpy and
matplotlib, e.g. the Anaconda interpreter). It will:

  1. attach to a running VS Code OpenOCD if one is available, otherwise
     start OpenOCD with openocd_wireless_dap.cfg;
  2. resolve the variables from the newest CMake ELF / Keil AXF with
     arm-none-eabi-gdb;
  3. read the variables through SWD in the background (OpenOCD TCL `mdb`);
  4. print them in the PyCharm console and draw a live curve.

Signals:
    chassis.yaw_delta                 angle difference (deg, left axis)
    gimbal_direct.feedback_vel.yaw    gimbal actual yaw rate (rad/s)
    gimbal_direct.yaw.set.vel         gimbal yaw rate command (rad/s)
    chassis.reference.wz              chassis yaw rate command (rad/s)
    chassis yaw rate estimate         gimbal_wz - d(yaw_delta)/dt (rad/s)

History:
    By default every run writes tools/logs/yaw_delta_YYYYmmdd_HHMMSS.csv and,
    when the script stops, also generates a wide history PNG next to the CSV.
    The CSV is flushed while running, so the historical waveform survives a
    closed plot or a stopped Python process. Use --no-csv to disable CSV and
    PNG, or --no-history-png to keep the CSV but skip the PNG.
    The live plot shows the last --window seconds; use --window 0 for all
    in-memory history.

Typical runs:
    python swd_yaw_plot.py
    python swd_yaw_plot.py --rate 50
    python swd_yaw_plot.py --window 0
    python swd_yaw_plot.py --cfg openocd_dap.cfg
    python swd_yaw_plot.py --demo --duration 5
    python swd_yaw_plot.py --check
"""

from __future__ import annotations

import argparse
import csv
import math
import struct
import subprocess
import sys
import time
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

try:
    import numpy as np  # noqa: F401  (kept for PyCharm dependency check)
    import matplotlib

    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
except ImportError as exc:  # pragma: no cover - PyCharm environment issue
    print("缺少 Python 依赖，请先安装: pip install numpy matplotlib")
    raise

import livewatch_plot as lw


DEG_PER_RAD = 180.0 / math.pi
DEFAULT_GIMBAL_WZ = "gimbal_direct.feedback_vel.yaw"
DEFAULT_GIMBAL_CMD = "gimbal_direct.yaw.set.vel"
DEFAULT_CHASSIS_WZ = "chassis.reference.wz"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="DAPLink/SWD live plot for chassis.yaw_delta and yaw rates")
    parser.add_argument("--angle", default="chassis.yaw_delta",
                        help="angle expression to plot in degrees")
    parser.add_argument("--gimbal-wz", default=DEFAULT_GIMBAL_WZ,
                        help="gimbal actual yaw rate expression")
    parser.add_argument("--gimbal-cmd", default=DEFAULT_GIMBAL_CMD,
                        help="gimbal yaw rate command expression")
    parser.add_argument("--chassis-wz", default=DEFAULT_CHASSIS_WZ,
                        help="chassis yaw rate command expression")
    parser.add_argument("--wz", default=None,
                        help="deprecated alias for --chassis-wz")
    parser.add_argument("--rate", type=float, default=20.0,
                        help="sample rate in Hz (default 20)")
    parser.add_argument("--window", type=float, default=10.0,
                        help="plot window in seconds; 0 = all history")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="stop after N seconds (0 = run until closed)")
    parser.add_argument("--lpf-alpha", type=float, default=0.6,
                        help="low-pass coefficient for chassis rate estimate")
    parser.add_argument("--elf", default=None,
                        help="ELF/AXF file; default: newest of CMake ELF and Keil AXF")
    parser.add_argument("--cfg", default="openocd_wireless_dap.cfg",
                        help="OpenOCD config when it has to be started")
    parser.add_argument("--tcl", default=None,
                        help="OpenOCD TCL port or host:port")
    parser.add_argument("--attach", action="store_true",
                        help="only attach to an existing OpenOCD TCL port")
    parser.add_argument("--standalone", action="store_true",
                        help="always start a private OpenOCD")
    parser.add_argument("--demo", action="store_true",
                        help="synthetic data, no hardware")
    parser.add_argument("--check", action="store_true",
                        help="resolve symbols and exit")
    parser.add_argument("--csv", default=None,
                        help="CSV output path (default: tools/logs/...)")
    parser.add_argument("--no-csv", action="store_true",
                        help="disable CSV history logging")
    parser.add_argument("--no-history-png", action="store_true",
                        help="do not generate the history PNG on exit")
    parser.add_argument("--history-figsize", default="20x8",
                        help="history PNG figure size (default 20x8)")
    parser.add_argument("--history-dpi", type=int, default=160,
                        help="history PNG resolution (default 160)")
    return parser.parse_args(argv)


def default_elf():
    candidates = [
        lw.PROJECT_DIR / "build" / "vscode-debug" / "standard_robot.elf",
        lw.PROJECT_DIR / "MDK-ARM" / "standard_tpye_c" / "standard_tpye_c.axf",
    ]
    existing = [path for path in candidates if path.exists()]
    if not existing:
        return candidates[0]
    return max(existing, key=lambda path: path.stat().st_mtime)


def default_csv_path():
    logs = TOOLS_DIR / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    return logs / time.strftime("yaw_delta_%Y%m%d_%H%M%S.csv")


def resolve_specs(args):
    gdb = lw.find_gdb()
    elf = Path(args.elf) if args.elf else default_elf()
    specs = {
        "angle": lw.resolve_expression(gdb, elf, args.angle),
        "gimbal_wz": lw.resolve_expression(gdb, elf, args.gimbal_wz),
        "gimbal_cmd": lw.resolve_expression(gdb, elf, args.gimbal_cmd),
        "chassis_wz": lw.resolve_expression(gdb, elf, args.chassis_wz),
    }
    print(f"ELF: {elf}")
    for name, spec in specs.items():
        print(f"  {name}: " + lw.describe_spec(spec))
    return specs


def open_connection(args):
    host = None
    port = None
    if args.tcl:
        host, port = lw.parse_host_port(args.tcl, lw.DEFAULT_TCL_PORT)
    elif args.attach:
        host, port = "127.0.0.1", lw.find_openocd_tcl_port()
    elif not args.standalone:
        try:
            host, port = "127.0.0.1", lw.find_openocd_tcl_port()
        except RuntimeError:
            pass

    owned_process = None
    if port is not None:
        client = lw.OpenOcdTcl(host, port)
        print(f"已连接 OpenOCD TCL {host}:{port}")
        try:
            client.exec("resume")
        except Exception:
            pass
        return client, owned_process

    openocd = lw.find_openocd()
    owned_process = lw.OpenOcdProcess(openocd, args.cfg)
    print(f"启动 OpenOCD: {args.cfg}")
    owned_process.start()
    client = lw.OpenOcdTcl("127.0.0.1", owned_process.tcl_port)
    try:
        client.exec("resume")
    except Exception:
        pass
    print("OpenOCD 已就绪，目标继续运行")
    return client, owned_process


def read_scalar(client, spec):
    raw = client.read_bytes(spec.addr, spec.size)
    return struct.unpack(spec.fmt, raw)[0]


def demo_values(t):
    phase = 2.0 * math.pi * 0.2 * t
    angle = math.radians(25.0 + 15.0 * math.sin(phase))
    gimbal_wz = 2.5 * math.sin(phase + 0.5)
    gimbal_cmd = 2.8 * math.sin(phase + 0.5)
    chassis_wz = 2.0 * math.sin(phase + 0.2)
    return angle, gimbal_wz, gimbal_cmd, chassis_wz


def main(argv=None):
    args = parse_args(argv)
    if args.rate <= 0:
        raise SystemExit("--rate 必须大于 0")
    if not (0.0 < args.lpf_alpha <= 1.0):
        raise SystemExit("--lpf-alpha 必须在 (0, 1] 内")
    if args.history_dpi <= 0:
        raise SystemExit("--history-dpi 必须大于 0")
    if args.window < 0:
        raise SystemExit("--window 不能小于 0")
    if args.wz:
        args.chassis_wz = args.wz

    specs = None
    if not args.demo:
        specs = resolve_specs(args)
        if args.check:
            return 0
    elif args.check:
        print("demo 模式无需解析符号")
        return 0

    csv_path = None if args.no_csv else (Path(args.csv) if args.csv
                                         else default_csv_path())
    csv_handle = None
    csv_writer = None
    csv_rows = 0
    if csv_path is not None:
        try:
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            csv_handle = open(csv_path, "w", newline="", encoding="utf-8")
            csv_writer = csv.writer(csv_handle)
            csv_writer.writerow([
                "time_s", "yaw_delta_deg", "gimbal_wz_rad_s",
                "gimbal_cmd_rad_s", "chassis_wz_ref_rad_s",
                "chassis_wz_est_rad_s"])
            csv_handle.flush()
            print(f"CSV 历史记录: {csv_path}")
        except OSError as exc:
            print(f"CSV 打开失败，继续运行但不保存: {exc}")
            csv_handle = None
            csv_writer = None

    client = None
    owned_process = None
    hist_t = []
    hist_angle = []
    hist_gimbal = []
    hist_gimbal_cmd = []
    hist_chassis_ref = []
    hist_chassis_est = []
    view_start = 0

    fig, ax = plt.subplots(figsize=(11, 5))
    fig.canvas.manager.set_window_title("SWD yaw_delta / yaw rates")
    line_angle, = ax.plot([], [], color="#2f86cf", linewidth=1.4,
                          label=f"{args.angle} (deg)")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("yaw_delta (deg)")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper left", fontsize=8)

    ax_rate = ax.twinx()
    line_gimbal, = ax_rate.plot([], [], color="#2dd4a7", linewidth=1.2,
                                label=f"{args.gimbal_wz} (gimbal actual)")
    line_gimbal_cmd, = ax_rate.plot([], [], color="#2dd4a7", linewidth=1.0,
                                    linestyle="--", alpha=0.8,
                                    label=f"{args.gimbal_cmd} (gimbal cmd)")
    line_chassis_ref, = ax_rate.plot([], [], color="#f4c542", linewidth=1.2,
                                     label=f"{args.chassis_wz} (chassis cmd)")
    line_chassis_est, = ax_rate.plot([], [], color="#f2555a", linewidth=1.2,
                                     label="chassis actual estimate")
    ax_rate.set_ylabel("yaw rate (rad/s)")
    ax_rate.legend(loc="upper right", fontsize=8)

    plt.ion()
    plt.show(block=False)

    start = time.time()
    last_print = 0.0
    errors = 0
    prev_angle = None
    prev_sample_t = None
    chassis_est = 0.0

    print("列: t(s)  yaw_delta(deg)  gimbal_wz  gimbal_cmd  "
          "chassis_wz_ref  chassis_wz_est (rad/s)")

    try:
        if not args.demo:
            client, owned_process = open_connection(args)

        while True:
            now = time.time()
            t = now - start
            if args.demo:
                angle, gimbal_wz, gimbal_cmd, chassis_wz = demo_values(t)
            else:
                try:
                    angle = read_scalar(client, specs["angle"])
                    gimbal_wz = read_scalar(client, specs["gimbal_wz"])
                    gimbal_cmd = read_scalar(client, specs["gimbal_cmd"])
                    chassis_wz = read_scalar(client, specs["chassis_wz"])
                    errors = 0
                except Exception as exc:
                    errors += 1
                    if errors == 1 or errors % 20 == 0:
                        print(f"SWD 读取失败({errors}): {exc}")
                    if errors >= 100:
                        print("连续读取失败，退出")
                        break
                    time.sleep(1.0 / args.rate)
                    continue

            if prev_angle is None:
                chassis_est = 0.0
            else:
                dt_sample = t - prev_sample_t
                if dt_sample > 1e-6:
                    delta = angle - prev_angle
                    while delta > math.pi:
                        delta -= 2.0 * math.pi
                    while delta < -math.pi:
                        delta += 2.0 * math.pi
                    raw_rate = delta / dt_sample
                    est_raw = gimbal_wz - raw_rate
                    chassis_est += args.lpf_alpha * (est_raw - chassis_est)
            prev_angle = angle
            prev_sample_t = t

            angle_deg = angle * DEG_PER_RAD
            hist_t.append(t)
            hist_angle.append(angle_deg)
            hist_gimbal.append(gimbal_wz)
            hist_gimbal_cmd.append(gimbal_cmd)
            hist_chassis_ref.append(chassis_wz)
            hist_chassis_est.append(chassis_est)

            if csv_writer is not None:
                csv_writer.writerow([
                    f"{t:.6f}", f"{angle_deg:.6f}", f"{gimbal_wz:.6f}",
                    f"{gimbal_cmd:.6f}", f"{chassis_wz:.6f}",
                    f"{chassis_est:.6f}"])
                csv_rows += 1
                if csv_rows % 20 == 0:
                    csv_handle.flush()

            if args.window > 0:
                cutoff = t - args.window
                while (view_start < len(hist_t) - 1
                       and hist_t[view_start] < cutoff):
                    view_start += 1
                x0 = max(0.0, cutoff)
                x1 = max(args.window, t + 0.1)
            else:
                view_start = 0
                x0 = 0.0
                x1 = max(1.0, t + 0.1)

            view_t = hist_t[view_start:]
            view_angle = hist_angle[view_start:]
            view_gimbal = hist_gimbal[view_start:]
            view_gimbal_cmd = hist_gimbal_cmd[view_start:]
            view_chassis_ref = hist_chassis_ref[view_start:]
            view_chassis_est = hist_chassis_est[view_start:]

            line_angle.set_data(view_t, view_angle)
            line_gimbal.set_data(view_t, view_gimbal)
            line_gimbal_cmd.set_data(view_t, view_gimbal_cmd)
            line_chassis_ref.set_data(view_t, view_chassis_ref)
            line_chassis_est.set_data(view_t, view_chassis_est)

            ax.set_xlim(x0, x1)
            lo, hi = min(view_angle), max(view_angle)
            span = max(hi - lo, 1.0)
            ax.set_ylim(lo - 0.1 * span, hi + 0.1 * span)

            rate_values = (view_gimbal + view_gimbal_cmd
                           + view_chassis_ref + view_chassis_est)
            rlo, rhi = min(rate_values), max(rate_values)
            rspan = max(rhi - rlo, 0.5)
            ax_rate.set_ylim(rlo - 0.1 * rspan, rhi + 0.1 * rspan)

            try:
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
            except Exception:
                pass

            if now - last_print >= 0.5:
                print(f"t={t:7.2f}s  yaw_delta={angle_deg:7.2f} deg  "
                      f"gimbal_wz={gimbal_wz:7.3f}  "
                      f"gimbal_cmd={gimbal_cmd:7.3f}  "
                      f"chassis_ref={chassis_wz:7.3f}  "
                      f"chassis_est={chassis_est:7.3f} rad/s")
                last_print = now

            if args.duration and t >= args.duration:
                break
            if not plt.fignum_exists(fig.number):
                break

            sleep_time = 1.0 / args.rate - (time.time() - now)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("用户停止")
    finally:
        if csv_handle is not None:
            try:
                csv_handle.flush()
                csv_handle.close()
            except Exception:
                pass
            if csv_path is not None:
                print(f"CSV 已保存: {csv_path}")
        try:
            plt.ioff()
        except Exception:
            pass
        if client is not None:
            try:
                client.close()
            except Exception:
                pass
        if owned_process is not None:
            owned_process.stop()

        if (not args.no_history_png and csv_path is not None
                and csv_rows > 0):
            png_path = csv_path.with_suffix(".png")
            cmd = [
                sys.executable,
                str(TOOLS_DIR / "plot_yaw_csv.py"),
                str(csv_path),
                "--no-show",
                "--png", str(png_path),
                "--figsize", args.history_figsize,
                "--dpi", str(args.history_dpi),
            ]
            print(f"生成历史 PNG: {png_path}")
            try:
                result = subprocess.run(cmd)
                if result.returncode != 0:
                    print(f"历史 PNG 生成失败，退出码 {result.returncode}")
            except Exception as exc:
                print(f"历史 PNG 生成失败: {exc}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
