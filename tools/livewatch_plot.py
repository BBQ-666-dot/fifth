# -*- coding: utf-8 -*-
'''
Live Watch 波形显示工具
=======================

不中断程序运行, 通过 OpenOCD 的 TCL RPC 后台读取内存变量, 实时绘制波形.
适用于 OpenOCD + 无线 DAPLink / CMSIS-DAP / J-Link / STLink 等调试器.

用法示例:
    python livewatch_plot.py                                  # 默认: 连接 VS Code 调试会话(找不到则自启), 并自动同步 Live Watch 变量
    python livewatch_plot.py --wait-attach                    # 等待 VS Code 启动 OpenOCD 后自动连接(随调试自动打开用)
    python livewatch_plot.py --standalone                     # 强制自己启动 openocd_wireless_dap.cfg
    python livewatch_plot.py --var gimbal_direct.yaw.fdb.pos  # 监视指定变量(可多次)
    python livewatch_plot.py --no-follow-vscode               # 不自动同步 VS Code Live Watch 变量
    python livewatch_plot.py --tcl 50001                      # 连接指定 TCL 端口
    python livewatch_plot.py --check                          # 仅解析符号, 打印地址/类型后退出
    python livewatch_plot.py --demo                           # 无硬件演示界面

在 VS Code 的 CORTEX LIVE WATCH 面板添加/删除变量后, 本工具会自动同步并绘制对应波形.

依赖: matplotlib / numpy / tkinter (Anaconda 自带)
'''

import argparse
import glob
import json
import math
import os
import queue
import re
import shutil
import socket
import sqlite3
import struct
import subprocess
import sys
import threading
import time
import tkinter as tk
import tkinter.filedialog
import tkinter.messagebox
import tkinter.ttk as ttk
from collections import deque
from pathlib import Path
from urllib.parse import unquote

import numpy as np

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei UI", "Microsoft YaHei",
                                          "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent

DEFAULT_VARS = [
    "gimbal_direct.yaw.fdb.pos",
    "gimbal_direct.pitch.fdb.pos",
    "gimbal_direct.reference.yaw",
    "gimbal_direct.reference.pitch",
]

DEFAULT_TCL_PORT = 6666
DEFAULT_GDB_PORT = 3333
VSCODE_TCL_PORT_MIN = 50000
VSCODE_TCL_PORT_MAX = 52000

BG = "#101319"
CARD = "#181d25"
CARD_HI = "#222a35"
BORDER = "#2b3542"
FG = "#e9edf3"
FG_DIM = "#8b95a7"
ACCENT = "#3da9fc"
ACCENT_DK = "#2f86cf"
OK = "#2dd4a7"
WARN = "#f4c542"
ERR = "#f2555a"

FONT = "Microsoft YaHei UI"
FONT_TXT = (FONT, 10)
FONT_SM = (FONT, 9)
FONT_MONO = ("Consolas", 10)

LINE_COLORS = ["#3da9fc", "#2dd4a7", "#f4c542", "#f2555a",
               "#b197fc", "#ff922b", "#63e6be", "#ff8787"]

WINDOW_CHOICES = [("5 s", 5.0), ("10 s", 10.0), ("30 s", 30.0),
                  ("60 s", 60.0), ("全部", None)]
RATE_CHOICES = [2.0, 5.0, 10.0, 20.0, 50.0]

_NO_WINDOW = 0
if os.name == "nt":
    _NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)

_FMT_BY_TYPE = {
    "float": "<f", "double": "<d", "fp32": "<f", "fp64": "<d",
    "uint8_t": "<B", "int8_t": "<b", "char": "<b", "signed char": "<b",
    "unsigned char": "<B", "_Bool": "<B", "bool": "<B",
    "uint16_t": "<H", "int16_t": "<h", "short": "<h", "short int": "<h",
    "unsigned short": "<H", "unsigned short int": "<H",
    "uint32_t": "<I", "int32_t": "<i", "int": "<i", "unsigned int": "<I",
    "long": "<i", "long int": "<i", "unsigned long": "<I", "unsigned long int": "<I",
    "uint64_t": "<Q", "int64_t": "<q", "long long": "<q",
    "unsigned long long": "<Q", "long long int": "<q",
}
_FMT_BY_SIZE = {1: "<B", 2: "<H", 4: "<I", 8: "<Q"}


def apply_theme(root):
    root.configure(bg=BG)
    root.option_add("*TCombobox*Listbox.background", CARD_HI)
    root.option_add("*TCombobox*Listbox.foreground", FG)
    root.option_add("*TCombobox*Listbox.selectBackground", ACCENT)
    root.option_add("*TCombobox*Listbox.selectForeground", "#ffffff")
    root.option_add("*TCombobox*Listbox.font", FONT_TXT)
    style = ttk.Style(root)
    try:
        style.theme_use("clam")
    except Exception:
        pass
    style.configure("Dark.TCombobox",
                    fieldbackground=CARD_HI, background=CARD_HI, foreground=FG,
                    arrowcolor=FG_DIM, bordercolor=BORDER, lightcolor=BORDER,
                    darkcolor=BORDER, insertcolor=FG, padding=4,
                    selectbackground=CARD_HI, selectforeground=FG, font=FONT_TXT)
    style.map("Dark.TCombobox",
              fieldbackground=[("readonly", CARD_HI)],
              foreground=[("readonly", FG)],
              background=[("readonly", CARD_HI)])


def themed_button(parent, text, command=None, accent=False):
    bg = ACCENT if accent else CARD_HI
    button = tk.Button(parent, text=text, command=command, bg=bg,
                       fg="#ffffff" if accent else FG,
                       activebackground=ACCENT_DK if accent else BORDER,
                       activeforeground="#ffffff", relief="flat", bd=0,
                       font=FONT_TXT, cursor="hand2", padx=12, pady=4,
                       highlightthickness=0)

    def on_enter(_):
        button.configure(bg=ACCENT_DK if accent else BORDER)

    def on_leave(_):
        button.configure(bg=ACCENT if accent else CARD_HI)

    button.bind("<Enter>", on_enter)
    button.bind("<Leave>", on_leave)
    return button


def find_gdb(explicit=None):
    candidates = [explicit, os.environ.get("ARM_GDB"),
                  r"C:\ST\STM32CubeCLT_1.22.0\GNU-tools-for-STM32\bin\arm-none-eabi-gdb.exe"]
    for cand in candidates:
        if cand and os.path.isfile(cand):
            return cand
    found = shutil.which("arm-none-eabi-gdb")
    if found:
        return found
    raise RuntimeError("未找到 arm-none-eabi-gdb, 请用 --gdb 指定路径")


def find_openocd(explicit=None):
    candidates = [explicit, r"C:\msys64\mingw64\bin\openocd.exe"]
    for cand in candidates:
        if cand and os.path.isfile(cand):
            return cand
    found = shutil.which("openocd")
    if found:
        return found
    raise RuntimeError("未找到 openocd, 请用 --openocd 指定路径")


class VarSpec:
    __slots__ = ("expr", "addr", "size", "type_name", "fmt", "sizeof")

    def __init__(self, expr, addr, size, type_name, fmt, sizeof=None):
        self.expr = expr
        self.addr = addr
        self.size = size
        self.type_name = type_name
        self.fmt = fmt
        self.sizeof = sizeof if sizeof is not None else size


def describe_spec(spec):
    return f"{spec.expr}  @0x{spec.addr:08X}  {spec.sizeof}B  {spec.type_name}"


def _run_gdb(gdb, elf, commands, timeout=60):
    elf_cmd = str(elf).replace("\\", "/")
    args = [gdb, "-batch", "-nx", "-ex", f'file "{elf_cmd}"']
    for command in commands:
        args += ["-ex", command]
    proc = subprocess.run(args, capture_output=True, text=True,
                          encoding="utf-8", errors="replace",
                          timeout=timeout, creationflags=_NO_WINDOW)
    return (proc.stdout or "") + "\n" + (proc.stderr or "")


def _clean_type(type_name):
    words = [word for word in type_name.replace("*", " * ").split()
             if word not in ("const", "volatile", "static", "struct", "union", "enum")]
    return " ".join(words).strip()


def _fmt_for(type_name, size):
    fmt = _FMT_BY_TYPE.get(_clean_type(type_name))
    if fmt is not None and struct.calcsize(fmt) == size:
        return fmt
    if fmt is not None and size in _FMT_BY_SIZE:
        return _FMT_BY_SIZE[size]
    return _FMT_BY_SIZE.get(size)


def resolve_raw(gdb, elf, expr):
    text = _run_gdb(gdb, elf, [f"p &({expr})", f"p sizeof({expr})", f"whatis {expr}"])
    addr = None
    sizeof = None
    type_name = ""
    for line in text.splitlines():
        line = line.strip()
        if addr is None:
            match = re.match(r"^\$\d+ = .*?(0x[0-9a-fA-F]+)", line)
            if match:
                addr = int(match.group(1), 16)
                continue
        if addr is not None and sizeof is None:
            match = re.match(r"^\$\d+ = (\d+)$", line)
            if match:
                sizeof = int(match.group(1))
                continue
        if addr is not None and sizeof is not None and not type_name:
            if line.startswith("type = "):
                type_name = line[len("type = "):].strip()
                break
    if addr is None:
        raise ValueError(f"GDB 找不到符号: {expr}\n{text.strip()[-400:]}")
    if sizeof is None:
        sizeof = 4
    return addr, sizeof, type_name


def resolve_expression(gdb, elf, expr):
    addr, sizeof, raw_type = resolve_raw(gdb, elf, expr)
    fmt = _fmt_for(raw_type, sizeof)
    known_scalar = _clean_type(raw_type) in _FMT_BY_TYPE
    composite = ("[" in raw_type or "struct " in raw_type or "union " in raw_type
                 or (not known_scalar and sizeof not in _FMT_BY_SIZE))
    if composite or fmt is None:
        raise ValueError(f"{expr} 是复合类型 ({raw_type}, {sizeof}B), 将展开成员")
    return VarSpec(expr, addr, struct.calcsize(fmt), _clean_type(raw_type), fmt, sizeof)


_FIELD_RE = re.compile(r"^/\*\s*(\d+)\s*\|\s*(\d+)\s*\*/\s*(.+?);\s*$")
_DECL_ARRAY_RE = re.compile(r"^(.+?)\s+([A-Za-z_]\w*)\s*\[(\d+)\]$")
_DECL_NAME_RE = re.compile(r"^(.+?)\s+([A-Za-z_]\w*)$")
_TYPE_ARRAY_RE = re.compile(r"type\s*=\s*(.+?)\s*\[(\d+)\]\s*$", re.M)


def _ptype_fields(gdb, elf, expr):
    text = _run_gdb(gdb, elf, [f"ptype /o {expr}"])
    fields = []
    for line in text.splitlines():
        match = _FIELD_RE.match(line.strip())
        if match:
            fields.append((int(match.group(1)), int(match.group(2)),
                           match.group(3).strip()))
    return fields, _TYPE_ARRAY_RE.search(text)


def _walk_fields(gdb, elf, expr, addr, fields, specs, depth, cap):
    if depth > 2 or len(specs) >= cap:
        return
    for offset, field_size, decl in fields:
        if len(specs) >= cap:
            return
        if "(" in decl or ":" in decl or "*" in decl or "{" in decl:
            continue
        array_match = _DECL_ARRAY_RE.match(decl)
        name_match = _DECL_NAME_RE.match(decl)
        if array_match:
            type_text = array_match.group(1)
            name = array_match.group(2)
            count = int(array_match.group(3))
            if count <= 0:
                continue
            element_size = field_size // count
            if not element_size:
                continue
            fmt = _fmt_for(type_text, element_size)
            if fmt is None:
                continue
            for index in range(count):
                if len(specs) >= cap:
                    return
                specs.append(VarSpec(
                    f"{expr}.{name}[{index}]", addr + offset + index * element_size,
                    struct.calcsize(fmt), _clean_type(type_text), fmt, element_size))
        elif name_match:
            type_text = name_match.group(1)
            name = name_match.group(2)
            child_expr = f"{expr}.{name}"
            child_addr = addr + offset
            fmt = _fmt_for(type_text, field_size)
            if _clean_type(type_text) in _FMT_BY_TYPE and fmt is not None:
                specs.append(VarSpec(child_expr, child_addr, struct.calcsize(fmt),
                                     _clean_type(type_text), fmt, field_size))
                continue
            child_fields, child_array = _ptype_fields(gdb, elf, child_expr)
            if child_fields:
                _walk_fields(gdb, elf, child_expr, child_addr, child_fields,
                             specs, depth + 1, cap)
            elif fmt is not None and field_size in _FMT_BY_SIZE:
                specs.append(VarSpec(child_expr, child_addr, struct.calcsize(fmt),
                                     _clean_type(type_text), fmt, field_size))


def expand_expression(gdb, elf, expr, cap=12):
    try:
        return [resolve_expression(gdb, elf, expr)]
    except ValueError:
        pass
    addr, sizeof, raw_type = resolve_raw(gdb, elf, expr)
    specs = []
    array_match = re.search(r"\[(\d+)\]", raw_type)
    if array_match:
        count = int(array_match.group(1))
        base_type = raw_type.split("[")[0].strip()
        element_size = sizeof // count if count else 0
        fmt = _fmt_for(base_type, element_size)
        if fmt is None or not element_size:
            raise ValueError(f"无法解析数组 {expr} ({raw_type})")
        for index in range(min(count, cap)):
            specs.append(VarSpec(f"{expr}[{index}]", addr + index * element_size,
                                 struct.calcsize(fmt), _clean_type(base_type),
                                 fmt, element_size))
        return specs
    fields, _ = _ptype_fields(gdb, elf, expr)
    _walk_fields(gdb, elf, expr, addr, fields, specs, 0, cap)
    if not specs:
        raise ValueError(f"{expr} ({raw_type}) 内没有可监视的标量成员")
    return specs


def expand_expressions(gdb, elf, expressions, cap_each=12):
    items = []
    errors = []
    seen = set()
    for expr in expressions:
        try:
            specs = expand_expression(gdb, elf, expr, cap_each)
        except Exception as exc:
            errors.append((expr, str(exc).splitlines()[0]))
            continue
        for spec in specs:
            if spec.expr in seen:
                continue
            seen.add(spec.expr)
            items.append((expr, spec))
    return items, errors


class OpenOcdTcl:
    TERM = b"\x1a"

    def __init__(self, host="127.0.0.1", port=DEFAULT_TCL_PORT, timeout=5.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.buf = b""
        self.lock = threading.Lock()
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)

    def exec(self, command):
        with self.lock:
            self.sock.sendall(command.encode("utf-8", errors="replace") + self.TERM)
            while self.TERM not in self.buf:
                chunk = self.sock.recv(4096)
                if not chunk:
                    raise ConnectionError("OpenOCD 连接已断开")
                self.buf += chunk
            data, self.buf = self.buf.split(self.TERM, 1)
        return data.decode("utf-8", errors="replace").strip()

    def read_bytes(self, addr, size):
        response = self.exec(f"mdb 0x{addr:08x} {size}").strip()
        match = re.match(r"^0x[0-9a-fA-F]+:\s*((?:[0-9a-fA-F]{2}\s*)+)$", response)
        if match:
            tokens = match.group(1).split()
        else:
            tokens = [tok[2:] if tok.lower().startswith("0x") else tok
                      for tok in response.split()]
            tokens = [tok for tok in tokens
                      if re.fullmatch(r"[0-9a-fA-F]{2}", tok)]
        if len(tokens) < size:
            raise IOError(f"读取 0x{addr:08X} 失败: {response[:120]!r}")
        return bytes(int(tok, 16) for tok in tokens[:size])

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


def parse_host_port(value, default_port):
    if ":" in value:
        host, port = value.rsplit(":", 1)
        return host or "127.0.0.1", int(port)
    return "127.0.0.1", int(value) if value else default_port


def find_openocd_tcl_port(port_min=VSCODE_TCL_PORT_MIN, port_max=VSCODE_TCL_PORT_MAX):
    import concurrent.futures

    candidates = list(range(port_min, port_max + 1))
    if not (port_min <= DEFAULT_TCL_PORT <= port_max):
        candidates.append(DEFAULT_TCL_PORT)

    def probe(port):
        try:
            sock = socket.create_connection(("127.0.0.1", port), timeout=0.1)
        except OSError:
            return None
        try:
            sock.settimeout(0.25)
            sock.sendall(b"\x1a")
            if b"\x1a" not in sock.recv(64):
                return None
            sock.sendall(b"version\x1a")
            if b"Open On-Chip Debugger" in sock.recv(512):
                return port
        except OSError:
            pass
        finally:
            sock.close()
        return None

    with concurrent.futures.ThreadPoolExecutor(max_workers=96) as pool:
        futures = [pool.submit(probe, port) for port in candidates]
        try:
            for future in concurrent.futures.as_completed(futures, timeout=8.0):
                result = future.result()
                if result is not None:
                    for item in futures:
                        item.cancel()
                    return result
        except concurrent.futures.TimeoutError:
            pass
    raise RuntimeError("未找到 OpenOCD TCL 端口, 请确认 VS Code 调试会话正在运行, "
                       "或用 --tcl 指定端口")


def find_workspace_state_dbs(project_dir):
    project = str(project_dir).replace("/", "\\").rstrip("\\").lower()
    bases = [
        os.path.join(os.environ.get("APPDATA", ""), "Code", "User", "workspaceStorage"),
        os.path.join(os.environ.get("APPDATA", ""), "Code - Insiders", "User",
                     "workspaceStorage"),
    ]
    dbs = []
    for base in bases:
        for ws_json in glob.glob(os.path.join(base, "*", "workspace.json")):
            try:
                with open(ws_json, "r", encoding="utf-8") as handle:
                    folder = json.load(handle).get("folder", "")
            except Exception:
                continue
            if not folder.startswith("file:///"):
                continue
            path = unquote(folder[len("file:///"):]).replace("/", "\\")
            if path.rstrip("\\").lower() == project:
                db = Path(ws_json).parent / "state.vscdb"
                if db.exists():
                    dbs.append(db)
    dbs.sort(key=lambda item: item.stat().st_mtime, reverse=True)
    return dbs


def find_workspace_state_db(project_dir):
    dbs = find_workspace_state_dbs(project_dir)
    return dbs[0] if dbs else None


def read_followed_expressions(dbs, fresh_seconds=900):
    fresh_cutoff = time.time() - fresh_seconds
    for db in dbs:
        try:
            if db.stat().st_mtime < fresh_cutoff:
                continue
        except OSError:
            continue
        expressions = read_livewatch_expressions(db)
        if expressions is not None:
            return expressions
    for db in dbs:
        expressions = read_livewatch_expressions(db)
        if expressions is not None:
            return expressions
    return None


def read_livewatch_expressions(db_path):
    if db_path is None or not Path(db_path).exists():
        return None
    try:
        conn = sqlite3.connect(f"file:{Path(db_path).as_posix()}?mode=ro",
                               uri=True, timeout=1.0)
    except sqlite3.Error:
        return None
    try:
        row = conn.execute("SELECT value FROM ItemTable WHERE key=?",
                           ("marus25.cortex-debug",)).fetchone()
    except sqlite3.Error:
        row = None
    finally:
        conn.close()
    if not row:
        return None
    value = row[0]
    if isinstance(value, bytes):
        value = value.decode("utf-8", "replace")
    try:
        memento = json.loads(value)
    except Exception:
        return None
    tree = memento.get("livewatch.watchTree")
    if not isinstance(tree, dict):
        return None
    expressions = []
    for child in tree.get("children", []) or []:
        expr = child.get("expr") or child.get("name")
        if expr:
            expressions.append(expr)
    return expressions


class VscodeLiveWatchSync(threading.Thread):
    def __init__(self, project_dir, window, interval=1.0):
        super().__init__(daemon=True)
        self.project_dir = project_dir
        self.window = window
        self.interval = interval
        self.stop_event = threading.Event()

    def run(self):
        last = None
        while not self.stop_event.is_set():
            dbs = find_workspace_state_dbs(self.project_dir)
            expressions = read_followed_expressions(dbs) if dbs else None
            if expressions is not None and expressions != last:
                last = list(expressions)
                try:
                    self.window.sync_from_vscode(last)
                except Exception:
                    pass
            self.stop_event.wait(self.interval)

    def stop(self):
        self.stop_event.set()


def _assign_kill_on_close_job(proc):
    if os.name != "nt":
        return None
    try:
        import ctypes
        from ctypes import wintypes
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)

        class BasicLimitInformation(ctypes.Structure):
            _fields_ = [("PerProcessUserTimeLimit", ctypes.c_int64),
                        ("PerJobUserTimeLimit", ctypes.c_int64),
                        ("LimitFlags", wintypes.DWORD),
                        ("MinimumWorkingSetSize", ctypes.c_size_t),
                        ("MaximumWorkingSetSize", ctypes.c_size_t),
                        ("ActiveProcessLimit", wintypes.DWORD),
                        ("Affinity", ctypes.c_void_p),
                        ("PriorityClass", wintypes.DWORD),
                        ("SchedulingClass", wintypes.DWORD)]

        class IoCounters(ctypes.Structure):
            _fields_ = [("ReadOperationCount", ctypes.c_uint64),
                        ("WriteOperationCount", ctypes.c_uint64),
                        ("OtherOperationCount", ctypes.c_uint64),
                        ("ReadTransferCount", ctypes.c_uint64),
                        ("WriteTransferCount", ctypes.c_uint64),
                        ("OtherTransferCount", ctypes.c_uint64)]

        class ExtendedLimitInformation(ctypes.Structure):
            _fields_ = [("BasicLimitInformation", BasicLimitInformation),
                        ("IoInfo", IoCounters),
                        ("ProcessMemoryLimit", ctypes.c_size_t),
                        ("JobMemoryLimit", ctypes.c_size_t),
                        ("PeakProcessMemoryUsed", ctypes.c_size_t),
                        ("PeakJobMemoryUsed", ctypes.c_size_t)]

        job = kernel32.CreateJobObjectW(None, None)
        if not job:
            return None
        info = ExtendedLimitInformation()
        info.BasicLimitInformation.LimitFlags = 0x2000
        kernel32.SetInformationJobObject(job, 9, ctypes.byref(info),
                                         ctypes.sizeof(info))
        handle = wintypes.HANDLE(int(proc._handle))
        if not kernel32.AssignProcessToJobObject(job, handle):
            kernel32.CloseHandle(job)
            return None
        return job
    except Exception:
        return None


class OpenOcdProcess:
    def __init__(self, exe, cfg, tcl_port=DEFAULT_TCL_PORT, gdb_port=DEFAULT_GDB_PORT):
        self.exe = exe
        self.cfg = cfg
        self.tcl_port = tcl_port
        self.gdb_port = gdb_port
        self.proc = None
        self._job = None
        self._output = deque(maxlen=400)
        self._reader = None

    def _read_output(self):
        try:
            for line in self.proc.stdout:
                self._output.append(line.decode("utf-8", "replace").rstrip())
        except Exception:
            pass

    def start(self):
        command = [self.exe, "-f", self.cfg,
                   "-c", f"tcl_port {self.tcl_port}",
                   "-c", f"gdb_port {self.gdb_port}",
                   "-c", "init"]
        self.proc = subprocess.Popen(command, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT,
                                     cwd=str(PROJECT_DIR),
                                     creationflags=_NO_WINDOW)
        self._job = _assign_kill_on_close_job(self.proc)
        self._reader = threading.Thread(target=self._read_output, daemon=True)
        self._reader.start()
        self.wait_ready()

    def wait_ready(self, timeout=20.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError("OpenOCD 启动失败(探头可能被另一个 OpenOCD / "
                                   "调试会话占用, 请先关闭它再试):\n" + self.log_tail())
            try:
                probe = socket.create_connection(("127.0.0.1", self.tcl_port), timeout=0.2)
                probe.close()
                return
            except OSError:
                time.sleep(0.2)
        raise RuntimeError("OpenOCD TCL 端口连接超时:\n" + self.log_tail())

    def log_tail(self, lines=15):
        if not self._output:
            return "(无日志)"
        return "\n".join(list(self._output)[-lines:])

    def stop(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        if self.proc is not None and self.proc.stdout is not None:
            try:
                self.proc.stdout.close()
            except Exception:
                pass


class DemoClient:
    def __init__(self):
        self.t0 = time.time()

    def read_bytes(self, addr, size):
        data = bytearray()
        offset = 0
        while offset < size:
            index = max(0, (addr + offset - 0x20000000) // 4)
            t = time.time() - self.t0
            freq = 0.25 + 0.17 * index
            value = math.sin(2.0 * math.pi * freq * t + index) * (1.0 + 0.5 * index)
            data += struct.pack("<f", value)
            offset += 4
        return bytes(data[:size])

    def close(self):
        pass


class LiveVar:
    def __init__(self, spec, color):
        self.expr = spec.expr
        self.addr = spec.addr
        self.size = spec.size
        self.fmt = spec.fmt
        self.type_name = spec.type_name
        self.color = color
        self.enabled = True
        self.source = "manual"
        self.source_expr = spec.expr
        self.data = deque()
        self.line = None
        self.row = None
        self.check = None
        self.value_label = None
        self.errors = 0
        self.cap = 20000

    def append(self, t, value):
        self.data.append((t, value))
        while len(self.data) > self.cap:
            self.data.popleft()

    def clear(self):
        self.data.clear()


class Sampler(threading.Thread):
    def __init__(self, client, variables, rate):
        super().__init__(daemon=True)
        self.client = client
        self.variables = variables
        self.rate = rate
        self.running = True
        self.stop_event = threading.Event()
        self.start_time = time.time()
        self.sample_count = 0
        self.error_count = 0
        self.last_error = ""

    def run(self):
        while not self.stop_event.is_set():
            if not self.running:
                time.sleep(0.05)
                continue
            cycle_start = time.time()
            t = cycle_start - self.start_time
            enabled = [v for v in list(self.variables) if v.enabled]
            groups = []
            for var in sorted(enabled, key=lambda item: item.addr):
                if groups and var.addr - groups[-1][1] <= 8:
                    groups[-1][1] = max(groups[-1][1], var.addr + var.size)
                    groups[-1][2].append(var)
                else:
                    groups.append([var.addr, var.addr + var.size, [var]])
            for start, end, vars_in_group in groups:
                if self.stop_event.is_set():
                    break
                try:
                    raw = self.client.read_bytes(start, end - start)
                except Exception as exc:
                    for var in vars_in_group:
                        var.errors += 1
                    self.error_count += 1
                    self.last_error = str(exc)[:120]
                    continue
                for var in vars_in_group:
                    offset = var.addr - start
                    try:
                        var.append(t, struct.unpack(
                            var.fmt, raw[offset:offset + var.size])[0])
                        self.sample_count += 1
                    except Exception:
                        var.errors += 1
                        self.error_count += 1
            sleep_time = 1.0 / max(self.rate, 0.1) - (time.time() - cycle_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop(self):
        self.stop_event.set()


class LiveWatchWindow:
    def __init__(self, client, specs, args, owned_process=None):
        self.client = client
        self.owned_process = owned_process
        self.elf = args.elf
        self.gdb = args.gdb_path
        self.window_sec = 10.0
        self.paused = False
        self.legend = None
        self._last_status_time = time.time()
        self._last_sample_count = 0
        self._last_draw_error = None

        self.root = tk.Tk()
        self.root.title("Live Watch 波形显示")
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        width = min(1280, screen_w - 60)
        height = min(760, screen_h - 80)
        self.root.geometry(f"{width}x{height}+{max(0, (screen_w - width) // 2)}"
                           f"+{max(0, (screen_h - height) // 2)}")
        self.root.minsize(900, 560)
        self.demo = args.demo
        apply_theme(self.root)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.variables = []
        self.follow_thread = None
        self.ui_queue = queue.Queue()
        self.cli_vars = bool(getattr(args, "var", []))
        self.sampler = Sampler(client, self.variables, args.rate)

        self._build_toolbar()
        self._build_var_panel()
        self._build_plot()

        for color_index, spec in enumerate(specs):
            var = self._add_variable(spec, color_index)
            var.source = "cli" if self.cli_vars else "default"
        self._rebuild_legend()

        self.sampler.start()
        if self.demo:
            self._set_state("演示模式 (无硬件)", WARN)
        elif self.owned_process is not None:
            self._set_state("独立模式 · 已启动 OpenOCD (独占调试器)", OK)
        else:
            self._set_state("已连接调试会话", OK)

        if getattr(args, "follow_vscode", False) and not self.demo:
            self.follow_thread = VscodeLiveWatchSync(PROJECT_DIR, self)
            self.follow_thread.start()
        self.root.after(50, self._update_plot)
        self.root.after(150, self._poll_ui_queue)
        self.root.after(300, self._update_status)

    # ---------------------------------------------------------------- UI
    def _build_toolbar(self):
        bar = tk.Frame(self.root, bg=BG, height=52)
        bar.place(x=0, y=0, relwidth=1, height=52)
        bar.pack_propagate(False)

        self.dot = tk.Canvas(bar, width=16, height=16, bg=BG, highlightthickness=0)
        self.dot.pack(side="left", padx=(12, 4))
        self.dot_item = self.dot.create_oval(3, 3, 14, 14, fill=WARN, outline="")

        self.state_label = tk.Label(bar, text="连接中...", bg=BG, fg=FG_DIM,
                                    font=FONT_TXT)
        self.state_label.pack(side="left")

        self.btn_run = themed_button(bar, "停止采样", self._toggle_run, accent=True)
        self.btn_run.pack(side="left", padx=(18, 6), pady=9)

        self.btn_pause = themed_button(bar, "暂停显示", self._toggle_pause)
        self.btn_pause.pack(side="left", padx=6, pady=9)

        self.btn_clear = themed_button(bar, "清空", self._clear_data)
        self.btn_clear.pack(side="left", padx=6, pady=9)

        tk.Label(bar, text="时间窗", bg=BG, fg=FG_DIM, font=FONT_SM
                 ).pack(side="left", padx=(12, 2))
        self.combo_window = ttk.Combobox(bar, values=[c[0] for c in WINDOW_CHOICES],
                                         state="readonly", width=6, font=FONT_TXT,
                                         style="Dark.TCombobox")
        self.combo_window.current(1)
        self.combo_window.pack(side="left")
        self.combo_window.bind("<<ComboboxSelected>>", self._on_window_changed)

        tk.Label(bar, text="采样率", bg=BG, fg=FG_DIM, font=FONT_SM
                 ).pack(side="left", padx=(12, 2))
        self.combo_rate = ttk.Combobox(bar, values=[f"{r:g} Hz" for r in RATE_CHOICES],
                                       state="readonly", width=6, font=FONT_TXT,
                                       style="Dark.TCombobox")
        default_rate = min(range(len(RATE_CHOICES)),
                           key=lambda i: abs(RATE_CHOICES[i] - self.sampler.rate))
        self.combo_rate.current(default_rate)
        self.combo_rate.pack(side="left")
        self.combo_rate.bind("<<ComboboxSelected>>", self._on_rate_changed)

        self.rate_label = tk.Label(bar, text="采样 0.0 Hz  错误 0", bg=BG, fg=FG_DIM,
                                   font=FONT_MONO)
        self.rate_label.pack(side="right", padx=(6, 12))

        self.btn_png = themed_button(bar, "导出图片", self._export_png)
        self.btn_png.pack(side="right", padx=6, pady=9)

        self.btn_csv = themed_button(bar, "导出 CSV", self._export_csv)
        self.btn_csv.pack(side="right", padx=6, pady=9)

    def _build_var_panel(self):
        panel = tk.Frame(self.root, bg=CARD)
        panel.place(x=0, y=52, width=320, relheight=1, height=-52)

        tk.Label(panel, text="监视变量", bg=CARD, fg=FG, font=(FONT, 10, "bold")
                 ).place(x=12, y=8)

        self.var_canvas = tk.Canvas(panel, bg=CARD, highlightthickness=0)
        self.var_canvas.place(x=4, y=36, width=296, relheight=1, height=-100)
        scrollbar = tk.Scrollbar(panel, command=self.var_canvas.yview, bg=CARD_HI,
                                 troughcolor=CARD, activebackground=BORDER,
                                 highlightthickness=0, bd=0, relief="flat")
        scrollbar.place(x=300, y=36, width=14, relheight=1, height=-100)
        self.var_canvas.configure(yscrollcommand=scrollbar.set)

        self.var_list = tk.Frame(self.var_canvas, bg=CARD)
        self._var_window = self.var_canvas.create_window((0, 0), window=self.var_list,
                                                         anchor="nw")
        self.var_list.bind("<Configure>", lambda e: self.var_canvas.configure(
            scrollregion=self.var_canvas.bbox("all")))
        self.var_canvas.bind("<Configure>", lambda e: self.var_canvas.itemconfigure(
            self._var_window, width=e.width))

        panel.bind("<Enter>", lambda e: self.var_canvas.bind_all(
            "<MouseWheel>", self._on_mousewheel))
        panel.bind("<Leave>", lambda e: self.var_canvas.unbind_all("<MouseWheel>"))

        tk.Label(panel, text="添加变量 (GDB 表达式)", bg=CARD, fg=FG_DIM, font=FONT_SM
                 ).place(x=12, rely=1.0, y=-66, anchor="sw")
        self.entry_expr = tk.Entry(panel, bg=CARD_HI, fg=FG, insertbackground=FG,
                                   relief="flat", font=FONT_MONO)
        self.entry_expr.place(x=12, rely=1.0, y=-8, width=230, height=30, anchor="sw")
        self.entry_expr.bind("<Return>", self._add_from_entry)
        themed_button(panel, "添加", self._add_from_entry, accent=True
                      ).place(x=248, rely=1.0, y=-8, width=66, height=30, anchor="sw")

    def _build_plot(self):
        frame = tk.Frame(self.root, bg=BG)
        frame.place(x=320, y=52, relwidth=1, relheight=1, width=-320, height=-52)
        self.fig = plt.Figure(figsize=(9, 6), dpi=100, facecolor=BG)
        self.ax = self.fig.add_subplot(111)
        self._style_axes()
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _style_axes(self):
        self.ax.set_facecolor(CARD)
        for spine in self.ax.spines.values():
            spine.set_color(BORDER)
        self.ax.tick_params(colors=FG_DIM, labelsize=8)
        self.ax.grid(True, color=BORDER, alpha=0.45, linewidth=0.6)
        self.ax.set_axisbelow(True)
        self.ax.set_title("Live Watch 实时波形", fontsize=10, color=FG, pad=6)
        self.ax.set_xlabel("t (s)", fontsize=9, color=FG_DIM)
        self.ax.set_xlim(0, 10)

    # ------------------------------------------------------------ variables
    def _add_variable(self, spec, color_index=None):
        if color_index is None:
            color_index = len(self.variables)
        var = LiveVar(spec, LINE_COLORS[color_index % len(LINE_COLORS)])
        var.line, = self.ax.plot([], [], color=var.color, linewidth=1.3,
                                 label=var.expr)
        self._create_var_row(var)
        self.variables.append(var)
        self._rebuild_legend()
        return var

    def _create_var_row(self, var):
        row = tk.Frame(self.var_list, bg=CARD)
        row.pack(fill="x", pady=1)
        var.check = tk.BooleanVar(value=True)
        check = tk.Checkbutton(row, text=var.expr, variable=var.check,
                               command=lambda v=var: self._toggle_variable(v),
                               bg=CARD, fg=var.color, selectcolor=CARD_HI,
                               activebackground=CARD, activeforeground=var.color,
                               font=FONT_SM, anchor="w", bd=0, highlightthickness=0)
        check.pack(side="left", fill="x", expand=True)
        var.value_label = tk.Label(row, text="--", bg=CARD, fg=FG_DIM,
                                   font=FONT_MONO, width=10, anchor="e")
        var.value_label.pack(side="right")
        remove = tk.Button(row, text="×", command=lambda v=var: self._remove_variable(v),
                           bg=CARD, fg=FG_DIM, activebackground=CARD_HI,
                           activeforeground=ERR, relief="flat", bd=0,
                           font=(FONT, 10), cursor="hand2", highlightthickness=0)
        remove.pack(side="right")
        var.row = row

    def _toggle_variable(self, var):
        var.enabled = bool(var.check.get())
        self._rebuild_legend()

    def _remove_variable(self, var):
        if var in self.variables:
            self.variables.remove(var)
        if var.line is not None:
            var.line.remove()
            var.line = None
        if var.row is not None:
            var.row.destroy()
            var.row = None
        self._rebuild_legend()
        self._refresh_caps()

    def _add_from_entry(self, event=None):
        expr = self.entry_expr.get().strip()
        if not expr:
            return
        if any(v.expr == expr for v in self.variables):
            self._set_state(f"变量已存在: {expr}", WARN)
            return
        self._set_state(f"解析 {expr} ...", WARN)

        def worker():
            try:
                specs = expand_expression(self.gdb, self.elf, expr)
                self._post_ui(lambda: self._on_expr_resolved(specs))
            except Exception as exc:
                message = str(exc)
                self._post_ui(lambda: messagebox.showerror("解析失败", message))

        threading.Thread(target=worker, daemon=True).start()

    def _on_expr_resolved(self, specs):
        added = 0
        for spec in specs:
            if not any(v.expr == spec.expr for v in self.variables):
                self._add_variable(spec)
                added += 1
        self.entry_expr.delete(0, tk.END)
        self._set_state(f"已添加 {added} 个变量", OK)

    def _post_ui(self, func):
        self.ui_queue.put(func)

    def _poll_ui_queue(self):
        try:
            while True:
                func = self.ui_queue.get_nowait()
                try:
                    func()
                except Exception:
                    pass
        except queue.Empty:
            pass
        self.root.after(150, self._poll_ui_queue)

    def sync_from_vscode(self, expressions):
        items, errors = expand_expressions(self.gdb, self.elf, expressions)
        self._post_ui(lambda: self._apply_vscode_sync(expressions, items))

    def _apply_vscode_sync(self, expressions, new_items):
        wanted = set(expressions)
        if wanted:
            for var in [v for v in self.variables if v.source == "default"]:
                self._remove_variable(var)
        for origin, spec in new_items:
            if not any(v.expr == spec.expr for v in self.variables):
                var = self._add_variable(spec)
                var.source = "vscode"
                var.source_expr = origin
        for var in [v for v in self.variables
                    if v.source == "vscode" and v.source_expr not in wanted]:
            self._remove_variable(var)
        self._set_state(self._mode_text(), OK)

    # ------------------------------------------------------------- plotting
    def _update_plot(self):
        if not self.paused:
            self._draw_now()
        self.root.after(40, self._update_plot)

    def _draw_now(self):
        t_end = None
        for var in self.variables:
            if var.enabled and var.data:
                t_end = max(t_end or 0.0, var.data[-1][0])
        if t_end is None:
            t_end = time.time() - self.sampler.start_time

        t_start = None
        if self.window_sec is not None:
            t_start = max(0.0, t_end - self.window_sec)

        ymin = ymax = None
        for var in self.variables:
            if var.line is None:
                continue
            if not var.enabled or not var.data:
                var.line.set_data([], [])
                if var.value_label is not None:
                    var.value_label.configure(text="--")
                continue
            times = []
            values = []
            for t, value in reversed(var.data):
                if t_start is not None and t < t_start:
                    break
                times.append(t)
                values.append(value)
            times.reverse()
            values.reverse()
            var.line.set_data(times, values)
            if var.value_label is not None:
                var.value_label.configure(text=f"{values[-1]:.4g}")
            if values:
                lo, hi = min(values), max(values)
                ymin = lo if ymin is None else min(ymin, lo)
                ymax = hi if ymax is None else max(ymax, hi)

        self.ax.set_xlim(t_start if t_start is not None else 0.0,
                         t_end if t_end > 0 else 1.0)
        if ymin is not None:
            span = ymax - ymin
            pad = span * 0.12 if span > 1e-9 else max(abs(ymax) * 0.1, 0.1)
            self.ax.set_ylim(ymin - pad, ymax + pad)
        try:
            self.canvas.draw_idle()
        except Exception as exc:
            self._last_draw_error = str(exc)

    def _rebuild_legend(self):
        if self.legend is not None:
            self.legend.remove()
            self.legend = None
        handles = [v.line for v in self.variables if v.line is not None and v.enabled]
        labels = [v.expr for v in self.variables if v.line is not None and v.enabled]
        if handles:
            self.legend = self.ax.legend(handles, labels, loc="upper right",
                                         fontsize=7, facecolor=CARD_HI,
                                         edgecolor=BORDER, framealpha=0.9,
                                         labelcolor=FG)

    def _refresh_caps(self):
        cap = int(max(2000, self.sampler.rate * 62))
        for var in self.variables:
            var.cap = cap

    # ------------------------------------------------------------- toolbar
    def _mode_text(self):
        if self.demo:
            return "演示模式"
        base = "独立模式" if self.owned_process is not None else "已连接会话"
        if self.follow_thread is not None:
            followed = len([v for v in self.variables if v.source == "vscode"])
            base += f" · 跟随VS Code({followed})"
        return base

    def _toggle_run(self):
        self.sampler.running = not self.sampler.running
        self.btn_run.configure(text="开始采样" if not self.sampler.running else "停止采样")
        suffix = "" if self.sampler.running else " · 已停止采样"
        self._set_state(self._mode_text() + suffix,
                        OK if self.sampler.running else WARN)

    def _toggle_pause(self):
        self.paused = not self.paused
        self.btn_pause.configure(text="继续显示" if self.paused else "暂停显示")

    def _clear_data(self):
        for var in self.variables:
            var.clear()

    def _on_window_changed(self, event=None):
        index = self.combo_window.current()
        self.window_sec = WINDOW_CHOICES[index][1]

    def _on_rate_changed(self, event=None):
        index = self.combo_rate.current()
        self.sampler.rate = RATE_CHOICES[index]
        self._refresh_caps()

    def _on_mousewheel(self, event):
        self.var_canvas.yview_scroll(int(-event.delta / 120), "units")

    def _export_csv(self):
        path = tkinter.filedialog.asksaveasfilename(
            title="导出波形数据", defaultextension=".csv",
            filetypes=[("CSV 文件", "*.csv")])
        if not path:
            return
        import csv
        names = [v.expr for v in self.variables]
        all_times = sorted({t for v in self.variables for t, _ in v.data})
        maps = [{t: value for t, value in v.data} for v in self.variables]
        with open(path, "w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.writer(handle)
            writer.writerow(["time(s)"] + names)
            for t in all_times:
                row = [f"{t:.4f}"]
                for mapping in maps:
                    value = mapping.get(t)
                    row.append(f"{value:.6f}" if value is not None else "")
                writer.writerow(row)
        self._set_state(f"已导出 {Path(path).name}", OK)

    def _export_png(self):
        path = tkinter.filedialog.asksaveasfilename(
            title="导出波形图片", defaultextension=".png",
            filetypes=[("PNG 图片", "*.png")])
        if not path:
            return
        self.fig.savefig(path, facecolor=self.fig.get_facecolor(), dpi=150)
        self._set_state(f"已导出 {Path(path).name}", OK)

    def _set_state(self, text, color=None):
        self.state_label.configure(text=text)
        if color is not None:
            self.dot.itemconfigure(self.dot_item, fill=color)

    def _update_status(self):
        now = time.time()
        dt = now - self._last_status_time
        if dt >= 0.5:
            enabled = max(1, len([v for v in self.variables if v.enabled]))
            rate = (self.sampler.sample_count - self._last_sample_count) / dt / enabled
            self._last_sample_count = self.sampler.sample_count
            self._last_status_time = now
            text = f"采样 {rate:5.1f} Hz  错误 {self.sampler.error_count}"
            if self.sampler.last_error and not self.sampler.running:
                text += f"  {self.sampler.last_error[:40]}"
            self.rate_label.configure(text=text)
            if self.owned_process is not None and self.owned_process.proc is not None:
                if self.owned_process.proc.poll() is not None:
                    self._set_state("OpenOCD 已退出", ERR)
        self.root.after(300, self._update_status)

    def _on_close(self):
        try:
            self.sampler.stop()
            self.sampler.join(timeout=1.0)
        except Exception:
            pass
        if self.follow_thread is not None:
            self.follow_thread.stop()
        try:
            self.client.close()
        except Exception:
            pass
        if self.owned_process is not None:
            self.owned_process.stop()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Live Watch 波形显示 (通过 OpenOCD TCL 后台读内存, 不中断程序)")
    parser.add_argument("--elf", default=str(PROJECT_DIR / "build" / "vscode-debug"
                                             / "standard_robot.elf"))
    parser.add_argument("--gdb", default=None, help="arm-none-eabi-gdb 路径")
    parser.add_argument("--openocd", default=None, help="openocd 可执行文件路径")
    parser.add_argument("--cfg", default=str(PROJECT_DIR / "openocd_wireless_dap.cfg"),
                        help="独立启动 OpenOCD 时使用的配置文件")
    parser.add_argument("--var", action="append", default=[],
                        help="要监视的 GDB 表达式, 可多次指定")
    parser.add_argument("--rate", type=float, default=20.0, help="采样率 Hz (默认 20)")
    parser.add_argument("--attach", action="store_true",
                        help="只连接 VS Code/cortex-debug 已启动的 OpenOCD")
    parser.add_argument("--standalone", action="store_true",
                        help="强制自己启动 OpenOCD(默认会先尝试连接已运行的调试会话)")
    parser.add_argument("--tcl", default=None, help="直接指定 TCL 端口或 host:port")
    parser.add_argument("--check", action="store_true",
                        help="仅解析符号并打印地址/类型, 不用起界面")
    parser.add_argument("--wait-attach", type=float, nargs="?", const=120.0, default=0.0,
                        help="等待 VS Code 启动 OpenOCD 后自动连接(秒), 随调试自动打开用")
    parser.add_argument("--follow-vscode", dest="follow_vscode", action="store_true",
                        default=True, help="自动同步 VS Code Live Watch 变量(默认开启)")
    parser.add_argument("--no-follow-vscode", dest="follow_vscode", action="store_false",
                        help="不同步 VS Code Live Watch 变量")
    parser.add_argument("--demo", action="store_true", help="无硬件演示模式")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    expressions = args.var or list(DEFAULT_VARS)

    client = None
    owned_process = None
    try:
        if args.demo:
            args.gdb_path = None
            specs = [VarSpec(expr, 0x20000000 + index * 4, 4, "float", "<f")
                     for index, expr in enumerate(expressions)]
            client = DemoClient()
        else:
            args.gdb_path = find_gdb(args.gdb)
            items, errors = expand_expressions(args.gdb_path, args.elf, expressions)
            for expr, message in errors:
                print(f"跳过 {expr}: {message}", file=sys.stderr)
            specs = [spec for _, spec in items]
            if args.check:
                print(f"ELF: {args.elf}")
                for spec in specs:
                    print("  " + describe_spec(spec))
                return 0
            host = port = None
            if args.wait_attach and args.tcl is None:
                deadline = time.time() + args.wait_attach
                while time.time() < deadline:
                    try:
                        host, port = "127.0.0.1", find_openocd_tcl_port()
                        break
                    except RuntimeError:
                        time.sleep(0.5)
                if port is None:
                    raise RuntimeError(f"等待 OpenOCD 超过 {args.wait_attach:.0f} 秒, "
                                       "请确认 VS Code 调试会话已正常启动")
            elif args.tcl is not None:
                host, port = parse_host_port(args.tcl, DEFAULT_TCL_PORT)
            elif args.attach:
                host, port = "127.0.0.1", find_openocd_tcl_port()
            elif not args.standalone:
                try:
                    host, port = "127.0.0.1", find_openocd_tcl_port()
                except RuntimeError:
                    pass
            if port is not None:
                client = OpenOcdTcl(host, port)
                print(f"已连接 OpenOCD TCL {host}:{port}", flush=True)
            else:
                openocd = find_openocd(args.openocd)
                owned_process = OpenOcdProcess(openocd, args.cfg)
                print(f"启动 OpenOCD: {args.cfg}", flush=True)
                owned_process.start()
                client = OpenOcdTcl("127.0.0.1", owned_process.tcl_port)
                try:
                    client.exec("resume")
                except Exception:
                    pass
                print("OpenOCD 已就绪, 目标继续运行", flush=True)
        window = LiveWatchWindow(client, specs, args, owned_process)
        window.run()
        return 0
    except Exception as exc:
        message = str(exc)
        print(message, file=sys.stderr)
        if not args.check and sys.stderr is None:
            try:
                root = tk.Tk()
                root.withdraw()
                tkinter.messagebox.showerror("Live Watch 启动失败", message)
                root.destroy()
            except Exception:
                pass
        return 1
    finally:
        if client is not None:
            try:
                client.close()
            except Exception:
                pass
        if owned_process is not None:
            owned_process.stop()


if __name__ == "__main__":
    sys.exit(main())
