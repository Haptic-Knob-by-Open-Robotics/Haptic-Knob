"""
Haptic Knob Tuner GUI
=====================
Tkinter + matplotlib live-tuning GUI for the Haptic Knob ESP32 project.
Communicates over USB serial at 115200 baud.

Requirements:
    pip install pyserial matplotlib

Usage:
    python haptic_knob_tuner.py

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
SERIAL PROTOCOL  (what this GUI expects from the firmware)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Telemetry lines (ESP32 → PC), 50 Hz, prefix "T,":

    T,<mode>,<angle_rad>,<angle_deg>,<vel_rad_s>,<accel_rad_s2>,
      <iq_meas>,<ia>,<ib>,<ic>,<meas_ts_us>,
      <iq_cmd>,<cmd_ts_us>,
      <ctrl_en>,<trial_cfg>,<trial_act>,
      <fault_lat>,<fault_bits>,<ctrl_hb_us>,<tel_hb_us>\\n

    Field index (0-based after splitting on ","):
      [0]  "T"
      [1]  active_mode        int 0-4
      [2]  angle_rad          float
      [3]  angle_deg_wrapped  float
      [4]  velocity_rad_s     float
      [5]  acceleration_rad_s2 float
      [6]  iq_meas            float (A)
      [7]  ia                 float (A)
      [8]  ib                 float (A)
      [9]  ic                 float (A)
      [10] meas_last_update_us uint32
      [11] iq_cmd             float (A)
      [12] cmd_last_update_us uint32
      [13] control_enabled    0|1
      [14] trial_configured   0|1
      [15] trial_active       0|1
      [16] fault_latched      0|1
      [17] fault_bits         uint32 hex
      [18] ctrl_heartbeat_us  uint32
      [19] tel_heartbeat_us   uint32

Ack / error lines (ESP32 → PC):
    ACK,<echo>\\n
    ERR,<message>\\n

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
COMMANDS  (what this GUI sends to the firmware)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    SET_MODE,<0-4>                  — change active HapticMode
    SET_ENABLE,<0|1>                — enable / disable control
    ZERO                            — reset theta_origin to current angle

    SET_RES,<resistance_gain>
    SET_CAP,<k_virtual>,<b_virtual>
    SET_IND,<L>,<damping>,<alpha_db>,<omega_db>,<iq_db>
    SET_DIODE,<threshold>,<gain>
    SET_RLC,<R>,<L>,<C>,<input_gain>,<torque_gain>

    SET_PID,<mode_idx>,<QP>,<QI>,<QD>,<DP>,<DI>,<DD>

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
WHAT ANDY SHOULD IMPLEMENT IN TelemetryTask
(see full notes at bottom of this file)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import threading
import time
import tkinter as tk
from tkinter import ttk
from collections import deque
from typing import Optional

import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ═══════════════════════════════════════════════════════════════════
# Theme  (Catppuccin Mocha — matches existing project scripts)
# ═══════════════════════════════════════════════════════════════════
C = {
    "bg":      "#1e1e2e",
    "surface": "#313244",
    "overlay": "#45475a",
    "text":    "#cdd6f4",
    "subtext": "#a6adc8",
    "blue":    "#89b4fa",
    "purple":  "#cba6f7",
    "green":   "#a6e3a1",
    "red":     "#f38ba8",
    "orange":  "#fab387",
    "yellow":  "#f9e2af",
    "teal":    "#94e2d5",
    "grid":    "#44445a",
    "axes":    "#2a2a3e",
    "edge":    "#555577",
}

matplotlib.rcParams.update({
    "figure.facecolor": C["bg"],
    "axes.facecolor":   C["axes"],
    "axes.edgecolor":   C["edge"],
    "axes.labelcolor":  C["text"],
    "xtick.color":      C["subtext"],
    "ytick.color":      C["subtext"],
    "text.color":       C["text"],
    "grid.color":       C["grid"],
    "grid.linestyle":   "--",
    "grid.alpha":       0.4,
    "legend.facecolor": C["axes"],
    "legend.edgecolor": C["edge"],
    "legend.labelcolor": C["text"],
})

# ═══════════════════════════════════════════════════════════════════
# App constants
# ═══════════════════════════════════════════════════════════════════
BAUD_RATE     = 115200
PLOT_WINDOW_S = 8.0     # seconds of history visible in the plot
PLOT_MAXLEN   = 800     # deque capacity (~16 s at 50 Hz)
UI_REFRESH_MS = 50      # readout update interval  (20 Hz)
PLOT_REFRESH_MS = 50    # plot redraw interval     (20 Hz)

MODES    = ["Resistor", "Capacitor", "Inductor", "Diode", "RLC"]
MODE_IDX = {name: i for i, name in enumerate(MODES)}

# Default values mirror SharedState.h defaults
DEFAULTS = {
    "resistance_gain":     "0.1",
    "k_virtual":           "0.6",
    "b_virtual":           "0.03",
    "virtual_inductance":  "0.020",
    "inductor_damping":    "0.0",
    "alpha_deadband":      "0.0",
    "omega_deadband":      "0.0",
    "iq_deadband":         "0.0",
    "diode_threshold":     "0.1",
    "diode_gain":          "2.0",
    "virtual_resistance":  "0.5",
    "rlc_inductance":      "0.020",
    "virtual_capacitance": "0.30",
    "input_gain":          "1.0",
    "torque_gain":         "0.020",
}

# PID defaults per mode index: (qP, qI, qD, dP, dI, dD)
PID_DEFAULTS = {
    0: ("10.0", "150.0", "0.0001", "10.0", "150.0", "0.0001"),   # Resistor
    1: ("5.0",  "35.0",  "0.0001", "5.0",  "35.0",  "0.0001"),   # Capacitor
    2: ("3.0",  "300.0", "0.0",    "3.0",  "300.0", "0.0"),       # Inductor
    3: ("3.0",  "300.0", "0.0",    "3.0",  "300.0", "0.0"),       # Diode
    4: ("3.0",  "300.0", "0.0",    "3.0",  "300.0", "0.0"),       # RLC
}

# Model parameter definitions: (param_key, display_label)
MODEL_PARAMS = {
    "Resistor": [
        ("resistance_gain",    "Resistance Gain"),
    ],
    "Capacitor": [
        ("k_virtual",          "Stiffness  K"),
        ("b_virtual",          "Damping    B"),
    ],
    "Inductor": [
        ("virtual_inductance", "Inductance  L"),
        ("inductor_damping",   "Damping"),
        ("alpha_deadband",     "Accel Deadband (rad/s²)"),
        ("omega_deadband",     "Vel Deadband   (rad/s)"),
        ("iq_deadband",        "Iq Deadband    (A)"),
    ],
    "Diode": [
        ("diode_threshold",    "Threshold (rad/s)"),
        ("diode_gain",         "Gain"),
    ],
    "RLC": [
        ("virtual_resistance",  "R  (damping)"),
        ("rlc_inductance",      "L  (inertia)"),
        ("virtual_capacitance", "C  (compliance)"),
        ("input_gain",          "Input Gain"),
        ("torque_gain",         "Torque Gain"),
    ],
}

# Serial command builder per mode, given the Entry values dict
def _cmd_res(v):
    return f"SET_RES,{v['resistance_gain']}"

def _cmd_cap(v):
    return f"SET_CAP,{v['k_virtual']},{v['b_virtual']}"

def _cmd_ind(v):
    return (f"SET_IND,{v['virtual_inductance']},{v['inductor_damping']},"
            f"{v['alpha_deadband']},{v['omega_deadband']},{v['iq_deadband']}")

def _cmd_diode(v):
    return f"SET_DIODE,{v['diode_threshold']},{v['diode_gain']}"

def _cmd_rlc(v):
    return (f"SET_RLC,{v['virtual_resistance']},{v['rlc_inductance']},"
            f"{v['virtual_capacitance']},{v['input_gain']},{v['torque_gain']}")

MODEL_CMD = {
    "Resistor":  _cmd_res,
    "Capacitor": _cmd_cap,
    "Inductor":  _cmd_ind,
    "Diode":     _cmd_diode,
    "RLC":       _cmd_rlc,
}


# ═══════════════════════════════════════════════════════════════════
# Widget helpers
# ═══════════════════════════════════════════════════════════════════
def _lbl(parent, text, color=None, font=None, **kw):
    return tk.Label(
        parent, text=text,
        bg=C["bg"], fg=color or C["text"],
        font=font or ("Consolas", 9),
        **kw
    )

def _btn(parent, text, command, color=None, **kw):
    return tk.Button(
        parent, text=text, command=command,
        bg=C["surface"], fg=color or C["text"],
        activebackground=C["overlay"], activeforeground=C["text"],
        relief=tk.FLAT, padx=6, pady=2,
        font=("Consolas", 9),
        cursor="hand2",
        **kw
    )

def _entry(parent, width=10, default=""):
    e = tk.Entry(
        parent,
        bg=C["surface"], fg=C["text"],
        insertbackground=C["text"],
        relief=tk.FLAT, width=width,
        font=("Consolas", 9),
    )
    if default:
        e.insert(0, default)
    return e

def _frm(parent, **kw):
    return tk.Frame(parent, bg=C["bg"], **kw)

def _lfrm(parent, title):
    return tk.LabelFrame(
        parent, text=f" {title} ",
        bg=C["bg"], fg=C["blue"],
        font=("Consolas", 9, "bold"),
        relief=tk.GROOVE, bd=1,
        padx=4, pady=3,
    )

def _sep(parent):
    return tk.Frame(parent, bg=C["edge"], height=1)


# ═══════════════════════════════════════════════════════════════════
# Scrollable left panel helper
# ═══════════════════════════════════════════════════════════════════
class ScrollableFrame(tk.Frame):
    """A vertically scrollable frame."""
    def __init__(self, parent, width, **kw):
        super().__init__(parent, bg=C["bg"], **kw)
        canvas = tk.Canvas(self, bg=C["bg"], highlightthickness=0,
                           width=width, bd=0)
        scrollbar = ttk.Scrollbar(self, orient="vertical", command=canvas.yview)
        self.inner = tk.Frame(canvas, bg=C["bg"])
        self._win_id = canvas.create_window((0, 0), window=self.inner, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.inner.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        canvas.bind(
            "<Configure>",
            lambda e: canvas.itemconfig(self._win_id, width=e.width)
        )
        # Mouse-wheel scroll
        def _on_wheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        canvas.bind_all("<MouseWheel>", _on_wheel)


# ═══════════════════════════════════════════════════════════════════
# Main GUI
# ═══════════════════════════════════════════════════════════════════
class HapticKnobGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Haptic Knob Tuner")
        self.root.configure(bg=C["bg"])
        self.root.geometry("1280x780")
        self.root.minsize(900, 600)

        # Configure ttk combobox style
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TCombobox",
            background=C["surface"], foreground=C["text"],
            fieldbackground=C["surface"], selectbackground=C["overlay"],
            selectforeground=C["text"], arrowcolor=C["text"],
        )
        style.map("TCombobox",
            fieldbackground=[("readonly", C["surface"])],
            selectbackground=[("readonly", C["surface"])],
            selectforeground=[("readonly", C["text"])],
        )
        style.configure("Vertical.TScrollbar",
            background=C["surface"], troughcolor=C["bg"],
            arrowcolor=C["subtext"], bordercolor=C["bg"],
        )

        # ── Serial state ─────────────────────────────────────────
        self._ser: Optional[serial.Serial] = None
        self._stop_reader = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None

        # ── Thread-safe data ─────────────────────────────────────
        self._data_lock = threading.Lock()
        self._t_buf:       "deque[float]" = deque(maxlen=PLOT_MAXLEN)
        self._iq_meas_buf: "deque[float]" = deque(maxlen=PLOT_MAXLEN)
        self._iq_cmd_buf:  "deque[float]" = deque(maxlen=PLOT_MAXLEN)
        self._t0: Optional[float] = None
        self._latest: dict = {}
        self._log_lines: "deque[str]" = deque(maxlen=200)

        # ── Tk variables ─────────────────────────────────────────
        self._port_var   = tk.StringVar(value="COM7")
        self._mode_var   = tk.StringVar(value=MODES[1])   # default Capacitor
        self._enable_var = tk.BooleanVar(value=False)

        # ── Readout StringVars (keyed) ────────────────────────────
        self._rv: dict[str, tk.StringVar] = {}

        # ── Entry widget references ───────────────────────────────
        self._param_entries: dict[str, tk.Entry] = {}
        self._pid_entries:   dict[str, tk.Entry] = {}

        # ── Build UI ─────────────────────────────────────────────
        self._build_ui()
        self._refresh_param_panel(self._mode_var.get())

        # ── Start update loops ────────────────────────────────────
        self.root.after(UI_REFRESH_MS, self._update_ui)
        self.root.after(PLOT_REFRESH_MS, self._update_plot)

    # ─────────────────────────────────────────────────────────────
    # Layout
    # ─────────────────────────────────────────────────────────────
    def _build_ui(self):
        outer = _frm(self.root)
        outer.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        # Left: scrollable control panel
        self._scroll_panel = ScrollableFrame(outer, width=310)
        self._scroll_panel.pack(side=tk.LEFT, fill=tk.Y)
        left = self._scroll_panel.inner

        # Right: plot area
        right = _frm(outer)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(8, 0))

        self._build_connection(left)
        self._build_system(left)
        self._build_readouts(left)
        self._build_params_area(left)
        self._build_log(left)
        self._build_plot(right)

    # ── Connection frame ─────────────────────────────────────────
    def _build_connection(self, parent):
        f = _lfrm(parent, "Connection")
        f.pack(fill=tk.X, pady=(0, 4))

        row = _frm(f)
        row.pack(fill=tk.X, pady=1)
        _lbl(row, "Port:", width=5, anchor="w").pack(side=tk.LEFT)

        ports = [p.device for p in serial.tools.list_ports.comports()] or ["COM7"]
        if self._port_var.get() not in ports:
            ports.insert(0, self._port_var.get())
        self._port_combo = ttk.Combobox(row, textvariable=self._port_var,
                                        values=ports, width=9,
                                        font=("Consolas", 9), state="readonly")
        self._port_combo.pack(side=tk.LEFT, padx=(2, 4))
        _btn(row, "⟳", self._refresh_ports, width=2).pack(side=tk.LEFT)

        row2 = _frm(f)
        row2.pack(fill=tk.X, pady=(3, 1))
        self._connect_btn = _btn(row2, "Connect", self._toggle_connect,
                                 color=C["green"])
        self._connect_btn.pack(side=tk.LEFT)
        self._status_lbl = _lbl(row2, "Disconnected", color=C["red"])
        self._status_lbl.pack(side=tk.LEFT, padx=8)

    # ── System controls frame ─────────────────────────────────────
    def _build_system(self, parent):
        f = _lfrm(parent, "System")
        f.pack(fill=tk.X, pady=(0, 4))

        row = _frm(f)
        row.pack(fill=tk.X, pady=1)
        _lbl(row, "Mode:", width=5, anchor="w").pack(side=tk.LEFT)
        self._mode_combo = ttk.Combobox(row, textvariable=self._mode_var,
                                        values=MODES, width=11,
                                        font=("Consolas", 9), state="readonly")
        self._mode_combo.pack(side=tk.LEFT, padx=(2, 0))
        self._mode_combo.bind("<<ComboboxSelected>>", self._on_mode_selected)

        row2 = _frm(f)
        row2.pack(fill=tk.X, pady=(3, 1))
        self._enable_btn = _btn(row2, "Enable Control",
                                self._toggle_enable, color=C["green"])
        self._enable_btn.pack(side=tk.LEFT)

        row3 = _frm(f)
        row3.pack(fill=tk.X, pady=1)
        _btn(row3, "Zero Origin", self._send_zero).pack(side=tk.LEFT)
        self._fault_lbl = _lbl(row3, "  No Fault", color=C["green"])
        self._fault_lbl.pack(side=tk.LEFT)

    # ── Live readouts frame ───────────────────────────────────────
    def _build_readouts(self, parent):
        f = _lfrm(parent, "Live Readouts")
        f.pack(fill=tk.X, pady=(0, 4))

        FIELDS = [
            ("angle_deg",   "Angle (deg)"),
            ("angle_rad",   "Angle (rad)"),
            ("velocity",    "Vel   (rad/s)"),
            ("accel",       "Accel (rad/s²)"),
            ("iq_meas",     "Iq Meas   (A)"),
            ("iq_cmd",      "Iq Cmd    (A)"),
            ("ia",          "Ia        (A)"),
            ("ib",          "Ib        (A)"),
            ("ic",          "Ic        (A)"),
            ("ctrl_en",     "Ctrl Enabled"),
            ("active_mode", "Active Mode"),
        ]
        for key, label in FIELDS:
            row = _frm(f)
            row.pack(fill=tk.X, pady=0)
            _lbl(row, label, color=C["subtext"], width=16,
                 anchor="w", font=("Consolas", 8)).pack(side=tk.LEFT)
            var = tk.StringVar(value="---")
            self._rv[key] = var
            tk.Label(row, textvariable=var, bg=C["bg"], fg=C["orange"],
                     font=("Consolas", 8), anchor="e", width=12).pack(side=tk.RIGHT)

    # ── Model params area (dynamic, per-mode) ─────────────────────
    def _build_params_area(self, parent):
        self._params_outer = _lfrm(parent, "Model Parameters")
        self._params_outer.pack(fill=tk.X, pady=(0, 4))

        # Build one hidden frame per mode
        self._mode_frames: dict[str, tk.Frame] = {}
        for mode in MODES:
            frame = _frm(self._params_outer)
            self._mode_frames[mode] = frame
            self._populate_mode_frame(frame, mode)

        # PID frame (always shown below the active model frame)
        self._pid_frame = _frm(self._params_outer)
        self._build_pid_section(self._pid_frame)

    def _populate_mode_frame(self, frame, mode):
        for key, label in MODEL_PARAMS[mode]:
            row = _frm(frame)
            row.pack(fill=tk.X, pady=1)
            _lbl(row, label, color=C["subtext"], width=24,
                 anchor="w", font=("Consolas", 8)).pack(side=tk.LEFT)
            e = _entry(row, width=9, default=DEFAULTS.get(key, "0.0"))
            e.pack(side=tk.LEFT)
            self._param_entries[key] = e

        row_btn = _frm(frame)
        row_btn.pack(fill=tk.X, pady=(4, 2))
        _btn(row_btn, f"Send  {mode}  Params",
             lambda m=mode: self._send_model_params(m),
             color=C["blue"]).pack(side=tk.LEFT)

    def _build_pid_section(self, frame):
        _sep(frame).pack(fill=tk.X, pady=(3, 3))
        _lbl(frame, "Current Loop PID", color=C["purple"],
             font=("Consolas", 9, "bold")).pack(anchor="w")

        PID_ROWS = [
            ("Q-axis", "pid_qP", "pid_qI", "pid_qD"),
            ("D-axis", "pid_dP", "pid_dI", "pid_dD"),
        ]
        for axis_label, pk, ik, dk in PID_ROWS:
            row = _frm(frame)
            row.pack(fill=tk.X, pady=1)
            _lbl(row, axis_label, color=C["subtext"], width=7,
                 anchor="w", font=("Consolas", 8)).pack(side=tk.LEFT)
            for short, key in (("P", pk), ("I", ik), ("D", dk)):
                _lbl(row, f" {short}:", color=C["subtext"],
                     font=("Consolas", 8)).pack(side=tk.LEFT)
                e = _entry(row, width=7)
                e.pack(side=tk.LEFT, padx=1)
                self._pid_entries[key] = e

        row_btn = _frm(frame)
        row_btn.pack(fill=tk.X, pady=(4, 2))
        _btn(row_btn, "Send PID Gains", self._send_pid_gains,
             color=C["purple"]).pack(side=tk.LEFT)

    # ── Log frame ────────────────────────────────────────────────
    def _build_log(self, parent):
        f = _lfrm(parent, "Serial Log")
        f.pack(fill=tk.X, pady=(0, 4))
        self._log_text = tk.Text(
            f, height=5, bg=C["surface"], fg=C["subtext"],
            font=("Consolas", 8), relief=tk.FLAT,
            state=tk.DISABLED, wrap=tk.WORD,
        )
        self._log_text.pack(fill=tk.X)

    # ── Plot area ─────────────────────────────────────────────────
    def _build_plot(self, parent):
        title_row = _frm(parent)
        title_row.pack(fill=tk.X, pady=(0, 4))
        _lbl(title_row, "Iq Tracking — iq_cmd vs iq_meas",
             color=C["blue"], font=("Consolas", 11, "bold")).pack(side=tk.LEFT)

        self._fig = Figure(figsize=(8, 5), dpi=96)
        self._ax  = self._fig.add_subplot(111)
        self._ax.set_xlabel("Time (s)")
        self._ax.set_ylabel("Current (A)")
        self._ax.grid(True)
        self._ax.set_xlim(0, PLOT_WINDOW_S)
        self._ax.set_ylim(-0.5, 0.5)

        (self._line_cmd,)  = self._ax.plot([], [], lw=1.8,
                                            color=C["purple"], label="iq_cmd")
        (self._line_meas,) = self._ax.plot([], [], lw=1.8,
                                            color=C["orange"], label="iq_meas",
                                            alpha=0.9)
        self._ax.legend(loc="upper right", fontsize=9)
        self._fig.tight_layout()

        canvas = FigureCanvasTkAgg(self._fig, master=parent)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._canvas = canvas

    # ─────────────────────────────────────────────────────────────
    # Param panel switching
    # ─────────────────────────────────────────────────────────────
    def _refresh_param_panel(self, mode: str):
        for frame in self._mode_frames.values():
            frame.pack_forget()
        self._pid_frame.pack_forget()

        self._mode_frames[mode].pack(fill=tk.X)
        self._pid_frame.pack(fill=tk.X)

        # Load PID defaults for this mode
        idx = MODE_IDX.get(mode, 0)
        defaults = PID_DEFAULTS[idx]
        for key, val in zip(
            ["pid_qP", "pid_qI", "pid_qD", "pid_dP", "pid_dI", "pid_dD"],
            defaults
        ):
            e = self._pid_entries[key]
            e.delete(0, tk.END)
            e.insert(0, val)

    # ─────────────────────────────────────────────────────────────
    # Serial management
    # ─────────────────────────────────────────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()] or ["COM7"]
        self._port_combo["values"] = ports

    def _toggle_connect(self):
        if self._ser and self._ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self._port_var.get()
        try:
            self._ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
            self._stop_reader.clear()
            self._reader_thread = threading.Thread(
                target=self._serial_reader, daemon=True, name="serial-reader")
            self._reader_thread.start()
            self._status_lbl.config(text=port, fg=C["green"])
            self._connect_btn.config(text="Disconnect", fg=C["red"])
            self._log(f"Connected to {port} @ {BAUD_RATE}")
        except serial.SerialException as exc:
            self._status_lbl.config(text=f"Err: {exc}", fg=C["red"])

    def _disconnect(self):
        self._stop_reader.set()
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self._status_lbl.config(text="Disconnected", fg=C["red"])
        self._connect_btn.config(text="Connect", fg=C["green"])
        self._log("Disconnected")

    def _serial_reader(self):
        """Background daemon: read lines from serial, route to parser."""
        while not self._stop_reader.is_set():
            try:
                if not (self._ser and self._ser.is_open):
                    time.sleep(0.05)
                    continue
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="ignore").strip()
                if not line:
                    continue
                if line.startswith("T,"):
                    self._parse_telemetry(line)
                else:
                    with self._data_lock:
                        self._log_lines.append(line)
            except (serial.SerialException, OSError):
                break

    def _parse_telemetry(self, line: str):
        """Parse a T,... CSV line into the data buffers and latest dict."""
        parts = line.split(",")
        if len(parts) < 20:
            return
        try:
            d = {
                "active_mode": int(parts[1]),
                "angle_rad":   float(parts[2]),
                "angle_deg":   float(parts[3]),
                "velocity":    float(parts[4]),
                "accel":       float(parts[5]),
                "iq_meas":     float(parts[6]),
                "ia":          float(parts[7]),
                "ib":          float(parts[8]),
                "ic":          float(parts[9]),
                "iq_cmd":      float(parts[11]),
                "ctrl_en":     int(parts[13]),
                "fault_lat":   int(parts[16]),
                "fault_bits":  int(parts[17]),
            }
        except (ValueError, IndexError):
            return

        now = time.monotonic()
        with self._data_lock:
            if self._t0 is None:
                self._t0 = now
            t = now - self._t0
            self._t_buf.append(t)
            self._iq_meas_buf.append(d["iq_meas"])
            self._iq_cmd_buf.append(d["iq_cmd"])
            self._latest = d

    # ─────────────────────────────────────────────────────────────
    # Commands
    # ─────────────────────────────────────────────────────────────
    def _send_command(self, cmd: str):
        if self._ser and self._ser.is_open:
            try:
                self._ser.write((cmd + "\n").encode("ascii"))
                self._log(f">> {cmd}")
            except serial.SerialException as exc:
                self._log(f"ERR send: {exc}")
        else:
            self._log("Not connected — command dropped")

    def _on_mode_selected(self, _event=None):
        mode = self._mode_var.get()
        self._refresh_param_panel(mode)
        self._send_command(f"SET_MODE,{MODE_IDX[mode]}")

    def _toggle_enable(self):
        new_state = not self._enable_var.get()
        self._enable_var.set(new_state)
        self._send_command(f"SET_ENABLE,{1 if new_state else 0}")
        if new_state:
            self._enable_btn.config(text="Disable Control", fg=C["red"])
        else:
            self._enable_btn.config(text="Enable Control", fg=C["green"])

    def _send_zero(self):
        self._send_command("ZERO")

    def _send_model_params(self, mode: str):
        values = {key: self._param_entries[key].get().strip()
                  for key, _ in MODEL_PARAMS[mode]}
        try:
            cmd = MODEL_CMD[mode](values)
            self._send_command(cmd)
        except (KeyError, ValueError) as exc:
            self._log(f"ERR building param command: {exc}")

    def _send_pid_gains(self):
        mode = self._mode_var.get()
        idx  = MODE_IDX[mode]
        keys = ["pid_qP", "pid_qI", "pid_qD", "pid_dP", "pid_dI", "pid_dD"]
        try:
            vals = ",".join(self._pid_entries[k].get().strip() for k in keys)
            self._send_command(f"SET_PID,{idx},{vals}")
        except (KeyError, ValueError) as exc:
            self._log(f"ERR building PID command: {exc}")

    # ─────────────────────────────────────────────────────────────
    # UI update loop  (runs in main thread via root.after)
    # ─────────────────────────────────────────────────────────────
    def _update_ui(self):
        with self._data_lock:
            d = dict(self._latest)
            new_logs = list(self._log_lines)
            self._log_lines.clear()

        if d:
            self._rv["angle_deg"].set(f"{d.get('angle_deg', 0.0):>10.2f}")
            self._rv["angle_rad"].set(f"{d.get('angle_rad', 0.0):>10.4f}")
            self._rv["velocity"].set( f"{d.get('velocity',  0.0):>10.3f}")
            self._rv["accel"].set(    f"{d.get('accel',     0.0):>10.3f}")
            self._rv["iq_meas"].set(  f"{d.get('iq_meas',  0.0):>10.4f}")
            self._rv["iq_cmd"].set(   f"{d.get('iq_cmd',   0.0):>10.4f}")
            self._rv["ia"].set(       f"{d.get('ia',        0.0):>10.4f}")
            self._rv["ib"].set(       f"{d.get('ib',        0.0):>10.4f}")
            self._rv["ic"].set(       f"{d.get('ic',        0.0):>10.4f}")
            self._rv["ctrl_en"].set(  "YES" if d.get("ctrl_en") else "NO")

            mode_idx  = d.get("active_mode", 0)
            mode_name = MODES[mode_idx] if 0 <= mode_idx < len(MODES) else "?"
            self._rv["active_mode"].set(mode_name)

            # Sync mode combo if firmware reports a different mode
            if mode_name != self._mode_var.get():
                self._mode_var.set(mode_name)
                self._refresh_param_panel(mode_name)

            # Fault indicator
            if d.get("fault_lat"):
                self._fault_lbl.config(
                    text=f"  FAULT 0x{d.get('fault_bits', 0):04X}",
                    fg=C["red"])
            else:
                self._fault_lbl.config(text="  No Fault", fg=C["green"])

            # Sync enable button to firmware state
            ctrl_en = bool(d.get("ctrl_en", False))
            if ctrl_en != self._enable_var.get():
                self._enable_var.set(ctrl_en)
                if ctrl_en:
                    self._enable_btn.config(text="Disable Control", fg=C["red"])
                else:
                    self._enable_btn.config(text="Enable Control", fg=C["green"])

        # Flush log lines to text widget
        if new_logs:
            self._log_text.config(state=tk.NORMAL)
            for line in new_logs:
                self._log_text.insert(tk.END, line + "\n")
            self._log_text.see(tk.END)
            self._log_text.config(state=tk.DISABLED)

        self.root.after(UI_REFRESH_MS, self._update_ui)

    # ─────────────────────────────────────────────────────────────
    # Plot update loop  (runs in main thread via root.after)
    # ─────────────────────────────────────────────────────────────
    def _update_plot(self):
        with self._data_lock:
            if not self._t_buf:
                self.root.after(PLOT_REFRESH_MS, self._update_plot)
                return
            t    = list(self._t_buf)
            meas = list(self._iq_meas_buf)
            cmd  = list(self._iq_cmd_buf)

        t_now = t[-1]
        t_lo  = t_now - PLOT_WINDOW_S

        # Trim to the visible window
        idx0 = 0
        for i, v in enumerate(t):
            if v >= t_lo:
                idx0 = i
                break
        t    = t[idx0:]
        meas = meas[idx0:]
        cmd  = cmd[idx0:]

        self._line_meas.set_data(t, meas)
        self._line_cmd.set_data(t, cmd)

        x_lo = max(0.0, t_lo)
        self._ax.set_xlim(x_lo, t_now + 0.1)

        all_y = meas + cmd
        if all_y:
            margin = max(0.05, (max(all_y) - min(all_y)) * 0.1)
            self._ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

        self._canvas.draw_idle()
        self.root.after(PLOT_REFRESH_MS, self._update_plot)

    # ─────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────
    def _log(self, msg: str):
        """Append a message to the log widget (thread-safe)."""
        with self._data_lock:
            self._log_lines.append(msg)

    def on_close(self):
        self._stop_reader.set()
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        self.root.destroy()


# ═══════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════
def main():
    root = tk.Tk()
    app = HapticKnobGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()


# ╔══════════════════════════════════════════════════════════════════╗
# ║   WHAT ANDY SHOULD IMPLEMENT IN TelemetryTask                   ║
# ╚══════════════════════════════════════════════════════════════════╝
#
# ── Overview ────────────────────────────────────────────────────────
# TelemetryTask already exists as a stub that reads SharedState and
# updates telemetry_last_heartbeat_us.  The functions sendTelemetryLine
# and processIncomingSerialCommands are defined but never called from
# the task loop.  You need to wire them in and implement handleCommandLine.
#
# ── 1. Task loop  (TelemetryTask, TelemetryTask.cpp) ────────────────
#
#   void TelemetryTask(void *pvParameters)
#   {
#     (void)pvParameters;
#     TickType_t lastWakeTime = xTaskGetTickCount();
#     const TickType_t period = pdMS_TO_TICKS(TELEMETRY_PERIOD_MS);  // 20 ms
#
#     while (1)
#     {
#       // 1. Parse any pending commands from the PC GUI
#       processIncomingSerialCommands();
#
#       // 2. Read all shared state (one mutex grab)
#       MeasuredState measured{};
#       HapticCommand haptic{};
#       RuntimeConfig config{};
#       SystemState   state{};
#       readMeasuredState(measured);
#       readHapticCommand(haptic);
#       readRuntimeConfig(config);
#       readSystemState(state);
#
#       // 3. Emit telemetry line with "T," prefix
#       Serial.print("T,");
#       sendTelemetryLine(measured, haptic, config, state);
#
#       // 4. Update our own heartbeat
#       state.telemetry_last_heartbeat_us = micros();
#       writeSystemState(state);
#
#       vTaskDelayUntil(&lastWakeTime, period);
#     }
#   }
#
# ── 2. sendTelemetryLine  (already written, just needs T, prefix) ────
#
# The existing sendTelemetryLine() function is correct.
# The only change is to call Serial.print("T,") *before* calling it
# (see task loop above — do NOT edit sendTelemetryLine itself).
#
# The GUI expects the final format to be:
#   T,<mode>,<angle_rad>,<angle_deg>,<vel>,<accel>,<iq_meas>,
#     <ia>,<ib>,<ic>,<meas_ts>,<iq_cmd>,<cmd_ts>,
#     <ctrl_en>,<trial_cfg>,<trial_act>,<fault_lat>,<fault_bits>,
#     <ctrl_hb>,<tel_hb>
#
# ── 3. handleCommandLine  (implement this) ──────────────────────────
#
# void handleCommandLine(String &line)
# {
#   RuntimeConfig config{};
#   SystemState   state{};
#   MeasuredState measured{};
#   readRuntimeConfig(config);
#   readSystemState(state);
#
#   // ── SET_MODE,<0-4> ──────────────────────────────────────────
#   if (line.startsWith("SET_MODE,")) {
#     int idx = line.substring(9).toInt();
#     if (idx >= 0 && idx <= 4) {
#       config.active_mode = static_cast<HapticMode>(idx);
#       writeRuntimeConfig(config);
#       Serial.println("ACK,SET_MODE");
#     } else { Serial.println("ERR,bad mode index"); }
#   }
#
#   // ── SET_ENABLE,<0|1> ────────────────────────────────────────
#   else if (line.startsWith("SET_ENABLE,")) {
#     int val = line.substring(11).toInt();
#     state.control_enabled = (val != 0);
#     writeSystemState(state);
#     Serial.println("ACK,SET_ENABLE");
#   }
#
#   // ── ZERO ────────────────────────────────────────────────────
#   else if (line == "ZERO") {
#     readMeasuredState(measured);
#     config.theta_origin = measured.angle_rad;
#     writeRuntimeConfig(config);
#     Serial.println("ACK,ZERO");
#   }
#
#   // ── SET_RES,<resistance_gain> ───────────────────────────────
#   else if (line.startsWith("SET_RES,")) {
#     float r = line.substring(8).toFloat();
#     config.resistance_gain = r;
#     writeRuntimeConfig(config);
#     Serial.println("ACK,SET_RES");
#   }
#
#   // ── SET_CAP,<K>,<B> ─────────────────────────────────────────
#   else if (line.startsWith("SET_CAP,")) {
#     float k, b;
#     sscanf(line.c_str() + 8, "%f,%f", &k, &b);
#     config.k_virtual = k;
#     config.b_virtual = b;
#     writeRuntimeConfig(config);
#     Serial.println("ACK,SET_CAP");
#   }
#
#   // ── SET_IND,<L>,<damping>,<alpha_db>,<omega_db>,<iq_db> ─────
#   else if (line.startsWith("SET_IND,")) {
#     float L, damp, adb, odb, iqdb;
#     sscanf(line.c_str() + 8, "%f,%f,%f,%f,%f",
#            &L, &damp, &adb, &odb, &iqdb);
#     config.virtual_inductance  = L;
#     config.inductor_damping    = damp;
#     config.alpha_deadband      = adb;
#     config.omega_deadband      = odb;
#     config.iq_deadband         = iqdb;
#     writeRuntimeConfig(config);
#     Serial.println("ACK,SET_IND");
#   }
#
#   // ── SET_DIODE,<threshold>,<gain> ────────────────────────────
#   else if (line.startsWith("SET_DIODE,")) {
#     float thresh, gain;
#     sscanf(line.c_str() + 10, "%f,%f", &thresh, &gain);
#     config.diode_threshold = thresh;
#     config.diode_gain      = gain;
#     writeRuntimeConfig(config);
#     Serial.println("ACK,SET_DIODE");
#   }
#
#   // ── SET_RLC,<R>,<L>,<C>,<input_gain>,<torque_gain> ──────────
#   else if (line.startsWith("SET_RLC,")) {
#     float R, L, Cv, ig, tg;
#     sscanf(line.c_str() + 8, "%f,%f,%f,%f,%f",
#            &R, &L, &Cv, &ig, &tg);
#     config.virtual_resistance  = R;
#     config.virtual_inductance  = L;
#     config.virtual_capacitance = Cv;
#     config.input_gain          = ig;
#     config.torque_gain         = tg;
#     writeRuntimeConfig(config);
#     Serial.println("ACK,SET_RLC");
#   }
#
#   // ── SET_PID,<mode_idx>,<QP>,<QI>,<QD>,<DP>,<DI>,<DD> ───────
#   else if (line.startsWith("SET_PID,")) {
#     int modeIdx;
#     float qp, qi, qd, dp, di, dd;
#     sscanf(line.c_str() + 8, "%d,%f,%f,%f,%f,%f,%f",
#            &modeIdx, &qp, &qi, &qd, &dp, &di, &dd);
#     CurrentLoopPID gains = {{qp, qi, qd}, {dp, di, dd}};
#     switch (modeIdx) {
#       case 0: config.resistor_pid  = gains; break;
#       case 1: config.capacitor_pid = gains; break;
#       case 2: config.inductor_pid  = gains; break;
#       case 3: config.diode_pid     = gains; break;
#       case 4: config.rlc_pid       = gains; break;
#     }
#     writeRuntimeConfig(config);
#     Serial.println("ACK,SET_PID");
#   }
#
#   else {
#     Serial.print("ERR,unknown command: ");
#     Serial.println(line);
#   }
# }
#
# ── 4. Update rates ─────────────────────────────────────────────────
#
#   Telemetry:  50 Hz  (TELEMETRY_PERIOD_MS = 20 ms — already in Config.h)
#   Commands:   polled every task iteration (non-blocking Serial.available())
#
# ── 5. What to avoid to protect control performance ─────────────────
#
#   a. Do NOT call Serial.read/write from ControlTask or MotorControlTask.
#      All serial I/O stays in TelemetryTask only.
#
#   b. TelemetryTask priority is 2 (lowest).  MotorControlTask is 5 and
#      ModelControlTask is 4.  This is correct — telemetry gets pre-empted
#      by control tasks so it can never delay a motor control step.
#
#   c. The SharedState mutex (g_state_mutex) is held only briefly.
#      readRuntimeConfig / writeRuntimeConfig are already thread-safe.
#      Never hold the mutex while doing Serial I/O.
#
#   d. Do NOT use blocking reads (while Serial.available() == 0) or
#      delay() inside TelemetryTask.  Use vTaskDelayUntil() for timing
#      and Serial.available() checks (non-blocking) for commands.
#
#   e. Do NOT slow down telemetry below 50 Hz (or above 100 Hz).
#      At 50 Hz the GUI plot stays smooth without spamming the bus.
#      Above 100 Hz you risk starving the Watchdog task.
#
#   f. The Serial.println() at the end of sendTelemetryLine() emits a
#      trailing "\n" which is what the GUI line parser expects.
#      Do not add a second newline.
