"""
Haptic Knob — Global PID Tuner GUI
====================================
Style mirrors plot_PID_Tuning.py (Catppuccin Mocha dark theme).

Plots (always fixed, right panel):
  - Phase Currents  (Ia, Ib, Ic)
  - Iq Tracking     (IqCmd vs IqMeas)

Everything else in the telemetry schema is shown as a live number readout
in the left panel — no separate plot needed.

To add/change a model: edit only the MODELS dict below.
  "chart": "phase"  →  line goes on the Phase Currents plot
  "chart": "iq"     →  line goes on the IqCmd vs IqMeas plot
  (no "chart" key)  →  shown as a live number readout

Requirements: pip install pyserial matplotlib
Usage:        python pid_tuner.py
"""

import threading
import time
import tkinter as tk
from tkinter import ttk
from collections import deque

import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec

# ─────────────────────────────────────────────────────────────────────────────
# THEME  (Catppuccin Mocha — matches plot_PID_Tuning.py)
# ─────────────────────────────────────────────────────────────────────────────
C = {
    "bg":       "#1e1e2e",   # window / frame background
    "surface":  "#313244",   # widget backgrounds (entries, buttons)
    "overlay":  "#45475a",   # hover / active
    "text":     "#cdd6f4",   # primary text
    "subtext":  "#a6adc8",   # secondary text / axis ticks
    "blue":     "#89b4fa",   # titles, axis labels
    "purple":   "#cba6f7",   # PID label, IqCmd line
    "green":    "#a6e3a1",   # Ib line, success messages
    "red":      "#f38ba8",   # Ia line, error messages
    "orange":   "#fab387",   # IqMeas line, Ic line
    "grid":     "#44445a",   # plot gridlines
    "axes":     "#2a2a3e",   # plot axes background
    "edge":     "#555577",   # axes / legend borders
}

# Apply dark theme to matplotlib globally (affects the embedded figure)
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
})

DEFAULT_PORT = "COM7"   # change this to your usual port


# ─────────────────────────────────────────────────────────────────────────────
# MODEL SCHEMA  ←  THE ONLY THING YOU NEED TO EDIT
# ─────────────────────────────────────────────────────────────────────────────
#
# telemetry entries:
#   "chart": "phase"  →  plotted on Phase Currents chart  (Ia, Ib, Ic)
#   "chart": "iq"     →  plotted on Iq Tracking chart     (IqCmd, IqMeas)
#   no "chart" key    →  shown as a live number in the readout panel
#
# "key" must exactly match the label before ":" in your Serial.printf output.
# ─────────────────────────────────────────────────────────────────────────────

def _cap_send(p):
    return f"CAP:{p['K']:.2f},{p['B']:.2f}"

def _rlc_send(p):
    return f"RLC:{p['R']:.2f},{p['L']:.3f},{p['C']:.3f}"


MODELS = {

    # ── Resistor ──────────────────────────────────────────────────────────────
    # Serial.printf: "Angle: %7.2f deg | AngleRad: %7.3f | Vel: %7.3f rad/s |
    #   TorqueCmd: %7.4f N*m | IqCmd: %6.3f A |
    #   Ia: %6.3f A | Ib: %6.3f A | Ic: %6.3f A | IqMeas: %6.3f A | Gain: %.3f"
    "Resistor": {
        "mode_cmd": "R",
        "params": [
            {"label": "Resistance Gain", "key": "gain",
             "send": lambda v: f"G:{v:.4f}",
             "default": "0.0001", "unit": "×"},
        ],
        "telemetry": [
            {"key": "AngleRad",  "label": "Angle",      "unit": "rad"},
            {"key": "Vel",       "label": "Velocity",   "unit": "rad/s"},
            {"key": "TorqueCmd", "label": "Torque Cmd", "unit": "N·m"},
            {"key": "Gain",      "label": "Gain",       "unit": ""},
            {"key": "IqCmd",     "label": "IqCmd",      "unit": "A",  "chart": "iq"},
            {"key": "IqMeas",    "label": "IqMeas",     "unit": "A",  "chart": "iq"},
            {"key": "Ia",        "label": "Ia",         "unit": "A",  "chart": "phase"},
            {"key": "Ib",        "label": "Ib",         "unit": "A",  "chart": "phase"},
            {"key": "Ic",        "label": "Ic",         "unit": "A",  "chart": "phase"},
        ],
    },

    # ── Capacitor ─────────────────────────────────────────────────────────────
    # Serial.printf: "AngleDeg:%7.2f | AngleRad:%7.3f | Origin:%7.3f | Disp:%7.3f |
    #   Vel:%7.3f | K:%6.3f | B:%6.3f |
    #   TorqueCmd:%7.4f | IqCmd:%6.3f | IqMeas:%6.3f |
    #   Ia:%6.3f | Ib:%6.3f | Ic:%6.3f"
    "Capacitor": {
        "mode_cmd": "C",
        "send_all": _cap_send,
        "params": [
            {"label": "Spring K",  "key": "K", "default": "0.6",  "unit": "N/rad"},
            {"label": "Damping B", "key": "B", "default": "0.03", "unit": "N·s/rad"},
        ],
        "telemetry": [
            {"key": "AngleRad",  "label": "Angle",      "unit": "rad"},
            {"key": "Origin",    "label": "Origin",     "unit": "rad"},
            {"key": "Disp",      "label": "Disp",       "unit": "rad"},
            {"key": "Vel",       "label": "Velocity",   "unit": "rad/s"},
            {"key": "TorqueCmd", "label": "Torque Cmd", "unit": "N·m"},
            {"key": "IqCmd",     "label": "IqCmd",      "unit": "A",  "chart": "iq"},
            {"key": "IqMeas",    "label": "IqMeas",     "unit": "A",  "chart": "iq"},
            {"key": "Ia",        "label": "Ia",         "unit": "A",  "chart": "phase"},
            {"key": "Ib",        "label": "Ib",         "unit": "A",  "chart": "phase"},
            {"key": "Ic",        "label": "Ic",         "unit": "A",  "chart": "phase"},
        ],
    },

    # ── Inductor ──────────────────────────────────────────────────────────────
    # Serial.printf: "Angle: %7.2f deg | AngleRad: %7.3f | Vel: %7.3f rad/s |
    #   AlphaRaw: %8.3f | AlphaFilt: %8.3f |
    #   TorqueCmd: %7.4f N*m | IqCmd: %6.3f A |
    #   Ia: %6.3f A | Ib: %6.3f A | Ic: %6.3f A"
    # NOTE: no IqMeas and no live-tunable params in current firmware.
    "Inductor": {
        "mode_cmd": "L",
        "params": [],
        "telemetry": [
            {"key": "AngleRad",  "label": "Angle",      "unit": "rad"},
            {"key": "Vel",       "label": "Velocity",   "unit": "rad/s"},
            {"key": "AlphaRaw",  "label": "Accel Raw",  "unit": "rad/s²"},
            {"key": "AlphaFilt", "label": "Accel Filt", "unit": "rad/s²"},
            {"key": "TorqueCmd", "label": "Torque Cmd", "unit": "N·m"},
            {"key": "IqCmd",     "label": "IqCmd",      "unit": "A",  "chart": "iq"},
            {"key": "Ia",        "label": "Ia",         "unit": "A",  "chart": "phase"},
            {"key": "Ib",        "label": "Ib",         "unit": "A",  "chart": "phase"},
            {"key": "Ic",        "label": "Ic",         "unit": "A",  "chart": "phase"},
        ],
    },

    # ── Diode ─────────────────────────────────────────────────────────────────
    # Serial.printf: "Angle: %6.1f deg | Vel: %7.3f rad/s | Vdes: %6.3f V"
    # NOTE: voltage mode only — no current sensing, no IqMeas, no live params.
    "Diode": {
        "mode_cmd": "D",
        "params": [],
        "telemetry": [
            {"key": "Angle", "label": "Angle",    "unit": "deg"},
            {"key": "Vel",   "label": "Velocity", "unit": "rad/s"},
            {"key": "Vdes",  "label": "V cmd",    "unit": "V"},
            # no phase currents or IqMeas — voltage mode
        ],
    },

    # ── RLC ───────────────────────────────────────────────────────────────────
    # Serial.printf: "AngleDeg:%7.2f | AngleRad:%7.3f | Vel:%7.3f |
    #   R:%6.3f | L:%6.3f | C:%6.3f |
    #   iVirt:%7.4f | vCVirt:%7.4f |
    #   TorqueCmd:%7.4f | IqCmd:%6.3f | IqMeas:%6.3f |
    #   Ia:%6.3f | Ib:%6.3f | Ic:%6.3f"
    # Commands: RLC:R,L,C  |  G:inputGain  |  TG:torqueGain  |  RESET
    "RLC": {
        "mode_cmd": "RLC",
        "send_all": _rlc_send,
        "params": [
            {"label": "Resistance R",  "key": "R",  "default": "1.5",   "unit": "Ω"},
            {"label": "Inductance L",  "key": "L",  "default": "0.08",  "unit": "H"},
            {"label": "Capacitance C", "key": "C",  "default": "0.30",  "unit": "F"},
            {"label": "Input Gain",    "key": "ig", "default": "1.0",   "unit": "",
             "send": lambda v: f"G:{float(v):.4f}"},
            {"label": "Torque Gain",   "key": "tg", "default": "0.020", "unit": "",
             "send": lambda v: f"TG:{float(v):.4f}"},
        ],
        "telemetry": [
            {"key": "AngleRad",  "label": "Angle",           "unit": "rad"},
            {"key": "Vel",       "label": "Velocity",        "unit": "rad/s"},
            {"key": "iVirt",     "label": "Virtual Current", "unit": "A"},
            {"key": "vCVirt",    "label": "Virtual Voltage", "unit": "V"},
            {"key": "TorqueCmd", "label": "Torque Cmd",      "unit": "N·m"},
            {"key": "IqCmd",     "label": "IqCmd",           "unit": "A",  "chart": "iq"},
            {"key": "IqMeas",    "label": "IqMeas",          "unit": "A",  "chart": "iq"},
            {"key": "Ia",        "label": "Ia",              "unit": "A",  "chart": "phase"},
            {"key": "Ib",        "label": "Ib",              "unit": "A",  "chart": "phase"},
            {"key": "Ic",        "label": "Ic",              "unit": "A",  "chart": "phase"},
        ],
    },
}

# PID: Q-axis and D-axis use the same gains for haptic control.
# We take 3 values and send them twice: PID:Kp,Ki,Kd,Kp,Ki,Kd
PID_FIELDS = [
    ("Kp", "5.0"),
    ("Ki", "200.0"),
    ("Kd", "0.0001"),
]

BAUD_RATES         = [115200, 57600, 38400, 9600]
TELEMETRY_WINDOW_S = 8.0
ANIMATION_MS       = 30     # ~33 fps (matches plot_PID_Tuning.py)
LOG_MAX_LINES      = 100
MAX_POINTS         = 5000


# ─────────────────────────────────────────────────────────────────────────────
# SerialBridge
# ─────────────────────────────────────────────────────────────────────────────

class SerialBridge:
    def __init__(self):
        self.ser = None
        self._lock = threading.Lock()
        # Each entry: (timestamp, {key: float_value}) — full parsed telemetry row
        self.telemetry_queue = deque(maxlen=MAX_POINTS)
        self.log_queue       = deque(maxlen=LOG_MAX_LINES)
        self._running = False

    def connect(self, port, baud):
        self.disconnect()
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self._running = True
        threading.Thread(target=self._read_loop, daemon=True).start()

    def disconnect(self):
        self._running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open

    def send(self, cmd):
        if not self.connected:
            return
        with self._lock:
            self.ser.write((cmd.strip() + "\n").encode())
        self.log_queue.append(f"> {cmd.strip()}")

    def _read_loop(self):
        while self._running and self.connected:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode(errors="replace").strip()
                if not line:
                    continue
                self.log_queue.append(f"< {line}")
                self._try_parse_telemetry(line)
            except serial.SerialException:
                break

    def _try_parse_telemetry(self, line):
        """Parse "Label: value unit | Label: value unit | ..." into {key: float}.
        Non-numeric fields and lines without "|" are silently ignored.
        """
        if "|" not in line:
            return
        fields = {}
        for chunk in line.split("|"):
            chunk = chunk.strip()
            if ":" not in chunk:
                continue
            key, _, val_str = chunk.partition(":")
            key = key.strip()
            try:
                fields[key] = float(val_str.strip().split()[0])
            except (ValueError, IndexError):
                pass
        if fields:
            self.telemetry_queue.append((time.perf_counter(), fields))


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()] or ["(none)"]


def dark_entry(parent, width=10, initial=""):
    """A dark-styled tk.Entry matching the color theme."""
    e = tk.Entry(parent, width=width, bg=C["surface"], fg=C["purple"],
                 insertbackground=C["text"], relief=tk.FLAT,
                 highlightthickness=1, highlightcolor=C["edge"],
                 highlightbackground=C["edge"])
    e.insert(0, initial)
    return e


def dark_button(parent, text, command, color=None):
    bg = color or C["surface"]
    return tk.Button(parent, text=text, command=command,
                     bg=bg, fg=C["text"], activebackground=C["overlay"],
                     activeforeground=C["text"], relief=tk.FLAT,
                     padx=8, pady=3, cursor="hand2")


def dark_label(parent, text="", fg=None, font=None):
    kw = dict(bg=C["bg"], fg=fg or C["text"], text=text)
    if font:
        kw["font"] = font
    return tk.Label(parent, **kw)


# ─────────────────────────────────────────────────────────────────────────────
# TunerApp
# ─────────────────────────────────────────────────────────────────────────────

class TunerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Haptic Knob — PID Tuner")
        self.root.configure(bg=C["bg"])
        self.bridge = SerialBridge()

        # Latest telemetry snapshot (written by _poll_log, read by animation)
        self._latest: dict = {}

        self._build_ui()
        self._poll_log()

    # ── Build UI ─────────────────────────────────────────────────────────────

    def _build_ui(self):
        self._build_topbar()

        # Middle: left controls + right plots (expands to fill space)
        pane = tk.PanedWindow(self.root, orient=tk.HORIZONTAL,
                              bg=C["bg"], sashrelief=tk.FLAT, sashwidth=4)
        pane.pack(fill=tk.BOTH, expand=True, padx=6, pady=(4, 0))

        left = tk.Frame(pane, bg=C["bg"], width=270)
        pane.add(left, minsize=250)

        right = tk.Frame(pane, bg=C["bg"])
        pane.add(right, minsize=500)

        self._build_left(left)
        self._build_plot(right)

        # Bottom: serial log — full width
        self._build_log_bar()

    def _build_topbar(self):
        bar = tk.Frame(self.root, bg=C["surface"])
        bar.pack(fill=tk.X, padx=0, pady=0)

        dark_label(bar, "Port:", fg=C["subtext"]).pack(side=tk.LEFT, padx=(8, 2), pady=6)
        self.port_var = tk.StringVar()
        ports = list_serial_ports()
        self.port_var.set(DEFAULT_PORT if DEFAULT_PORT in ports else ports[0])
        self.port_cb = ttk.Combobox(bar, textvariable=self.port_var, values=ports, width=10)
        self.port_cb.pack(side=tk.LEFT, padx=2)

        dark_label(bar, "Baud:", fg=C["subtext"]).pack(side=tk.LEFT, padx=(8, 2))
        self.baud_var = tk.StringVar(value=str(BAUD_RATES[0]))
        ttk.Combobox(bar, textvariable=self.baud_var, values=BAUD_RATES, width=8
                     ).pack(side=tk.LEFT, padx=2)

        self.connect_btn = dark_button(bar, "Connect", self._toggle_connect)
        self.connect_btn.pack(side=tk.LEFT, padx=(8, 2))

        dark_button(bar, "Refresh", self._refresh_ports).pack(side=tk.LEFT, padx=2)
        dark_button(bar, "ZERO", lambda: self.bridge.send("ZERO"),
                    color="#5c4a00").pack(side=tk.LEFT, padx=(8, 2))

        self.status_lbl = dark_label(bar, "Disconnected", fg=C["red"])
        self.status_lbl.pack(side=tk.LEFT, padx=12)

    def _build_left(self, parent):
        # ── Model selector ───────────────────────────────────────────────────
        sec = self._section(parent, "Haptic Model")
        self.model_var = tk.StringVar(value=list(MODELS.keys())[0])
        cb = ttk.Combobox(sec, textvariable=self.model_var,
                          values=list(MODELS.keys()), state="readonly", width=18)
        cb.pack(padx=6, pady=4, anchor="w")
        cb.bind("<<ComboboxSelected>>", self._on_model_change)

        # ── Model params ─────────────────────────────────────────────────────
        self.param_section = self._section(parent, "Model Parameters")
        self._param_entries = {}   # key → tk.Entry
        self._build_param_panel()

        # ── Live readouts ────────────────────────────────────────────────────
        self.readout_section = self._section(parent, "Live Values")
        self._readout_labels = {}  # telemetry key → tk.Label (value display)
        self._build_readout_panel()

        # ── PID controls ─────────────────────────────────────────────────────
        pid_sec = self._section(parent, "Current PID  (Q = D axis)")
        self._pid_entries = {}
        for i, (lbl, default) in enumerate(PID_FIELDS):
            dark_label(pid_sec, lbl + ":", fg=C["subtext"]).grid(
                row=i*2, column=0, sticky="w", padx=6)
            e = dark_entry(pid_sec, width=12, initial=default)
            e.grid(row=i*2+1, column=0, padx=6, pady=(0, 4), sticky="w")
            self._pid_entries[lbl] = e

        dark_button(pid_sec, "Apply PID", self._send_pid,
                    color="#1a3a5c").grid(
            row=len(PID_FIELDS)*2, column=0, pady=6, sticky="ew", padx=6)

        self._pid_status = dark_label(pid_sec, "", fg=C["green"])
        self._pid_status.grid(row=len(PID_FIELDS)*2+1, column=0, pady=(0, 4))

    def _build_log_bar(self):
        """Serial log pinned to the bottom of the window, full width."""
        outer = tk.Frame(self.root, bg=C["bg"])
        outer.pack(fill=tk.X, padx=6, pady=(2, 6))
        tk.Label(outer, text="  Serial Log  ", bg=C["surface"], fg=C["blue"],
                 font=("Helvetica", 9, "bold")).pack(anchor="w")
        inner = tk.Frame(outer, bg=C["bg"])
        inner.pack(fill=tk.X)
        self.log_text = tk.Text(inner, height=5, bg=C["surface"], fg=C["subtext"],
                                font=("Courier", 8), state=tk.DISABLED,
                                relief=tk.FLAT, insertbackground=C["text"])
        sb = tk.Scrollbar(inner, command=self.log_text.yview,
                          bg=C["surface"], troughcolor=C["bg"])
        self.log_text.configure(yscrollcommand=sb.set)
        self.log_text.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4, pady=4)
        sb.pack(side=tk.RIGHT, fill=tk.Y)

    def _section(self, parent, title):
        """Create a titled section frame (dark styled)."""
        outer = tk.Frame(parent, bg=C["bg"])
        outer.pack(fill=tk.X, padx=4, pady=(6, 0))
        tk.Label(outer, text=f"  {title}  ", bg=C["surface"], fg=C["blue"],
                 font=("Helvetica", 9, "bold")).pack(anchor="w")
        inner = tk.Frame(outer, bg=C["bg"])
        inner.pack(fill=tk.X)
        return inner

    def _build_param_panel(self):
        """Rebuild model param entry rows from current model's schema."""
        for w in self.param_section.winfo_children():
            w.destroy()
        self._param_entries.clear()

        model = MODELS[self.model_var.get()]

        if not model["params"]:
            dark_label(self.param_section,
                       "No live-tunable params in firmware.",
                       fg=C["subtext"]).pack(padx=6, pady=6, anchor="w")
            return

        for p in model["params"]:
            row = tk.Frame(self.param_section, bg=C["bg"])
            row.pack(fill=tk.X, padx=6, pady=2)
            dark_label(row, f"{p['label']} ({p.get('unit','')})",
                       fg=C["subtext"]).pack(side=tk.LEFT)
            e = dark_entry(row, width=10, initial=p["default"])
            e.pack(side=tk.RIGHT, padx=4)
            # If param has its own send function, fire on Enter
            if "send" in p:
                def _on_enter(event, _p=p, _e=e):
                    try:
                        self.bridge.send(_p["send"](_e.get()))
                    except (ValueError, TypeError):
                        pass
                e.bind("<Return>", _on_enter)
            self._param_entries[p["key"]] = e

        # Always show Apply button — Enter key also works per-field
        dark_button(self.param_section, "Apply",
                    self._send_model_params, color="#1a4a1a").pack(
            pady=4, padx=6, anchor="w")

    def _build_readout_panel(self):
        """Rebuild live-value labels for non-chart telemetry fields."""
        for w in self.readout_section.winfo_children():
            w.destroy()
        self._readout_labels.clear()

        model = MODELS[self.model_var.get()]
        readout_fields = [f for f in model["telemetry"] if "chart" not in f]

        if not readout_fields:
            dark_label(self.readout_section, "No readout fields.",
                       fg=C["subtext"]).pack(padx=6, pady=4, anchor="w")
            return

        grid = tk.Frame(self.readout_section, bg=C["bg"])
        grid.pack(fill=tk.X, padx=6, pady=4)

        for i, f in enumerate(readout_fields):
            # Label name
            dark_label(grid, f"{f['label']}:", fg=C["subtext"]).grid(
                row=i, column=0, sticky="w", pady=1)
            # Value (updated by _poll_log)
            val_lbl = dark_label(grid, "---", fg=C["purple"],
                                 font=("Courier", 10))
            val_lbl.grid(row=i, column=1, sticky="w", padx=8)
            # Unit
            dark_label(grid, f['unit'], fg=C["subtext"]).grid(
                row=i, column=2, sticky="w")
            self._readout_labels[f["key"]] = val_lbl

    # ── Plot ─────────────────────────────────────────────────────────────────

    def _build_plot(self, parent):
        """Build the matplotlib figure: Active PID label | 2 plots | stats bar."""
        fig = Figure(figsize=(7, 5), facecolor=C["bg"])
        fig.suptitle("Haptic Knob — Live Telemetry", fontsize=13,
                     fontweight="bold", color=C["purple"])

        # Layout: 3 rows  (PID label | plots | stats)
        gs = gridspec.GridSpec(3, 2, figure=fig,
                               height_ratios=[0.10, 1, 0.10],
                               hspace=0.50, wspace=0.35)

        # ── Active PID label ─────────────────────────────────────────────────
        ax_pid = fig.add_subplot(gs[0, :])
        ax_pid.axis("off")
        self._pid_label_text = ax_pid.text(
            0.5, 0.5, "Active PID: ---",
            transform=ax_pid.transAxes, ha="center", va="center",
            fontsize=9, fontfamily="monospace", color=C["green"],
            bbox=dict(boxstyle="round,pad=0.4",
                      facecolor=C["surface"], edgecolor=C["edge"]),
        )

        # ── Phase Currents ───────────────────────────────────────────────────
        ax_ph = fig.add_subplot(gs[1, 0])
        ax_ph.set_title("Phase Currents", color=C["blue"], fontsize=10)
        ax_ph.set_xlabel("Time (s)"); ax_ph.set_ylabel("Current (A)")
        ax_ph.grid(True)
        self._line_ia, = ax_ph.plot([], [], color=C["red"],    lw=1.0, label="Ia")
        self._line_ib, = ax_ph.plot([], [], color=C["green"],  lw=1.0, label="Ib")
        self._line_ic, = ax_ph.plot([], [], color=C["orange"], lw=1.0, label="Ic")
        ax_ph.legend(loc="upper right", fontsize=8)
        self._ax_ph = ax_ph

        # ── Iq Tracking ──────────────────────────────────────────────────────
        ax_iq = fig.add_subplot(gs[1, 1])
        ax_iq.set_title("IqCmd vs IqMeas", color=C["blue"], fontsize=10)
        ax_iq.set_xlabel("Time (s)"); ax_iq.set_ylabel("Current (A)")
        ax_iq.grid(True)
        self._line_iqcmd,  = ax_iq.plot([], [], color=C["purple"], lw=1.4,
                                         label="IqCmd",  zorder=3)
        self._line_iqmeas, = ax_iq.plot([], [], color=C["orange"], lw=1.0, ls="--",
                                         label="IqMeas", zorder=2)
        ax_iq.legend(loc="upper right", fontsize=8)
        self._ax_iq = ax_iq

        # ── Stats bar ────────────────────────────────────────────────────────
        ax_stats = fig.add_subplot(gs[2, :])
        ax_stats.axis("off")
        self._stats_text = ax_stats.text(
            0.5, 0.5, "Waiting for data…",
            transform=ax_stats.transAxes, ha="center", va="center",
            fontsize=8, fontfamily="monospace", color=C["text"],
        )

        canvas = FigureCanvasTkAgg(fig, master=parent)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._canvas = canvas
        self._fig = fig

        # FuncAnimation drives the plot refresh (~33 fps)
        self._ani = FuncAnimation(fig, self._animate,
                                  interval=ANIMATION_MS,
                                  blit=False, cache_frame_data=False)

    def _animate(self, _frame):
        """Called by FuncAnimation. Reads telemetry_queue and updates plots."""
        q = self.bridge.telemetry_queue
        if not q:
            return

        now    = time.perf_counter()
        cutoff = now - TELEMETRY_WINDOW_S
        data   = [(t, r) for t, r in q if t >= cutoff]
        if not data:
            return

        ts = [d[0] - now for d in data]   # relative time: 0 = now

        def col(key):
            return [d[1].get(key, float("nan")) for d in data]

        # Phase currents
        self._line_ia.set_data(ts, col("Ia"))
        self._line_ib.set_data(ts, col("Ib"))
        self._line_ic.set_data(ts, col("Ic"))
        self._ax_ph.set_xlim(-TELEMETRY_WINDOW_S, 0)
        ph_vals = [v for k in ("Ia","Ib","Ic") for v in col(k)
                   if v == v]  # filter NaN
        if ph_vals:
            pad = max(0.05, (max(ph_vals) - min(ph_vals)) * 0.15)
            self._ax_ph.set_ylim(min(ph_vals) - pad, max(ph_vals) + pad)

        # Iq tracking
        self._line_iqcmd.set_data(ts,  col("IqCmd"))
        self._line_iqmeas.set_data(ts, col("IqMeas"))
        self._ax_iq.set_xlim(-TELEMETRY_WINDOW_S, 0)
        iq_vals = [v for k in ("IqCmd","IqMeas") for v in col(k) if v == v]
        if iq_vals:
            pad = max(0.05, (max(iq_vals) - min(iq_vals)) * 0.15)
            self._ax_iq.set_ylim(min(iq_vals) - pad, max(iq_vals) + pad)

        # Stats bar
        n  = len(data)
        hz = (n-1) / (data[-1][0] - data[0][0]) if n >= 2 else 0
        last = data[-1][1]
        iqc  = last.get("IqCmd",  float("nan"))
        iqm  = last.get("IqMeas", float("nan"))
        ia   = last.get("Ia",     float("nan"))
        ib   = last.get("Ib",     float("nan"))
        ic   = last.get("Ic",     float("nan"))
        self._stats_text.set_text(
            f"Samples: {n}   Rate: {hz:.1f} Hz   |   "
            f"Ia: {ia:+.3f}  Ib: {ib:+.3f}  Ic: {ic:+.3f} A   |   "
            f"IqCmd: {iqc:+.3f}  IqMeas: {iqm:+.3f} A"
        )

        # Active PID label
        try:
            kp = self._pid_entries["Kp"].get()
            ki = self._pid_entries["Ki"].get()
            kd = self._pid_entries["Kd"].get()
            self._pid_label_text.set_text(
                f"Active PID  |  Kp={kp}   Ki={ki}   Kd={kd}   (Q = D axis)"
            )
        except Exception:
            pass

        # Update readout labels (non-chart telemetry fields)
        for key, lbl in self._readout_labels.items():
            val = last.get(key)
            if val is not None:
                lbl.config(text=f"{val:+.4f}")

    # ── Event handlers ───────────────────────────────────────────────────────

    def _on_model_change(self, _=None):
        model = MODELS[self.model_var.get()]
        self._build_param_panel()
        self._build_readout_panel()
        self.bridge.send(model["mode_cmd"])

    def _send_model_params(self):
        """Apply button handler — sends all current param values to firmware."""
        model = MODELS[self.model_var.get()]
        if "send_all" in model:
            # Multi-param: collect all values and send in one command
            vals = {k: e.get() for k, e in self._param_entries.items()}
            try:
                numeric = {k: float(v) for k, v in vals.items()}
                self.bridge.send(model["send_all"](numeric))
            except ValueError:
                pass
        else:
            # Single-param: send each param individually using its own send function
            for p in model["params"]:
                if "send" in p and p["key"] in self._param_entries:
                    try:
                        val = self._param_entries[p["key"]].get()
                        self.bridge.send(p["send"](val))
                    except (ValueError, TypeError):
                        pass

    def _send_pid(self):
        try:
            kp = float(self._pid_entries["Kp"].get())
            ki = float(self._pid_entries["Ki"].get())
            kd = float(self._pid_entries["Kd"].get())
        except ValueError:
            self._pid_status.config(text="Invalid value", fg=C["red"])
            return
        # Send same gains for both Q-axis and D-axis
        self.bridge.send(f"PID:{kp},{ki},{kd},{kp},{ki},{kd}")
        self._pid_status.config(
            text=f"Sent  Kp={kp}  Ki={ki}  Kd={kd}", fg=C["green"])
        self.root.after(3000, lambda: self._pid_status.config(text=""))

    def _toggle_connect(self):
        if self.bridge.connected:
            self.bridge.disconnect()
            self.connect_btn.config(text="Connect")
            self.status_lbl.config(text="Disconnected", fg=C["red"])
        else:
            port = self.port_var.get()
            baud = int(self.baud_var.get())
            try:
                self.bridge.connect(port, baud)
                self.connect_btn.config(text="Disconnect")
                self.status_lbl.config(text=f"Connected  {port}@{baud}", fg=C["green"])
            except serial.SerialException as e:
                self.status_lbl.config(text=f"Error: {e}", fg=C["red"])

    def _refresh_ports(self):
        ports = list_serial_ports()
        self.port_cb["values"] = ports
        if ports:
            self.port_var.set(DEFAULT_PORT if DEFAULT_PORT in ports else ports[0])

    # ── Polling ──────────────────────────────────────────────────────────────

    def _poll_log(self):
        """Drain the serial log queue into the text box (100ms)."""
        while self.bridge.log_queue:
            line = self.bridge.log_queue.popleft()
            self.log_text.config(state=tk.NORMAL)
            self.log_text.insert(tk.END, line + "\n")
            lines = int(self.log_text.index("end-1c").split(".")[0])
            if lines > LOG_MAX_LINES:
                self.log_text.delete("1.0", f"{lines - LOG_MAX_LINES}.0")
            self.log_text.see(tk.END)
            self.log_text.config(state=tk.DISABLED)
        self.root.after(100, self._poll_log)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    root = tk.Tk()
    root.minsize(820, 560)
    app = TunerApp(root)
    root.mainloop()
