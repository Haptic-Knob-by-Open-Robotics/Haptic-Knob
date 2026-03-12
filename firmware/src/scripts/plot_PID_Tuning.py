"""
Haptic Knob Serial Monitor
--------------------------
Live plots + mid-run tuning controls:
  - Gain multiplier slider  (sends  G:<value>)
  - PID fields + Send button (sends  PID:QP,QI,QD,DP,DI,DD)

Dependencies:
    pip install pyserial matplotlib

Usage:
    python plot_resistor.py --port COM7 --pid 2.0 200.0 0.0 2.0 200.0 0.0
    python plot_resistor.py --demo
"""

import argparse
import queue
import re
import threading
import time
from collections import deque

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import TextBox, Button, Slider
import serial

# ─────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────
WINDOW_SECONDS = 8
MAX_POINTS     = 5000     # deeper buffer for 50 Hz
ANIMATION_MS   = 30       # ~33 fps GUI refresh

# ─────────────────────────────────────────────
#  REGEX
# ─────────────────────────────────────────────
TELEM_RE = re.compile(
    r"Angle:\s*([\-\d.]+)\s*deg\s*\|"
    r"\s*AngleRad:\s*([\-\d.]+)\s*\|"
    r"\s*Vel:\s*([\-\d.]+)\s*rad/s\s*\|"
    r"\s*TorqueCmd:\s*([\-\d.]+)\s*N\*m\s*\|"
    r"\s*IqCmd:\s*([\-\d.]+)\s*A\s*\|"
    r"\s*Ia:\s*([\-\d.]+)\s*A\s*\|"
    r"\s*Ib:\s*([\-\d.]+)\s*A\s*\|"
    r"\s*Ic:\s*([\-\d.]+)\s*A"
    r"(?:\s*\|\s*IqMeas:\s*([\-\d.]+)\s*A)?"
    r"(?:\s*\|\s*Gain:\s*([\-\d.]+))?"
)
PID_Q_RE = re.compile(r"Q-axis:\s*P=([\-\d.]+),\s*I=([\-\d.]+),\s*D=([\-\d.]+)")
PID_D_RE = re.compile(r"D-axis:\s*P=([\-\d.]+),\s*I=([\-\d.]+),\s*D=([\-\d.]+)")

# ─────────────────────────────────────────────
#  SHARED STATE
# ─────────────────────────────────────────────
lock = threading.Lock()

timestamps  = deque(maxlen=MAX_POINTS)
ia_buf      = deque(maxlen=MAX_POINTS)
ib_buf      = deque(maxlen=MAX_POINTS)
ic_buf      = deque(maxlen=MAX_POINTS)
iq_cmd_buf  = deque(maxlen=MAX_POINTS)
iq_meas_buf = deque(maxlen=MAX_POINTS)
gain_buf    = deque(maxlen=MAX_POINTS)

pid_info = {
    "Q_P": None, "Q_I": None, "Q_D": None,
    "D_P": None, "D_I": None, "D_D": None,
}
current_gain = {"val": 1.0}

# Queue: GUI → serial thread
send_queue: queue.Queue = queue.Queue()

# Feedback message shown below the Send PID button
send_status = {"msg": "", "ts": 0.0}


# ─────────────────────────────────────────────
#  SERIAL THREAD
# ─────────────────────────────────────────────
def serial_reader(port, baud, stop_event, pid_gains):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"[serial] Opened {port} @ {baud} baud")
    except serial.SerialException as e:
        print(f"[serial] ERROR: {e}")
        stop_event.set()
        return

    pid_sent = False

    while not stop_event.is_set():
        # Drain outgoing queue first
        while not send_queue.empty():
            try:
                msg = send_queue.get_nowait()
                ser.write(msg.encode("utf-8"))
                print(f"[serial] >> {msg.strip()}")
            except (queue.Empty, serial.SerialException):
                break

        try:
            raw = ser.readline()
        except serial.SerialException:
            print("[serial] Read error")
            stop_event.set()
            break

        try:
            line = raw.decode("utf-8", errors="replace").strip()
        except Exception:
            continue
        if not line:
            continue

        print(f"[esp32] {line}")

        # Startup PID handshake
        if not pid_sent and "Example:" in line:
            gains_str = "{} {} {} {} {} {}\n".format(*pid_gains)
            ser.write(gains_str.encode("utf-8"))
            print(f"[serial] Sent startup gains: {gains_str.strip()}")
            pid_sent = True
            continue

        # PID confirmation echo
        m = PID_Q_RE.search(line)
        if m:
            with lock:
                pid_info["Q_P"] = float(m.group(1))
                pid_info["Q_I"] = float(m.group(2))
                pid_info["Q_D"] = float(m.group(3))
            continue
        m = PID_D_RE.search(line)
        if m:
            with lock:
                pid_info["D_P"] = float(m.group(1))
                pid_info["D_I"] = float(m.group(2))
                pid_info["D_D"] = float(m.group(3))
            continue

        # Telemetry
        m = TELEM_RE.search(line)
        if m:
            iq_cmd  = float(m.group(5))
            ia      = float(m.group(6))
            ib      = float(m.group(7))
            ic      = float(m.group(8))
            iq_meas = float(m.group(9)) if m.group(9) is not None else float("nan")
            gain    = float(m.group(10)) if m.group(10) is not None else current_gain["val"]

            t = time.perf_counter()
            with lock:
                timestamps.append(t)
                ia_buf.append(ia)
                ib_buf.append(ib)
                ic_buf.append(ic)
                iq_cmd_buf.append(iq_cmd)
                iq_meas_buf.append(iq_meas)
                gain_buf.append(gain)
                current_gain["val"] = gain

    ser.close()
    print("[serial] Closed.")


# ─────────────────────────────────────────────
#  DEMO
# ─────────────────────────────────────────────
def demo_generator(stop_event):
    import math, random
    print("[demo] Simulation mode.")
    t0 = time.perf_counter()
    g = 1.0
    with lock:
        pid_info.update({"Q_P": 2.0, "Q_I": 200.0, "Q_D": 0.0,
                         "D_P": 2.0, "D_I": 200.0, "D_D": 0.0})
    while not stop_event.is_set():
        t = time.perf_counter() - t0
        # Check if GUI sent a gain update
        while not send_queue.empty():
            try:
                msg = send_queue.get_nowait()
                if msg.startswith("G:"):
                    g = float(msg[2:])
                    with lock:
                        current_gain["val"] = g
                elif msg.startswith("PID:"):
                    parts = [float(x) for x in msg[4:].split(",")]
                    if len(parts) == 6:
                        with lock:
                            pid_info.update({
                                "Q_P": parts[0], "Q_I": parts[1], "Q_D": parts[2],
                                "D_P": parts[3], "D_I": parts[4], "D_D": parts[5],
                            })
            except (queue.Empty, ValueError):
                break

        omega = 2.0 * math.pi * 0.5
        ia     =  1.5 * math.sin(omega * t) + random.gauss(0, 0.02)
        ib     =  1.5 * math.sin(omega * t + 2.094) + random.gauss(0, 0.02)
        ic     =  1.5 * math.sin(omega * t + 4.189) + random.gauss(0, 0.02)
        iq_cmd =  min(2.0, max(-2.0, g * 1.2 * math.sin(omega * t * 0.3)))
        iq_meas = iq_cmd * 0.92 + random.gauss(0, 0.03)

        now = time.perf_counter()
        with lock:
            timestamps.append(now)
            ia_buf.append(ia); ib_buf.append(ib); ic_buf.append(ic)
            iq_cmd_buf.append(iq_cmd); iq_meas_buf.append(iq_meas)
            gain_buf.append(g)

        time.sleep(0.02)  # 50 Hz


# ─────────────────────────────────────────────
#  GUI
# ─────────────────────────────────────────────
def _pid_label():
    with lock:
        qp = pid_info["Q_P"]; qi = pid_info["Q_I"]; qd = pid_info["Q_D"]
        dp = pid_info["D_P"]; di = pid_info["D_I"]; dd = pid_info["D_D"]
    def fmt(v): return f"{v:.3f}" if v is not None else "---"
    return (f"Active PID  |  Q: P={fmt(qp)}  I={fmt(qi)}  D={fmt(qd)}"
            f"   |   D: P={fmt(dp)}  I={fmt(di)}  D={fmt(dd)}")


def build_gui(initial_pid, is_demo):
    matplotlib.rcParams.update({
        "figure.facecolor": "#1e1e2e",
        "axes.facecolor":   "#2a2a3e",
        "axes.edgecolor":   "#555577",
        "axes.labelcolor":  "#cdd6f4",
        "xtick.color":      "#a6adc8",
        "ytick.color":      "#a6adc8",
        "text.color":       "#cdd6f4",
        "grid.color":       "#44445a",
        "grid.linestyle":   "--",
        "grid.alpha":       0.4,
        "legend.facecolor": "#2a2a3e",
        "legend.edgecolor": "#555577",
    })

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle("Haptic Knob – Live Telemetry", fontsize=14,
                 fontweight="bold", color="#cba6f7")

    # Layout: 4 rows
    #   row 0 (small)  — active PID display
    #   row 1 (plots)  — phase currents | iq tracking
    #   row 2 (small)  — stats bar
    #   row 3 (medium) — gain slider | PID inputs
    gs = gridspec.GridSpec(4, 2, figure=fig,
                           height_ratios=[0.12, 1, 0.10, 0.38],
                           hspace=0.55, wspace=0.35)

    # ── Active PID label ─────────────────────────────────────────────
    ax_pid = fig.add_subplot(gs[0, :])
    ax_pid.axis("off")
    pid_text = ax_pid.text(
        0.5, 0.5, _pid_label(),
        transform=ax_pid.transAxes, ha="center", va="center",
        fontsize=10, fontfamily="monospace", color="#a6e3a1",
        bbox=dict(boxstyle="round,pad=0.4", facecolor="#313244", edgecolor="#555577"),
    )

    # ── Phase currents ────────────────────────────────────────────────
    ax_phases = fig.add_subplot(gs[1, 0])
    ax_phases.set_title("Phase Currents", color="#89b4fa")
    ax_phases.set_xlabel("Time (s)"); ax_phases.set_ylabel("Current (A)")
    ax_phases.grid(True)
    line_ia, = ax_phases.plot([], [], color="#f38ba8", lw=1.0, label="Ia")
    line_ib, = ax_phases.plot([], [], color="#a6e3a1", lw=1.0, label="Ib")
    line_ic, = ax_phases.plot([], [], color="#89b4fa", lw=1.0, label="Ic")
    ax_phases.legend(loc="upper right", fontsize=8)

    # ── Iq tracking ───────────────────────────────────────────────────
    ax_iq = fig.add_subplot(gs[1, 1])
    ax_iq.set_title("IqCmd vs IqMeas", color="#89b4fa")
    ax_iq.set_xlabel("Time (s)"); ax_iq.set_ylabel("Current (A)")
    ax_iq.grid(True)
    line_iq_cmd,  = ax_iq.plot([], [], color="#cba6f7", lw=1.4,
                                label="IqCmd (desired)", zorder=3)
    line_iq_meas, = ax_iq.plot([], [], color="#fab387", lw=1.0, ls="--",
                                label="IqMeas (motor.current.q)", zorder=2)
    ax_iq.legend(loc="upper right", fontsize=8)

    # ── Stats bar ─────────────────────────────────────────────────────
    ax_stats = fig.add_subplot(gs[2, :])
    ax_stats.axis("off")
    stats_text = ax_stats.text(
        0.5, 0.5, "Waiting for data…",
        transform=ax_stats.transAxes, ha="center", va="center",
        fontsize=9, fontfamily="monospace", color="#cdd6f4",
    )

    # ── Control panel ─────────────────────────────────────────────────
    # Left half: gain slider
    ax_ctrl_left = fig.add_subplot(gs[3, 0])
    ax_ctrl_left.axis("off")
    ax_ctrl_left.set_title("Resistance Gain Multiplier", color="#89b4fa", fontsize=10)

    # Gain slider axes (manually positioned inside the subplot)
    sl_pos = ax_ctrl_left.get_position()
    ax_slider = fig.add_axes([
        sl_pos.x0 + 0.02,
        sl_pos.y0 + 0.06,
        sl_pos.width - 0.04,
        0.025,
    ])
    gain_slider = Slider(
        ax_slider, "Gain", 0.001, 2.0,
        valinit=1.0, valstep=0.001,
        color="#89b4fa", track_color="#313244",
    )
    gain_slider.label.set_color("#cdd6f4")
    gain_slider.valtext.set_color("#cba6f7")

    gain_status_ax = fig.add_axes([sl_pos.x0 + 0.02, sl_pos.y0 + 0.01,
                                   sl_pos.width - 0.04, 0.03])
    gain_status_ax.axis("off")
    gain_status_text = gain_status_ax.text(
        0.5, 0.5, f"Effective resistance = 1.0 × 1.0 = 1.0",
        ha="center", va="center", fontsize=9,
        fontfamily="monospace", color="#a6adc8",
        transform=gain_status_ax.transAxes,
    )

    # Right half: PID inputs
    ax_ctrl_right = fig.add_subplot(gs[3, 1])
    ax_ctrl_right.axis("off")
    ax_ctrl_right.set_title("Live PID Update", color="#89b4fa", fontsize=10)

    rp = ax_ctrl_right.get_position()
    box_h   = 0.028
    box_gap = 0.008
    labels  = ["Q_P", "Q_I", "Q_D", "D_P", "D_I", "D_D"]
    defaults = [str(v) for v in initial_pid]

    text_boxes = {}
    n_boxes = len(labels)
    cols = 3
    col_w = (rp.width - 0.02) / cols

    for i, (lbl, dflt) in enumerate(zip(labels, defaults)):
        col = i % cols
        row = i // cols
        bx = rp.x0 + 0.01 + col * col_w
        by = rp.y1 - 0.04 - row * (box_h + box_gap) - box_h
        ax_box = fig.add_axes([bx, by, col_w * 0.7, box_h])
        tb = TextBox(ax_box, lbl + " ", initial=dflt,
                     color="#313244", hovercolor="#3d3d5c",
                     label_pad=0.05)
        tb.label.set_color("#cdd6f4")
        tb.label.set_fontsize(8)
        tb.text_disp.set_color("#cba6f7")
        tb.text_disp.set_fontsize(9)
        text_boxes[lbl] = tb

    # Send PID button
    btn_ax = fig.add_axes([rp.x0 + rp.width * 0.25, rp.y0 + 0.005,
                           rp.width * 0.5, 0.032])
    btn_send = Button(btn_ax, "Send PID", color="#313244", hovercolor="#45475a")
    btn_send.label.set_color("#a6e3a1")
    btn_send.label.set_fontsize(10)
    btn_send.label.set_fontweight("bold")

    # Status text below button
    status_ax = fig.add_axes([rp.x0, rp.y0 - 0.005, rp.width, 0.025])
    status_ax.axis("off")
    pid_send_status = status_ax.text(
        0.5, 0.5, "", ha="center", va="center",
        fontsize=8, fontfamily="monospace",
        color="#a6e3a1", transform=status_ax.transAxes,
    )

    # ── Callbacks ────────────────────────────────────────────────────

    def on_gain_changed(val):
        g = round(gain_slider.val, 3)
        send_queue.put(f"G:{g}\n")
        gain_status_text.set_text(
            f"Effective resistance = 1.0 × {g} = {g}"
        )
        fig.canvas.draw_idle()

    gain_slider.on_changed(on_gain_changed)

    def on_send_pid(_event):
        try:
            vals = [float(text_boxes[k].text) for k in labels]
        except ValueError:
            pid_send_status.set_text("✗ Invalid value — use numbers only")
            pid_send_status.set_color("#f38ba8")
            fig.canvas.draw_idle()
            return

        cmd = "PID:{},{},{},{},{},{}\n".format(*vals)
        send_queue.put(cmd)
        pid_send_status.set_text(
            f"✓ Sent  Q: P={vals[0]} I={vals[1]} D={vals[2]}"
            f"   D: P={vals[3]} I={vals[4]} D={vals[5]}"
        )
        pid_send_status.set_color("#a6e3a1")
        with lock:
            send_status["msg"] = cmd.strip()
            send_status["ts"]  = time.perf_counter()
        fig.canvas.draw_idle()

    btn_send.on_clicked(on_send_pid)

    return (fig, pid_text, stats_text,
            (ax_phases, ax_iq),
            (line_ia, line_ib, line_ic, line_iq_cmd, line_iq_meas),
            gain_slider, pid_send_status, gain_status_text, text_boxes, btn_send)


def make_animation(fig, pid_text, stats_text, axes, lines,
                   gain_slider, pid_send_status, gain_status_text,
                   text_boxes, btn_send):
    ax_phases, ax_iq = axes
    line_ia, line_ib, line_ic, line_iq_cmd, line_iq_meas = lines

    def update(_frame):
        with lock:
            ts   = list(timestamps)
            ia   = list(ia_buf)
            ib   = list(ib_buf)
            ic   = list(ic_buf)
            iqc  = list(iq_cmd_buf)
            iqm  = list(iq_meas_buf)
            gv   = current_gain["val"]

        pid_text.set_text(_pid_label())

        if len(ts) < 2:
            return

        t_now  = ts[-1]
        t_trim = t_now - WINDOW_SECONDS

        def window(t, y):
            pairs = [(ti, yi) for ti, yi in zip(t, y) if ti >= t_trim]
            if not pairs: return [], []
            t_, y_ = zip(*pairs)
            return list(t_), list(y_)

        ts_w, ia_w  = window(ts, ia)
        _,    ib_w  = window(ts, ib)
        _,    ic_w  = window(ts, ic)
        _,    iqc_w = window(ts, iqc)
        _,    iqm_w = window(ts, iqm)

        if not ts_w:
            return

        t_plot = [t - ts_w[-1] for t in ts_w]

        # Phase currents
        line_ia.set_data(t_plot, ia_w)
        line_ib.set_data(t_plot, ib_w)
        line_ic.set_data(t_plot, ic_w)
        ax_phases.set_xlim(-WINDOW_SECONDS, 0)
        all_i = ia_w + ib_w + ic_w
        if all_i:
            pad = max(0.05, (max(all_i) - min(all_i)) * 0.15)
            ax_phases.set_ylim(min(all_i) - pad, max(all_i) + pad)

        # Iq tracking
        line_iq_cmd.set_data(t_plot, iqc_w)
        line_iq_meas.set_data(t_plot, iqm_w)
        ax_iq.set_xlim(-WINDOW_SECONDS, 0)
        valid_iqm = [v for v in iqm_w if v == v]  # filter NaN
        all_iq = iqc_w + valid_iqm
        if all_iq:
            pad = max(0.05, (max(all_iq) - min(all_iq)) * 0.15)
            ax_iq.set_ylim(min(all_iq) - pad, max(all_iq) + pad)

        # Stats
        n = len(ts_w)
        hz = (n - 1) / (ts_w[-1] - ts_w[0]) if n >= 2 and ts_w[-1] != ts_w[0] else 0
        stats_text.set_text(
            f"Samples: {n}   |   Rate: {hz:.1f} Hz   |   Gain: {gv:.2f}   |   "
            f"Ia: {ia[-1]:+.3f}  Ib: {ib[-1]:+.3f}  Ic: {ic[-1]:+.3f} A   |   "
            f"IqCmd: {iqc[-1]:+.3f}  IqMeas: {iqm[-1]:+.3f} A"
        )

        # Fade out the "Sent" status after 3 seconds
        if time.perf_counter() - send_status.get("ts", 0) > 3.0:
            pid_send_status.set_text("")

    ani = FuncAnimation(fig, update, interval=ANIMATION_MS,
                        blit=False, cache_frame_data=False)
    return ani


# ─────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Haptic Knob Monitor")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", default=115200, type=int)
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--pid", nargs=6, type=float,
                        metavar=("Q_P","Q_I","Q_D","D_P","D_I","D_D"),
                        default=[2.0, 200.0, 0.0, 2.0, 200.0, 0.0])
    args = parser.parse_args()

    stop_event = threading.Event()
    is_demo = args.demo or args.port is None

    if is_demo:
        if args.port is None:
            print("[info] No --port given, running in demo mode.")
        t = threading.Thread(target=demo_generator, args=(stop_event,), daemon=True)
    else:
        print(f"[info] Connecting to {args.port} | startup PID: {args.pid}")
        t = threading.Thread(target=serial_reader,
                             args=(args.port, args.baud, stop_event, args.pid),
                             daemon=True)
    t.start()

    gui = build_gui(args.pid, is_demo)
    (fig, pid_text, stats_text, axes, lines,
     gain_slider, pid_send_status, gain_status_text, text_boxes, btn_send) = gui

    ani = make_animation(fig, pid_text, stats_text, axes, lines,
                         gain_slider, pid_send_status, gain_status_text,
                         text_boxes, btn_send)  # noqa: F841

    try:
        plt.show()
    finally:
        stop_event.set()
        t.join(timeout=2)
        print("[info] Exited.")


if __name__ == "__main__":
    main()