from __future__ import annotations

import csv
import re
import threading
from collections import deque
from datetime import datetime, timedelta
from pathlib import Path

import pandas as pd
import plotly.graph_objects as go
import serial
from dash import Dash, Input, Output, State, dcc, html, no_update
from plotly.subplots import make_subplots

ROOT_DIR = Path(__file__).resolve().parents[2]
DATA_DIR = ROOT_DIR / "calibration_data"
DATA_DIR.mkdir(parents=True, exist_ok=True)

CHANNELS = [
    "s1_x",
    "s1_y",
    "s1_z",
    "s2_x",
    "s2_y",
    "s2_z",
    "s3_x",
    "s3_y",
    "s3_z",
]

serial_port = None
serial_thread = None
serial_running = False

buffer_lock = threading.Lock()
sample_buffer = deque(maxlen=200000)
raw_line_buffer = deque(maxlen=20)

recording_active = False
recording_file = None
recording_writer = None
recording_target = ""
recording_samples = 0
recording_target_samples = 1200
recording_last_file = ""
recording_stop_reason = ""


def serial_loop() -> None:
    global serial_running
    global recording_active
    global recording_file
    global recording_writer
    global recording_target
    global recording_samples
    global recording_target_samples
    global recording_last_file
    global recording_stop_reason

    while serial_running:
        try:
            if serial_port is None:
                continue
            line = serial_port.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            with buffer_lock:
                raw_line_buffer.append((datetime.now(), line))

            # Parse strict 9-value format, e.g. "a,b,c; d,e,f; g,h,i".
            parts = [p.strip() for p in re.split(r"[;,]", line)]
            if len(parts) != 9:
                continue

            try:
                row_values = [float(v) for v in parts]
            except ValueError:
                continue
            ts = datetime.now()

            with buffer_lock:
                sample_buffer.append((ts, *row_values))
                if recording_active and recording_writer is not None:
                    recording_writer.writerow([ts.isoformat(), recording_target, *row_values])
                    recording_samples += 1
                    if recording_samples >= recording_target_samples:
                        if recording_file is not None:
                            recording_file.close()
                            if hasattr(recording_file, "name"):
                                recording_last_file = Path(recording_file.name).name
                        recording_file = None
                        recording_writer = None
                        recording_active = False
                        recording_stop_reason = "auto"
        except Exception:
            continue


def sanitize_for_filename(text: str) -> str:
    cleaned = text.strip()
    cleaned = cleaned.replace(" ", "_")
    cleaned = cleaned.replace(",", ".")
    cleaned = cleaned.replace("°", "deg")
    cleaned = re.sub(r"[^a-zA-Z0-9._+-]", "_", cleaned)
    return cleaned[:80] if cleaned else "unlabeled"


app = Dash(__name__)

app.layout = html.Div(
    style={"fontFamily": "sans-serif", "padding": "16px", "maxWidth": "1200px", "margin": "0 auto"},
    children=[
        html.H3("Spacemouse Calibration App (Minimal)"),
        html.Div(
            style={"display": "flex", "gap": "8px", "flexWrap": "wrap", "alignItems": "center"},
            children=[
                dcc.Input(id="port", type="text", value="/dev/tty.usbmodem101", placeholder="Serial port"),
                dcc.Input(id="baud", type="number", value=115200, min=1200, step=1),
                html.Button("Connect", id="connect_btn", n_clicks=0),
                html.Button("Disconnect", id="disconnect_btn", n_clicks=0),
                html.Div(id="connection_status", children="Disconnected"),
            ],
        ),
        html.Hr(),
        html.Div(
            style={"display": "flex", "gap": "8px", "flexWrap": "wrap", "alignItems": "center"},
            children=[
                dcc.Dropdown(
                    id="recording_mode",
                    options=[
                        {"label": "DoF target", "value": "dof"},
                        {"label": "No magnets", "value": "no_magnets"},
                        {"label": "Neutral position", "value": "neutral_position"},
                    ],
                    value="dof",
                    clearable=False,
                    style={"width": "180px"},
                ),
                dcc.Dropdown(
                    id="installed_magnets",
                    options=[
                        {"label": "All magnets", "value": "ALL"},
                        {"label": "Magnet 1 only", "value": "M1"},
                        {"label": "Magnet 2 only", "value": "M2"},
                        {"label": "Magnet 3 only", "value": "M3"},
                    ],
                    value="ALL",
                    clearable=False,
                    style={"width": "170px"},
                ),
                dcc.Dropdown(
                    id="dof",
                    options=[{"label": v, "value": v} for v in ["X", "Y", "Z", "RX", "RY", "RZ"]],
                    value="X",
                    clearable=False,
                    style={"width": "120px"},
                ),
                dcc.Input(id="magnitude", type="number", value=1.5, step=0.1),
                dcc.Dropdown(
                    id="unit",
                    options=[{"label": "mm", "value": "mm"}, {"label": "deg", "value": "deg"}],
                    value="mm",
                    clearable=False,
                    disabled=True,
                    style={"width": "100px"},
                ),
                html.Label("Samples:"),
                dcc.Input(id="target_samples", type="number", value=1200, min=1, step=1),
                html.Button("Start Recording", id="start_rec_btn", n_clicks=0),
                html.Button("Stop Recording", id="stop_rec_btn", n_clicks=0),
                html.Div(id="recording_status", children="Not recording"),
            ],
        ),
        html.Hr(),
        html.Div(
            style={"display": "flex", "gap": "8px", "flexWrap": "wrap", "alignItems": "center"},
            children=[
                html.Label("History window (minutes):"),
                dcc.Input(id="history_minutes", type="number", value=2, min=0.2, step=0.2),
                html.Div(id="live_stats"),
            ],
        ),
        dcc.Graph(id="history_plot", style={"height": "600px"}),
        html.H4("Last raw serial lines"),
        html.Pre(
            id="serial_lines",
            style={
                "height": "220px",
                "overflowY": "auto",
                "background": "#f7f7f7",
                "padding": "10px",
                "border": "1px solid #ddd",
                "fontSize": "12px",
                "lineHeight": "1.3",
                "whiteSpace": "pre-wrap",
                "marginTop": "8px",
            },
            children="waiting for serial data...",
        ),
        dcc.Interval(id="tick", interval=1000, n_intervals=0),
    ],
)


@app.callback(
    Output("connection_status", "children"),
    Input("connect_btn", "n_clicks"),
    Input("disconnect_btn", "n_clicks"),
    State("port", "value"),
    State("baud", "value"),
    prevent_initial_call=True,
)
def handle_connection(_connect_clicks: int, _disconnect_clicks: int, port: str, baud: int) -> str:
    global serial_port
    global serial_thread
    global serial_running

    trigger = Input
    try:
        from dash import ctx

        trigger = ctx.triggered_id
    except Exception:
        trigger = None

    if trigger == "connect_btn":
        if serial_port is not None and serial_port.is_open:
            return f"Already connected: {serial_port.port}"
        try:
            serial_port = serial.Serial(port=port, baudrate=int(baud), timeout=0.2)
            serial_running = True
            serial_thread = threading.Thread(target=serial_loop, daemon=True)
            serial_thread.start()
            return f"Connected: {port} @ {baud}"
        except Exception as exc:
            serial_port = None
            serial_running = False
            return f"Connect failed: {exc}"

    if trigger == "disconnect_btn":
        try:
            serial_running = False
            if serial_port is not None and serial_port.is_open:
                serial_port.close()
            serial_port = None
            return "Disconnected"
        except Exception as exc:
            return f"Disconnect failed: {exc}"

    return no_update


@app.callback(
    Output("unit", "value"),
    Output("unit", "disabled"),
    Input("dof", "value"),
)
def update_unit_from_dof(dof: str) -> tuple[str, bool]:
    if dof in ["RX", "RY", "RZ"]:
        return "deg", True
    return "mm", True


@app.callback(
    Output("recording_status", "children"),
    Input("start_rec_btn", "n_clicks"),
    Input("stop_rec_btn", "n_clicks"),
    State("recording_mode", "value"),
    State("installed_magnets", "value"),
    State("dof", "value"),
    State("magnitude", "value"),
    State("target_samples", "value"),
    prevent_initial_call=True,
)
def handle_recording(
    _start_clicks: int,
    _stop_clicks: int,
    recording_mode: str,
    installed_magnets: str,
    dof: str,
    magnitude: float,
    target_samples: int,
) -> str:
    global recording_active
    global recording_file
    global recording_writer
    global recording_target
    global recording_samples
    global recording_target_samples
    global recording_last_file
    global recording_stop_reason

    trigger = None
    try:
        from dash import ctx

        trigger = ctx.triggered_id
    except Exception:
        trigger = None

    if trigger == "start_rec_btn":
        with buffer_lock:
            if recording_active:
                return f"Already recording: {recording_target}"

            if recording_mode == "no_magnets":
                recording_target = f"{installed_magnets}_NO_MAGNETS"
            elif recording_mode == "neutral_position":
                recording_target = f"{installed_magnets}_NEUTRAL_POSITION"
            else:
                auto_unit = "deg" if dof in ["RX", "RY", "RZ"] else "mm"
                recording_target = f"{installed_magnets}_{dof}{magnitude}{auto_unit}"
            safe_target = sanitize_for_filename(recording_target)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            out_file = DATA_DIR / f"{stamp}_{safe_target}.csv"
            recording_target_samples = max(1, int(target_samples or 1))

            try:
                recording_file = open(out_file, "w", newline="", encoding="utf-8")
                recording_writer = csv.writer(recording_file)
                recording_writer.writerow(["timestamp_iso", "target_label", *CHANNELS])
                recording_samples = 0
                recording_last_file = out_file.name
                recording_stop_reason = ""
                recording_active = True
                return f"Recording: {out_file.name} (target={recording_target_samples})"
            except Exception as exc:
                recording_file = None
                recording_writer = None
                recording_active = False
                return f"Start failed: {exc}"

    if trigger == "stop_rec_btn":
        with buffer_lock:
            if not recording_active:
                return "Not recording"
            try:
                recording_active = False
                file_name = recording_last_file or "unknown"
                if recording_file is not None:
                    if hasattr(recording_file, "name"):
                        file_name = Path(recording_file.name).name
                    recording_file.close()
                recording_file = None
                recording_writer = None
                recording_last_file = file_name
                recording_stop_reason = "manual"
                return f"Saved {recording_samples} samples to {file_name}"
            except Exception as exc:
                return f"Stop failed: {exc}"

    return no_update


@app.callback(
    Output("history_plot", "figure"),
    Output("live_stats", "children"),
    Output("serial_lines", "children"),
    Input("tick", "n_intervals"),
    State("history_minutes", "value"),
)
def update_plot(_n: int, history_minutes: float) -> tuple[go.Figure, str, str]:
    history_minutes = float(history_minutes or 2)

    with buffer_lock:
        rows = list(sample_buffer)
        raw_rows = list(raw_line_buffer)
        active = recording_active
        recorded = recording_samples
        target = recording_target_samples
        stop_reason = recording_stop_reason
        last_file = recording_last_file

    raw_text = "\n".join([f"{ts.strftime('%H:%M:%S.%f')[:-3]} | {txt}" for ts, txt in raw_rows])
    if not raw_text:
        raw_text = "no raw lines yet"

    fig = make_subplots(
        rows=3,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.06,
        subplot_titles=("Sensor 1", "Sensor 2", "Sensor 3"),
    )
    if not rows:
        fig.update_layout(template="plotly_white", title="Waiting for data...")
        return fig, "buffer=0", raw_text

    df = pd.DataFrame(rows, columns=["timestamp", *CHANNELS])
    window_start = datetime.now() - timedelta(minutes=history_minutes)
    df = df[df["timestamp"] >= window_start]

    for ch in ["s1_x", "s1_y", "s1_z"]:
        fig.add_trace(go.Scatter(x=df["timestamp"], y=df[ch], mode="lines", name=ch), row=1, col=1)

    for ch in ["s2_x", "s2_y", "s2_z"]:
        fig.add_trace(go.Scatter(x=df["timestamp"], y=df[ch], mode="lines", name=ch), row=2, col=1)

    for ch in ["s3_x", "s3_y", "s3_z"]:
        fig.add_trace(go.Scatter(x=df["timestamp"], y=df[ch], mode="lines", name=ch), row=3, col=1)

    fig.update_layout(
        template="plotly_white",
        title=f"Last {history_minutes:.1f} min",
        xaxis3_title="Time",
        margin={"l": 40, "r": 20, "t": 50, "b": 40},
        legend={"orientation": "h", "y": 1.08},
        height=780,
    )
    fig.update_yaxes(title_text="Value", row=1, col=1)
    fig.update_yaxes(title_text="Value", row=2, col=1)
    fig.update_yaxes(title_text="Value", row=3, col=1)

    auto_msg = ""
    if (not active) and stop_reason == "auto" and last_file:
        auto_msg = f" | auto-saved={last_file}"
    stats = (
        f"buffer={len(rows)} | visible={len(df)} | recording={active} | recorded_samples={recorded}/{target}{auto_msg}"
    )
    return fig, stats, raw_text


if __name__ == "__main__":
    app.run(debug=True)
