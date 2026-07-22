import threading
import serial
import numpy as np
import plotly.graph_objects as go
from collections import deque
from dash import Dash, dcc, html, Input, Output, State
from plotly.subplots import make_subplots

# --- Configuration & State ---
serial_port = None
serial_running = False
buffer_lock = threading.Lock()

# Buffer stores 12 values
history = deque([[0.0] * 12 for _ in range(500)], maxlen=500)
log_buffer = deque(["Waiting for data..."] * 30, maxlen=30)

# FIXED: State uses a 3x3 Rotation Matrix (Identity)
obj_state = {"pos": [0.0, 0.0, 0.0], "rot_mat": np.eye(3)}


def _rotation_matrix(rx: float, ry: float, rz: float) -> np.ndarray:
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)

    rx_mat = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry_mat = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz_mat = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rz_mat @ ry_mat @ rx_mat


def serial_loop():
    global serial_running
    while serial_running:
        if serial_port and serial_port.is_open:
            try:
                line = serial_port.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                parts = [float(p) for p in line.split(",")]
                if len(parts) >= 12:
                    with buffer_lock:
                        history.append(parts)
                        log_buffer.append(line)

                        # 1. Update Position
                        for i in range(3):
                            obj_state["pos"][i] += parts[i] * 0.01

                        # 2. FIXED: Create Delta Rotation Matrix (scaled by small factor)
                        delta_rot = _rotation_matrix(parts[3] * 0.05, parts[4] * 0.05, parts[5] * 0.05)

                        # 3. FIXED: Global/Base-relative update (Delta @ Current)
                        obj_state["rot_mat"] = delta_rot @ obj_state["rot_mat"]

                        # Orthogonalize matrix to prevent drift
                        u, s, vh = np.linalg.svd(obj_state["rot_mat"])
                        obj_state["rot_mat"] = u @ vh
            except:
                continue


app = Dash(__name__)

app.layout = html.Div(
    [
        html.H3("Spacemouse 6DOF Telemetry"),
        html.Div(
            [
                dcc.Input(id="port", value="/dev/cu.usbmodem101"),
                html.Button("Connect", id="connect_btn"),
                html.Div(id="status", children="Disconnected"),
            ]
        ),
        dcc.Graph(id="main_graph", style={"height": "900px"}),
        html.Pre(id="log_window", style={"height": "200px", "overflow": "auto", "background": "#eee"}),
        dcc.Interval(id="tick", interval=50),
    ]
)


@app.callback(
    [Output("main_graph", "figure"), Output("log_window", "children"), Output("status", "children")],
    [Input("tick", "n_intervals")],
)
def update_ui(n):
    with buffer_lock:
        h_arr = np.array(history)
        rot_mat = obj_state["rot_mat"]
        logs = "\n".join(log_buffer)

    fig = make_subplots(
        rows=5,
        cols=2,
        specs=[
            [{"type": "scene"}, None],
            [{"colspan": 2}, None],
            [{"colspan": 2}, None],
            [{"colspan": 2}, None],
            [{"colspan": 2}, None],
        ],
        subplot_titles=(
            "Pos (XYZ)",
            "Rot (RX, RY, RZ)",
            "Vel (VX, VY, VZ)",
            "AngVel (RVX, RVY, RVZ)",
            "3D Joystick State",
        ),
        vertical_spacing=0.03,
    )

    # 1. Telemetry Plots
    for i in range(4):
        for j in range(3):
            fig.add_trace(go.Scatter(y=h_arr[:, i * 3 + j], name=f"Ch{i*3+j+1}"), row=i + 2, col=1)

    # 2. FIXED: Derived axes from Rotation Matrix columns
    # Column 0 = X-axis, Column 1 = Y-axis, Column 2 = Z-axis
    main_axis = rot_mat[:, 2]  # The "Up" vector (Z)
    secondary_axis = rot_mat[:, 0]  # The "Forward/Side" vector (X)

    fig.add_trace(
        go.Cone(
            x=[0],
            y=[0],
            z=[0],
            u=[main_axis[0]],
            v=[main_axis[1]],
            w=[main_axis[2]],
            sizemode="absolute",
            sizeref=0.5,
            showscale=False,
            colorscale=[[0, "red"], [1, "red"]],
        ),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Cone(
            x=[0],
            y=[0],
            z=[0],
            u=[secondary_axis[0]],
            v=[secondary_axis[1]],
            w=[secondary_axis[2]],
            sizemode="absolute",
            sizeref=0.3,
            showscale=False,
            colorscale=[[0, "blue"], [1, "blue"]],
        ),
        row=1,
        col=1,
    )

    # FIXED: Camera for X=Right, Y=Away, Z=Up
    fig.update_scenes(
        xaxis=dict(title="X"),
        yaxis=dict(title="Y"),
        zaxis=dict(title="Z"),
        camera=dict(
            eye=dict(x=0, y=-2, z=1.0),
            up=dict(x=0, y=0, z=1),
            center=dict(x=0, y=0, z=0),
        ),
        aspectmode="cube",
        row=1,
        col=1,
    )

    fig.update_layout(height=1200, showlegend=False)
    return fig, logs, "Connected" if serial_running else "Disconnected"


@app.callback(
    Output("status", "children", allow_duplicate=True),
    Input("connect_btn", "n_clicks"),
    State("port", "value"),
    prevent_initial_call=True,
)
def connect(n, port):
    global serial_port, serial_running
    try:
        serial_port = serial.Serial(port, 115200, timeout=0.1)
        serial_running = True
        threading.Thread(target=serial_loop, daemon=True).start()
        return "Connected"
    except Exception as e:
        return f"Error: {e}"


if __name__ == "__main__":
    app.run(debug=True)
