#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import tkinter as tk
from tkinter import filedialog, messagebox
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import UInt8MultiArray, UInt8, Empty
from omnirevolve_ros2_messages.msg import PlotterTelemetry

# ------------------- Constants / Topics / QoS -------------------

PKT_SIZE = 512

TOPIC_STREAM          = '/plotter/byte_stream'           # publisher -> ESP32
TOPIC_NEED            = '/plotter/cmd/need_packets'      # ESP32 -> publisher
TOPIC_PUB_FINISHED    = '/plotter/pub/draw_finished'     # publisher -> manager

TOPIC_DRAW_START      = '/plotter/cmd/draw_start'        # manager -> ESP32
TOPIC_DRAW_FINISH     = '/plotter/cmd/draw_finish'       # manager -> ESP32

TOPIC_CALIBRATE       = '/plotter/cmd/calibrate'
TOPIC_HOME            = '/plotter/cmd/home'

TOPIC_TELEMETRY       = '/plotter/telemetry'             # ESP32 -> UI

QOS_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

QOS_FINISHED_SUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

QOS_TELEMETRY = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# ------------------- Node state -------------------

@dataclass
class StreamState:
    file_path: str = ''
    data: bytes = b''
    offset: int = 0

# ------------------- ROS2 Node -------------------

class PlotterControlNode(Node):
    def __init__(self):
        super().__init__('omnirevolve_ros2_ui')

        # publishers
        self.pub_bytes       = self.create_publisher(UInt8MultiArray, TOPIC_STREAM, QOS_RELIABLE)
        self.pub_draw_start  = self.create_publisher(Empty, TOPIC_DRAW_START, QOS_RELIABLE)
        self.pub_draw_finish = self.create_publisher(Empty, TOPIC_DRAW_FINISH, QOS_RELIABLE)
        self.pub_calibrate   = self.create_publisher(Empty, TOPIC_CALIBRATE, QOS_RELIABLE)
        self.pub_home        = self.create_publisher(Empty, TOPIC_HOME, QOS_RELIABLE)

        # subscribers
        self.sub_need        = self.create_subscription(UInt8, TOPIC_NEED, self.on_need_packets, QOS_RELIABLE)
        self.sub_finished    = self.create_subscription(Empty, TOPIC_PUB_FINISHED, self.on_pub_draw_finished, QOS_FINISHED_SUB)
        self.sub_telem       = self.create_subscription(PlotterTelemetry, TOPIC_TELEMETRY, self.on_telemetry, QOS_TELEMETRY)

        # state
        self.stream = StreamState()
        self.is_drawing = False

        # telemetry state
        self.telem_x = None
        self.telem_y = None
        self.telem_calibrated = None
        self.telem_homed = None

        self.get_logger().info('omnirevolve_ros2_ui node ready.')

    # ---- telemetry ----
    def on_telemetry(self, msg: PlotterTelemetry):
        try:
            self.telem_x = int(msg.x_steps)
            self.telem_y = int(msg.y_steps)
            self.telem_calibrated = 1 if bool(msg.is_calibrated) else 0
            self.telem_homed = 1 if bool(msg.is_homed) else 0
        except Exception as e:
            self.get_logger().error(f'bad telemetry: {e}')

    # ---- publisher logic ----
    def on_need_packets(self, msg: UInt8):
        n = int(msg.data)
        if n <= 0:
            self.get_logger().warn(f'ignore need_packets {n} (must be > 0)')
            return
        for _ in range(n):
            if self.stream.offset >= len(self.stream.data):
                break
            self._send_one_packet()

        if self.stream.offset >= len(self.stream.data) and self.is_drawing:
            self._publish_pub_draw_finished()

    def _send_one_packet(self):
        start = self.stream.offset
        end = min(start + PKT_SIZE, len(self.stream.data))
        payload = self.stream.data[start:end]
        self.stream.offset = end
        if not payload:
            return
        msg = UInt8MultiArray()
        msg.data = list(payload)
        self.pub_bytes.publish(msg)
        self.get_logger().info(f'Sent {len(payload)}B  offset={self.stream.offset}/{len(self.stream.data)}')

    # ---- manager logic ----
    def _publish_pub_draw_finished(self):
        self.get_logger().info('DRAW_FINISHED (publisher side) → notifying manager')
        self._on_pub_finished_local()

    def on_pub_draw_finished(self, _msg: Empty):
        self.get_logger().info('DRAW_FINISHED received from /pub → forwarding draw_finish')
        self._on_pub_finished_local()

    def _on_pub_finished_local(self):
        self.pub_draw_finish.publish(Empty())
        self.get_logger().info('DRAW_FINISH published to ESP32')
        self.is_drawing = False

    # ---- commands from UI ----
    def cmd_load_file(self, path: str) -> None:
        abs_path = os.path.expanduser(path)
        with open(abs_path, 'rb') as f:
            data = f.read()
        self.stream = StreamState(file_path=abs_path, data=data, offset=0)
        self.get_logger().info(f'Loaded: {abs_path} ({len(data)} bytes)')

    def cmd_start_drawing(self) -> None:
        if not self.stream.data:
            raise RuntimeError('No file loaded')
        self.is_drawing = True
        self.stream.offset = 0
        self.pub_draw_start.publish(Empty())
        self.get_logger().info('DRAW_START published')

    def cmd_calibrate(self) -> None:
        self.pub_calibrate.publish(Empty())
        self.get_logger().info('CALIBRATE published')

    def cmd_home(self) -> None:
        self.pub_home.publish(Empty())
        self.get_logger().info('HOME published')

    def cmd_restart(self) -> None:
        if self.is_drawing:
            self.pub_draw_finish.publish(Empty())
            self.get_logger().info('DRAW_FINISH published (restart)')
            self.is_drawing = False
        self.stream.offset = 0
        self.get_logger().info('Publisher offset reset to 0')

# ------------------- Tkinter UI -------------------

class UI(tk.Tk):
    def __init__(self, node: PlotterControlNode):
        super().__init__()
        self.node = node
        self.title('OmniRevolve Plotter Control')

        # Разрешаем ресайз
        self.resizable(True, True)

        # Widgets state vars
        self.file_var     = tk.StringVar(value='No file loaded')
        self.size_var     = tk.StringVar(value='Size: 0 B')
        self.offset_var   = tk.StringVar(value='Offset: 0')
        self.progress_var = tk.StringVar(value='Progress: 0%')
        self.state_var    = tk.StringVar(value='State: READY')

        self.x_var   = tk.StringVar(value='X: —')
        self.y_var   = tk.StringVar(value='Y: —')
        self.cal_var = tk.StringVar(value='Calibrated: —')
        self.home_var= tk.StringVar(value='Homed: —')

        # ---- Разметка: 4 «контент»-колонки (0..3) + 5-я «пустая» (4) тянется
        for c in range(4):
            self.grid_columnconfigure(c, weight=0)
        self.grid_columnconfigure(4, weight=1)  # вся лишняя ширина уходит сюда

        row = 0
        tk.Label(self, text='Selected stream file:', anchor='w') \
            .grid(row=row, column=0, sticky='w', padx=10, pady=5, columnspan=4)
        row += 1
        tk.Label(self, textvariable=self.file_var, fg='blue', anchor='w',
                 wraplength=560, justify='left') \
            .grid(row=row, column=0, sticky='w', padx=10, pady=2, columnspan=4)

        row += 1
        tk.Label(self, textvariable=self.size_var, anchor='w') \
            .grid(row=row, column=0, sticky='w', padx=10)
        tk.Label(self, textvariable=self.offset_var, anchor='w') \
            .grid(row=row, column=1, sticky='w', padx=10)
        tk.Label(self, textvariable=self.progress_var, anchor='w') \
            .grid(row=row, column=2, sticky='w', padx=10)
        tk.Label(self, textvariable=self.state_var, anchor='w') \
            .grid(row=row, column=3, sticky='w', padx=10)

        # Telemetry
        row += 1
        tk.Label(self, text='Telemetry (steps):', anchor='w') \
            .grid(row=row, column=0, sticky='w', padx=10, pady=(10, 2), columnspan=4)

        row += 1
        tk.Label(self, textvariable=self.x_var, anchor='w') \
            .grid(row=row, column=0, sticky='w', padx=10)
        tk.Label(self, textvariable=self.y_var, anchor='w') \
            .grid(row=row, column=1, sticky='w', padx=10)
        tk.Label(self, textvariable=self.cal_var, anchor='w') \
            .grid(row=row, column=2, sticky='w', padx=10)
        tk.Label(self, textvariable=self.home_var, anchor='w') \
            .grid(row=row, column=3, sticky='w', padx=10)

        # Controls
        row += 1
        tk.Button(self, text='Load file…', width=16, command=self.on_load) \
            .grid(row=row, column=0, padx=10, pady=12, sticky='w')
        tk.Button(self, text='Start',      width=16, command=self.on_start) \
            .grid(row=row, column=1, padx=10, pady=12, sticky='w')
        tk.Button(self, text='Restart',    width=16, command=self.on_restart) \
            .grid(row=row, column=2, padx=10, pady=12, sticky='w')
        tk.Button(self, text='Calibrate',  width=16, command=self.on_calibrate) \
            .grid(row=row, column=3, padx=10, pady=12, sticky='w')

        row += 1
        tk.Button(self, text='Home', width=16, command=self.on_home) \
            .grid(row=row, column=0, padx=10, pady=5, sticky='w')

        # Нижняя «растяжимая» строка, чтобы лишняя высота уходила вниз
        row += 1
        self.grid_rowconfigure(row, weight=1)

        # Инициал: подогнать окно под содержимое и задать минимальный размер
        self.update_idletasks()
        self.minsize(self.winfo_reqwidth(), self.winfo_reqheight())

        # periodic updates
        self.after(50, self._pump_ros_once)
        self.after(100, self._refresh_status)

        # clean shutdown on window close
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---- Button handlers ----
    def on_load(self):
        path = filedialog.askopenfilename(title='Choose stream binary',
                                          filetypes=[('All files', '*.*')])
        if not path:
            return
        try:
            self.node.cmd_load_file(path)
            self.file_var.set(self.node.stream.file_path)
            self.size_var.set(f'Size: {len(self.node.stream.data)} B')
            self.offset_var.set('Offset: 0')
            self.progress_var.set('Progress: 0%')
            self.state_var.set('State: READY')
        except Exception as e:
            messagebox.showerror('Load error', str(e))

    def on_start(self):
        try:
            self.node.cmd_start_drawing()
            self.state_var.set('State: DRAWING')
        except Exception as e:
            messagebox.showerror('Start error', str(e))

    def on_restart(self):
        self.node.cmd_restart()
        self.offset_var.set('Offset: 0')
        self.progress_var.set('Progress: 0%')
        self.state_var.set('State: READY')

    def on_calibrate(self):
        self.node.cmd_calibrate()

    def on_home(self):
        self.node.cmd_home()

    # ---- Periodic ----
    def _pump_ros_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.after(20, self._pump_ros_once)

    def _refresh_status(self):
        # stream progress
        data_len = len(self.node.stream.data)
        off = self.node.stream.offset
        self.offset_var.set(f'Offset: {off}')
        pct = int((off / data_len) * 100) if data_len > 0 else 0
        self.progress_var.set(f'Progress: {pct}%')
        if not self.node.is_drawing and data_len > 0 and off == data_len:
            self.state_var.set('State: READY')

        # telemetry
        x = self.node.telem_x
        y = self.node.telem_y
        cal = self.node.telem_calibrated
        home = self.node.telem_homed

        self.x_var.set(f'X: {x}' if x is not None else 'X: —')
        self.y_var.set(f'Y: {y}' if y is not None else 'Y: —')
        self.cal_var.set('Calibrated: YES' if cal == 1 else ('Calibrated: NO' if cal == 0 else 'Calibrated: —'))
        self.home_var.set('Homed: YES' if home == 1 else ('Homed: NO' if home == 0 else 'Homed: —'))

        self.after(200, self._refresh_status)

    def on_close(self):
        try:
            self.destroy()
        except Exception:
            pass

# ------------------- main -------------------

def main():
    rclpy.init()
    node = PlotterControlNode()
    try:
        ui = UI(node)
        ui.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
