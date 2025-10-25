#!/usr/bin/env python3
"""
Real-time plotter for inverted pendulum on cart system
Receives data via UART and plots 5 graphs.
Supports sending commands to the microcontroller.

Starts with a graphical setup menu (Tkinter) for configuration.

Message format:
- [STATUS]timestamp;control_mode;cal_x;cal_t;target_vel;motor_acc;pwm_freq;cart_pos;cart_vel;pend_pos;pend_vel;ref_pos;shadow_ref
- [INFO]message
- [RESPONSE]message
"""

import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import queue
import time
import sys
import tkinter as tk
from tkinter import ttk, messagebox
import serial.tools.list_ports
from tkinter import filedialog
import csv
from datetime import datetime

# ==============================================================================
# CLASSE DE CONFIGURAÇÃO (MENU GRÁFICO - TKINTER)
# ==============================================================================


class SetupMenu:
    """Cria uma janela Tkinter para selecionar as configurações da porta serial."""

    def __init__(self, master):
        self.master = master
        master.title("Configurações do Pendulo Invertido")

        # Variáveis de controle para armazenar as seleções
        self.port_var = tk.StringVar(master)
        self.baudrate_var = tk.StringVar(master, value="115200")
        self.max_points_var = tk.StringVar(master, value="1000")
        self.settings = None

        # 1. Obter Portas Seriais Disponíveis
        self.available_ports = self._list_serial_ports()
        if not self.available_ports:
            self.available_ports = ["Nenhuma Porta Encontrada"]
        self.port_var.set(self.available_ports[0])

        # Opções de Baudrate e Max Points
        self.baudrates = [
            "9600",
            "19200",
            "38400",
            "57600",
            "115200",
            "230400",
            "460800",
            "921600",
        ]
        self.max_points_options = ["100", "250", "500", "1000", "2000"]

        # 2. Criar e configurar o Grid
        main_frame = ttk.Frame(master, padding="10")
        main_frame.pack(padx=10, pady=10)

        # Configurar grid
        main_frame.columnconfigure(0, weight=1, uniform="a")
        main_frame.columnconfigure(1, weight=3, uniform="a")

        # 3. Componentes da GUI

        # Porta Serial (Dropdown)
        ttk.Label(main_frame, text="Porta Serial:").grid(
            row=0, column=0, sticky="w", pady=5
        )
        self.port_menu = ttk.Combobox(
            main_frame,
            textvariable=self.port_var,
            values=self.available_ports,
            state="readonly",
        )
        self.port_menu.grid(row=0, column=1, sticky="ew", pady=5)

        # Baudrate (Dropdown)
        ttk.Label(main_frame, text="Baudrate:").grid(
            row=1, column=0, sticky="w", pady=5
        )
        self.baudrate_menu = ttk.Combobox(
            main_frame,
            textvariable=self.baudrate_var,
            values=self.baudrates,
            state="readonly",
        )
        self.baudrate_menu.grid(row=1, column=1, sticky="ew", pady=5)

        # Max Points (Dropdown)
        ttk.Label(main_frame, text="Máx. Pontos:").grid(
            row=2, column=0, sticky="w", pady=5
        )
        self.max_points_menu = ttk.Combobox(
            main_frame,
            textvariable=self.max_points_var,
            values=self.max_points_options,
            state="readonly",
        )
        self.max_points_menu.grid(row=2, column=1, sticky="ew", pady=5)

        # Botão Iniciar
        self.start_button = ttk.Button(
            main_frame, text="Iniciar Plotter", command=self._start_plotter
        )
        self.start_button.grid(row=3, column=0, columnspan=2, pady=15, sticky="ew")

        # Centralizar a janela na tela
        master.update_idletasks()
        width = master.winfo_width()
        height = master.winfo_height()
        x = (master.winfo_screenwidth() // 2) - (width // 2)
        y = (master.winfo_screenheight() // 2) - (height // 2)
        master.geometry(f"{width}x{height}+{x}+{y}")

        master.lift()
        master.attributes("-topmost", True)
        master.after_idle(master.attributes, "-topmost", False)

    def _list_serial_ports(self):
        """Lista as portas seriais disponíveis no sistema."""
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    def _start_plotter(self):
        """Coleta as configurações e fecha a janela de setup."""
        port = self.port_var.get()
        baudrate = self.baudrate_var.get()
        max_points = self.max_points_var.get()

        if port == "Nenhuma Porta Encontrada" or not port:
            messagebox.showerror(
                "Erro de Configuração", "Por favor, selecione uma porta serial válida."
            )
            return

        self.settings = {
            "port": port,
            "baudrate": int(baudrate),
            "max_points": int(max_points),
        }
        self.master.quit()
        self.master.destroy()

    def get_settings(self):
        """Executa a janela de setup e retorna as configurações."""
        self.master.mainloop()
        return self.settings


# ==============================================================================
# CLASSE PRINCIPAL DE PLOTAGEM COM CONTROLES
# ==============================================================================


class PendulumDataPlotter:
    def __init__(self, port, baudrate, max_points=500):
        """Initialize the pendulum data plotter"""
        self.port = port
        self.baudrate = baudrate
        self.max_points = max_points

        # Data queues for thread-safe communication
        self.data_queue = queue.Queue()

        # Data storage for plotting
        self.time_data = deque(maxlen=max_points)
        self.real_time = deque(maxlen=max_points)
        self.target_vel = deque(maxlen=max_points)
        self.motor_acc = deque(maxlen=max_points)
        self.pwm_freq = deque(maxlen=max_points)
        self.cart_pos = deque(maxlen=max_points)
        self.cart_vel = deque(maxlen=max_points)
        self.pendulum_pos = deque(maxlen=max_points)
        self.pendulum_vel = deque(maxlen=max_points)

        # Status data
        self.control_mode = 0
        self.cal_x_status = 0
        self.cal_t_status = 0
        self.ref_pos = 0.0
        self.shadow_ref = 0.0

        # Control mode names
        self.control_modes = {
            0: "OFF",
            1: "CART_PID",
            2: "CART_LEAD",
            3: "CART_SF",
            4: "PEND_SF",
            5: "FULL_LQR",
        }

        # Start time reference
        self.start_time = None

        # Serial connection
        self.serial_port = None
        self.serial_thread = None
        self.running = False

        # Initialize plot with subplots for graphs and controls
        self.fig = plt.figure(figsize=(16, 10))

        # Create grid spec for layout: 20% left panel, 80% right graphs
        gs = self.fig.add_gridspec(
            6,
            2,
            width_ratios=[1, 4],
            hspace=0.35,
            wspace=0.25,
            left=0.05,
            right=0.98,
            top=0.96,
            bottom=0.05,
        )

        # Control panel (left side - 20%)
        self.control_ax = self.fig.add_subplot(gs[:, 0])
        self.control_ax.axis("off")

        # Graph axes (right side - 80%)
        self.axes = [
            self.fig.add_subplot(gs[0, 1]),  # Target velocity
            self.fig.add_subplot(gs[1, 1]),  # PWM frequency
            self.fig.add_subplot(gs[2, 1]),  # Cart position
            self.fig.add_subplot(gs[3, 1]),  # Cart velocity
            self.fig.add_subplot(gs[4, 1]),  # Pendulum position
            self.fig.add_subplot(gs[5, 1]),  # Pendulum velocity
        ]

        self.lines = []
        self.animation = None

    def setup_plots(self):
        """Configure the plot appearance and create line objects"""

        titles = [
            "Target Velocity (m/s)",
            "PWM Frequency (Hz)",
            "Cart Position (m)",
            "Cart Velocity (m/s)",
            "Pendulum Position (rad)",
            "Pendulum Velocity (rad/s)",
        ]
        colors = ["purple", "blue", "green", "red", "orange", "brown"]

        for i, (ax, title, color) in enumerate(zip(self.axes, titles, colors)):
            ax.set_title(title, fontsize=10)
            ax.set_xlabel("Time (s)" if i == 4 else "")
            ax.grid(True, alpha=0.3)
            (line,) = ax.plot([], [], color=color, linewidth=1.5)
            self.lines.append(line)
            ax.set_xlim(0, 10)
            ax.set_ylim(-1, 1)

        # Setup control panel (only once)
        self.setup_control_panel()

        plt.tight_layout()

        # Store button positions for click detection
        self.store_button_positions()

    def setup_control_panel(self):
        """Setup control panel with buttons and info"""
        self.control_ax.clear()
        self.control_ax.axis("off")

        # CRITICAL: Set axis limits to ensure coordinates are 0-1
        self.control_ax.set_xlim(0, 1)
        self.control_ax.set_ylim(0, 1)

        # Title
        y_pos = 0.98
        self.control_ax.text(
            0.5,
            y_pos,
            "CONTROL",
            ha="center",
            va="top",
            fontsize=13,
            fontweight="bold",
            transform=self.control_ax.transAxes,
        )

        # Status section
        y_pos -= 0.06
        self.control_ax.text(
            0.5,
            y_pos,
            "━━━━━━━━━━━━━━",
            ha="center",
            va="top",
            fontsize=10,
            transform=self.control_ax.transAxes,
        )

        y_pos -= 0.05
        self.status_text = self.control_ax.text(
            0.5,
            y_pos,
            "",
            ha="center",
            va="top",
            fontsize=8,
            family="monospace",
            transform=self.control_ax.transAxes,
        )

        # Save CSV button (ANTES de "MODES")
        y_pos = 0.68  # Posição acima de MODES
        save_height = 0.05
        save_rect = plt.Rectangle(
            (0.05, y_pos - save_height),
            0.9,
            save_height,
            facecolor="lightskyblue",
            edgecolor="darkblue",
            linewidth=2,
            transform=self.control_ax.transAxes,
            picker=True,
        )
        self.control_ax.add_patch(save_rect)
        self.control_ax.text(
            0.5,
            y_pos - save_height / 2,
            "SAVE CSV",
            ha="center",
            va="center",
            fontsize=9,
            fontweight="bold",
            transform=self.control_ax.transAxes,
        )
        self.save_button = {
            "rect": save_rect,
            "y_min": y_pos - save_height,
            "y_max": y_pos,
        }

        # Mode buttons section (código existente - linha 320)
        y_pos = 0.60
        self.control_ax.text(
            0.5,
            y_pos,
            "MODES",
            ha="center",
            va="top",
            fontsize=10,
            fontweight="bold",
            transform=self.control_ax.transAxes,
        )

        # Mode buttons section
        y_pos = 0.60
        self.control_ax.text(
            0.5,
            y_pos,
            "MODES",
            ha="center",
            va="top",
            fontsize=10,
            fontweight="bold",
            transform=self.control_ax.transAxes,
        )

        y_pos -= 0.03
        button_height = 0.048
        button_spacing = 0.055

        # Gerar lista de modos dinamicamente a partir do dicionário
        modes = [
            (key, self.control_modes[key]) for key in sorted(self.control_modes.keys())
        ]

        self.mode_buttons = {}
        for mode_id, mode_name in modes:
            y_pos -= button_spacing
            rect = plt.Rectangle(
                (0.05, y_pos - button_height),
                0.9,
                button_height,
                facecolor="lightgray",
                edgecolor="black",
                linewidth=1.5,
                transform=self.control_ax.transAxes,
                picker=True,
            )
            self.control_ax.add_patch(rect)
            self.control_ax.text(
                0.5,
                y_pos - button_height / 2,
                mode_name,
                ha="center",
                va="center",
                fontsize=7,
                transform=self.control_ax.transAxes,
            )
            self.mode_buttons[mode_id] = {
                "rect": rect,
                "y_min": y_pos - button_height,
                "y_max": y_pos,
            }

        # Reference control section with slider
        y_pos -= 0.06
        self.control_ax.text(
            0.5,
            y_pos,
            "REFERENCE",
            ha="center",
            va="top",
            fontsize=10,
            fontweight="bold",
            transform=self.control_ax.transAxes,
        )

        # Slider background
        y_pos -= 0.04
        slider_height = 0.025
        slider_left = 0.05
        slider_width = 0.9

        # Draw slider track
        slider_track = plt.Rectangle(
            (slider_left, y_pos - slider_height),
            slider_width,
            slider_height,
            facecolor="lightgray",
            edgecolor="black",
            linewidth=1,
            transform=self.control_ax.transAxes,
            picker=True,
        )
        self.control_ax.add_patch(slider_track)

        # Draw tick marks for reference values
        num_ticks = 17  # -0.2 to 0.2 with 0.025 steps = 17 positions
        for i in range(num_ticks):
            tick_x = slider_left + (i / (num_ticks - 1)) * slider_width
            tick_y = y_pos - slider_height
            self.control_ax.plot(
                [tick_x, tick_x],
                [tick_y - 0.01, tick_y],
                color="black",
                linewidth=0.5,
                transform=self.control_ax.transAxes,
            )

        # Label min and max
        self.control_ax.text(
            slider_left,
            y_pos - slider_height - 0.02,
            "-0.2",
            ha="left",
            va="top",
            fontsize=6,
            transform=self.control_ax.transAxes,
        )
        self.control_ax.text(
            slider_left + slider_width,
            y_pos - slider_height - 0.02,
            "0.2",
            ha="right",
            va="top",
            fontsize=6,
            transform=self.control_ax.transAxes,
        )

        # Slider handle (will be updated in update_status_display)
        self.slider_handle = plt.Circle(
            (0.5, y_pos - slider_height / 2),
            0.015,
            facecolor="blue",
            edgecolor="darkblue",
            linewidth=2,
            transform=self.control_ax.transAxes,
            zorder=10,
        )
        self.control_ax.add_patch(self.slider_handle)

        # Store slider parameters
        self.slider_params = {
            "y_pos": y_pos,
            "height": slider_height,
            "left": slider_left,
            "width": slider_width,
            "y_min": y_pos - slider_height,
            "y_max": y_pos,
        }

        # Apply button
        y_pos -= 0.06
        apply_height = 0.05
        apply_rect = plt.Rectangle(
            (0.05, y_pos - apply_height),
            0.9,
            apply_height,
            facecolor="lightgreen",
            edgecolor="darkgreen",
            linewidth=2,
            transform=self.control_ax.transAxes,
            picker=True,
        )
        self.control_ax.add_patch(apply_rect)
        self.control_ax.text(
            0.5,
            y_pos - apply_height / 2,
            "✓ APPLY REFERENCE",
            ha="center",
            va="center",
            fontsize=9,
            fontweight="bold",
            transform=self.control_ax.transAxes,
        )
        self.apply_button = {
            "rect": apply_rect,
            "y_min": y_pos - apply_height,
            "y_max": y_pos,
        }

        # Reference value display
        y_pos -= 0.06
        self.ref_value_text = self.control_ax.text(
            0.5,
            y_pos,
            "",
            ha="center",
            va="top",
            fontsize=9,
            fontweight="bold",
            transform=self.control_ax.transAxes,
        )

    def store_button_positions(self):
        """Store button positions after setup for click detection"""
        # This is called after setup_control_panel to ensure positions are stored
        pass

    def on_click(self, event):
        """Handle mouse clicks on control panel"""
        if event.inaxes != self.control_ax:
            return

        # Get click coordinates in axes space (0 to 1)
        x, y = event.xdata, event.ydata
        if x is None or y is None:
            return

        # Check mode buttons
        for mode_id, btn_info in self.mode_buttons.items():
            if 0.05 <= x <= 0.95 and btn_info["y_min"] <= y <= btn_info["y_max"]:
                self.send_command(f"MODE:{mode_id}")
                print(f"Sent: MODE:{mode_id}")
                return

        # Check if Save CSV button was clicked
        if hasattr(self, "save_button"):
            if (
                0.05 <= x <= 0.95
                and self.save_button["y_min"] <= y <= self.save_button["y_max"]
            ):
                print("Save CSV button clicked!")
                self.save_to_csv()
                return

        # Check if click is on slider area
        if hasattr(self, "slider_params"):
            sp = self.slider_params
            if (
                sp["left"] <= x <= sp["left"] + sp["width"]
                and sp["y_min"] <= y <= sp["y_max"]
            ):
                # Calculate reference value from click position
                normalized_x = (x - sp["left"]) / sp["width"]  # 0 to 1
                ref_value = -0.2 + (normalized_x * 0.4)  # -0.2 to 0.2

                # Round to nearest 0.025
                ref_value = round(ref_value / 0.025) * 0.025

                # Clamp to range
                ref_value = max(-0.2, min(0.2, ref_value))

                # Send command
                self.send_command(f"SREF:{ref_value:.3f}")
                print(f"Sent: SREF:{ref_value:.3f}")
                return

        # Check apply button
        if hasattr(self, "apply_button"):
            ab = self.apply_button
            if 0.05 <= x <= 0.95 and ab["y_min"] <= y <= ab["y_max"]:
                self.send_command("AREF")
                print("Sent: AREF")
                return

    def send_command(self, cmd):
        """Send command to microcontroller"""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write((cmd + "\n").encode("utf-8"))
            except Exception as e:
                print(f"Error sending command: {e}")

    def update_status_display(self):
        """Update status text in control panel"""
        cal_x_str = (
            "Ok"
            if self.cal_x_status == 0
            else "Error" if self.cal_x_status == 2 else "Running"
        )
        cal_t_str = (
            "Ok"
            if self.cal_t_status == 0
            else "Error" if self.cal_t_status == 2 else "Running"
        )
        mode_str = self.control_modes.get(self.control_mode, "?")

        status = f"""Mode: {mode_str}
Cal X: {cal_x_str}  Cal T: {cal_t_str}

Active: {self.ref_pos:+.3f}m
"""
        self.status_text.set_text(status)

        # Update slider handle position based on shadow_ref
        if hasattr(self, "slider_handle") and hasattr(self, "slider_params"):
            sp = self.slider_params
            # Normalize shadow_ref from -0.2..0.2 to 0..1
            normalized = (self.shadow_ref + 0.2) / 0.4
            # Convert to slider x position
            handle_x = sp["left"] + (normalized * sp["width"])
            self.slider_handle.center = (handle_x, sp["y_pos"] - sp["height"] / 2)

        # Update reference value display
        if hasattr(self, "ref_value_text"):
            self.ref_value_text.set_text(f"Shadow: {self.shadow_ref:+.3f} m")

    def save_to_csv(self):
        """Save collected data to CSV file"""
        if len(self.time_data) == 0:
            print("No data to save!")
            return

        # Open file dialog
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialfile=f"pendulum_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
        )

        if not filename:
            print("Save cancelled")
            return

        try:
            with open(filename, "w", newline="") as csvfile:
                writer = csv.writer(csvfile)

                # Write header
                writer.writerow(
                    [
                        "Timestamp",
                        "Real_Time",
                        "Target_Vel",
                        "PWM_Freq",
                        "Cart_Pos",
                        "Cart_Vel",
                        "Pendulum_Pos",
                        "Pendulum_Vel",
                    ]
                )

                # Write data rows
                for i in range(len(self.time_data)):
                    writer.writerow(
                        [
                            self.time_data[i],
                            self.real_time[i],
                            self.target_vel[i],
                            self.pwm_freq[i],
                            self.cart_pos[i],
                            self.cart_vel[i],
                            self.pendulum_pos[i],
                            self.pendulum_vel[i],
                        ]
                    )

            print(f"Data saved successfully to: {filename}")
            print(f"Total points saved: {len(self.time_data)}")
        except Exception as e:
            print(f"Error saving file: {e}")

    def connect_serial(self):
        """Establish serial connection"""
        try:
            self.serial_port = serial.Serial(
                port=self.port, baudrate=self.baudrate, timeout=1.0
            )
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            messagebox.showerror(
                "Erro de Serial", f"Erro ao conectar na porta serial {self.port}: {e}"
            )
            print(f"Error connecting to serial port: {e}")
            return False

    def read_serial_data(self):
        """Thread function to read serial data continuously"""
        buffer = ""

        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    chunk = self.serial_port.read(self.serial_port.in_waiting).decode(
                        "utf-8", errors="ignore"
                    )
                    buffer += chunk

                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()

                        if line:
                            self.process_line(line)

                time.sleep(0.01)

            except Exception as e:
                print(f"Serial read error: {e}")
                self.running = False
                break

    def process_line(self, line):
        """Process incoming line based on header"""
        # Check for headers
        if line.startswith("[STATUS]"):
            # Extract data after header
            data_part = line[8:]  # Remove "[STATUS]"
            self.parse_status_data(data_part)
        elif line.startswith("[INFO]"):
            # Print info messages to console
            info_msg = line[6:]  # Remove "[INFO]"
            print(f"INFO: {info_msg}")
        elif line.startswith("[RESPONSE]"):
            # Print response messages to console
            response_msg = line[10:]  # Remove "[RESPONSE]"
            print(f"RESPONSE: {response_msg}")
        else:
            # Unknown format, print to console
            print(f"UNKNOWN: {line}")

    def parse_status_data(self, data_str):
        """
        Parse STATUS data line
        Format: timestamp;control_mode;cal_x;cal_t;target_vel;motor_acc;pwm_freq;cart_pos;cart_vel;pend_pos;pend_vel;ref_pos;shadow_ref
        """
        try:
            parts = data_str.split(";")
            if len(parts) >= 13:
                current_time = time.time()
                if self.start_time is None:
                    self.start_time = current_time

                # Parse all values
                timestamp = int(parts[0])
                control_mode = int(parts[1])
                cal_x_status = int(parts[2])
                cal_t_status = int(parts[3])
                target_vel = self._parse_float_or_nan(parts[4])
                motor_acc = self._parse_float_or_nan(parts[5])  # Not used for plotting
                pwm_freq = int(parts[6])
                cart_pos = self._parse_float_or_nan(parts[7])
                cart_vel = self._parse_float_or_nan(parts[8])
                pendulum_pos = self._parse_float_or_nan(parts[9])
                pendulum_vel = self._parse_float_or_nan(parts[10])
                ref_pos = self._parse_float_or_nan(parts[11])
                shadow_ref = self._parse_float_or_nan(parts[12])

                # Check if all critical values are valid (not NaN)
                critical_values = [
                    target_vel,
                    motor_acc,
                    pwm_freq,
                    cart_pos,
                    cart_vel,
                    pendulum_pos,
                    pendulum_vel,
                    ref_pos,
                    shadow_ref,
                ]
                if all(not np.isnan(v) for v in critical_values):
                    data = {
                        "timestamp": timestamp,
                        "real_time": current_time - self.start_time,
                        "control_mode": control_mode,
                        "cal_x_status": cal_x_status,
                        "cal_t_status": cal_t_status,
                        "target_vel": target_vel,
                        "motor_acc": motor_acc,
                        "pwm_freq": pwm_freq,
                        "cart_pos": cart_pos,
                        "cart_vel": cart_vel,
                        "pendulum_pos": pendulum_pos,
                        "pendulum_vel": pendulum_vel,
                        "ref_pos": ref_pos,
                        "shadow_ref": shadow_ref,
                    }

                    # Update status variables
                    self.control_mode = data["control_mode"]
                    self.cal_x_status = data["cal_x_status"]
                    self.cal_t_status = data["cal_t_status"]
                    self.ref_pos = data["ref_pos"]
                    self.shadow_ref = data["shadow_ref"]

                    # Add to queue for plotting
                    self.data_queue.put(data)
                else:
                    # Data contains NaN, skip plotting but still update status if available
                    self.control_mode = control_mode
                    self.cal_x_status = cal_x_status
                    self.cal_t_status = cal_t_status
                    if not np.isnan(ref_pos):
                        self.ref_pos = ref_pos
                    if not np.isnan(shadow_ref):
                        self.shadow_ref = shadow_ref

        except (ValueError, IndexError) as e:
            # Silently ignore parse errors
            pass

    def _parse_float_or_nan(self, value_str):
        """Parse float value, return NaN if invalid"""
        try:
            value = float(value_str)
            return value
        except (ValueError, TypeError):
            return np.nan

    def update_plot(self, frame):
        """Update plot with new data from queue"""
        data_updated = False
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
                self.time_data.append(data["timestamp"])
                self.real_time.append(data["real_time"])
                self.target_vel.append(data["target_vel"])
                self.motor_acc.append(data["motor_acc"])
                self.pwm_freq.append(data["pwm_freq"])
                self.cart_pos.append(data["cart_pos"])
                self.cart_vel.append(data["cart_vel"])
                self.pendulum_pos.append(data["pendulum_pos"])
                self.pendulum_vel.append(data["pendulum_vel"])
                data_updated = True
            except queue.Empty:
                break

        if data_updated and len(self.real_time) > 1:
            time_array = np.array(self.real_time)

            # Update all plots
            self.lines[0].set_data(time_array, self.target_vel)
            self.lines[1].set_data(time_array, self.pwm_freq)
            self.lines[2].set_data(time_array, self.cart_pos)
            self.lines[3].set_data(time_array, self.cart_vel)
            self.lines[4].set_data(time_array, self.pendulum_pos)
            self.lines[5].set_data(time_array, self.pendulum_vel)

            data_lists = [
                list(self.target_vel),
                list(self.pwm_freq),
                list(self.cart_pos),
                list(self.cart_vel),
                list(self.pendulum_pos),
                list(self.pendulum_vel),
            ]

            for ax, data_list in zip(self.axes, data_lists):
                if len(data_list) > 0:
                    # Update X-axis
                    time_min, time_max = time_array[0], time_array[-1]
                    time_range = time_max - time_min
                    padding_x = time_range * 0.05 if time_range > 0.001 else 1
                    ax.set_xlim(time_min - padding_x, time_max + padding_x)

                    # Update Y-axis
                    data_min, data_max = min(data_list), max(data_list)
                    data_range = data_max - data_min
                    padding_y = data_range * 0.2 if data_range > 1e-6 else 0.5
                    center = (data_min + data_max) / 2

                    if data_range < 1e-6:
                        ax.set_ylim(center - padding_y, center + padding_y)
                    else:
                        ax.set_ylim(data_min - padding_y, data_max + padding_y)

        # Always update status display (even without new plot data)
        self.update_status_display()

        return self.lines

    def start(self):
        """Start the data acquisition and plotting"""
        if not self.connect_serial():
            return False

        self.setup_plots()

        # Connect click event
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)

        self.running = True
        self.serial_thread = threading.Thread(target=self.read_serial_data)
        self.serial_thread.daemon = True
        self.serial_thread.start()

        self.animation = FuncAnimation(
            self.fig,
            self.update_plot,
            interval=50,
            blit=False,
            cache_frame_data=False,
        )

        print("Plotting started. Close the window to stop.")
        print("Click on control panel buttons to send commands.")

        try:
            plt.show(block=True)
        except Exception as e:
            print(f"Erro ao exibir plot: {e}")
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.stop()

        return True

    def stop(self):
        """Stop data acquisition and close connections"""
        print("Stopping...")
        self.running = False

        if self.serial_thread:
            self.serial_thread.join(timeout=2.0)

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("Serial port closed")

        plt.close("all")


# ==============================================================================
# FUNÇÃO MAIN
# ==============================================================================


def main():
    """Main entry point - Starts with GUI setup."""

    # 1. Cria e exibe o menu de configuração
    root = tk.Tk()

    try:
        from ttkbootstrap import Style

        Style(theme="cosmo")
    except ImportError:
        pass

    menu = SetupMenu(root)
    settings = menu.get_settings()

    if settings is None:
        print("Setup cancelado pelo usuário.")
        sys.exit(0)

    # Pausa para garantir que o Tkinter finalizou completamente
    time.sleep(0.5)

    # 2. Inicia o plotter com as configurações selecionadas
    print(
        f"Iniciando plotter com: Porta={settings['port']}, Baudrate={settings['baudrate']}, Pontos={settings['max_points']}"
    )
    plotter = PendulumDataPlotter(
        port=settings["port"],
        baudrate=settings["baudrate"],
        max_points=settings["max_points"],
    )

    if not plotter.start():
        sys.exit(1)


if __name__ == "__main__":
    main()
