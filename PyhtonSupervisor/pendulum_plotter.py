"""
Real-time plotter for inverted pendulum on cart system
Receives data via UART in JSON format and plots 5 graphs.
Supports sending JSON commands to the microcontroller.

Starts with a graphical setup menu (Tkinter) for configuration.

Message format:
- JSON Status: {"ts":12345,"mode":2,"cart_pos":0.025,...}
- JSON Info: {"type":"INFO","name":"...","version":"..."}
- JSON Response: {"status":"ok","msg":"..."}
- JSON Command: {"cmd":"MODE","value":5}
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
import json

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
        self.max_points_var = tk.StringVar(master, value="500")
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
        self.max_points_options = ["500", "1000", "2000", "5000", "10000", "20000"]

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
        self.current_vel = deque(maxlen=max_points)
        self.motor_acc = deque(maxlen=max_points)
        self.pwm_freq = deque(maxlen=max_points)

        self.cart_pos = deque(maxlen=max_points)
        self.cart_vel = deque(maxlen=max_points)
        self.cart_pos_filt = deque(maxlen=max_points)
        self.cart_vel_filt = deque(maxlen=max_points)

        self.pendulum_pos = deque(maxlen=max_points)
        self.pendulum_vel = deque(maxlen=max_points)
        self.pendulum_pos_filt = deque(maxlen=max_points)
        self.pendulum_vel_filt = deque(maxlen=max_points)

        # Status data
        self.control_mode = 0
        self.cal_x_status = 0
        self.cal_t_status = 0
        self.ref_pos = 0.0
        self.shadow_ref = 0.0

        # Device info
        self.device_name = "Unknown"
        self.device_version = "Unknown"

        # Control mode names
        self.control_modes = {
            0: "OFF",
            1: "CART_PID",
            2: "CART_LEAD",
            3: "CART_SF",
            4: "PEND_SF",
            5: "FULL_LQR",
        }

        # Calibration status mapping
        self.cal_status_map = {0: "Not Done", 1: "Running", 2: "Done"}

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

    def reset_data(self):
        """Reset all collected data"""
        self.time_data.clear()
        self.real_time.clear()
        self.target_vel.clear()
        self.current_vel.clear()
        self.motor_acc.clear()
        self.pwm_freq.clear()
        self.cart_pos.clear()
        self.cart_vel.clear()
        self.cart_pos_filt.clear()
        self.cart_vel_filt.clear()
        self.pendulum_pos.clear()
        self.pendulum_vel.clear()
        self.pendulum_pos_filt.clear()
        self.pendulum_vel_filt.clear()

        # Reset start time
        self.start_time = None

        # Clear pending data in queue
        while not self.data_queue.empty():
            try:
                self.data_queue.get_nowait()
            except queue.Empty:
                break

        print("✓ Data reset complete")

    def setup_plots(self):
        """Configure the plot appearance and create line objects"""

        titles = [
            "Motor Velocity (m/s)",
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
            (line,) = ax.plot(
                [],
                [],
                color=color,
                linewidth=1.5,
                label=("Target" if i == 0 else ("Raw" if i in [2, 3, 4, 5] else "")),
            )
            self.lines.append(line)

            # Add current velocity line for motor
            if i == 0:  # Motor Velocity
                (line_current,) = ax.plot(
                    [],
                    [],
                    color="black",
                    linewidth=1,
                    linestyle="--",
                    label="Current",
                )
                self.lines.append(line_current)
                ax.legend(loc="upper right", fontsize=8)
            # Add filtered lines for cart position and velocity
            elif i == 2:  # Cart Position
                (line_filt,) = ax.plot(
                    [],
                    [],
                    color="black",
                    linewidth=1,
                    linestyle="--",
                    label="Filtered",
                )
                self.lines.append(line_filt)
                ax.legend(loc="upper right", fontsize=8)
            elif i == 3:  # Cart Velocity
                (line_filt,) = ax.plot(
                    [],
                    [],
                    color="black",
                    linewidth=1,
                    linestyle="--",
                    label="Filtered",
                )
                self.lines.append(line_filt)
                ax.legend(loc="upper right", fontsize=8)
            # Add filtered lines for pendulum position and velocity
            elif i == 4:  # Pendulum Position
                (line_filt,) = ax.plot(
                    [],
                    [],
                    color="black",
                    linewidth=1,
                    linestyle="--",
                    label="Filtered",
                )
                self.lines.append(line_filt)
                ax.legend(loc="upper right", fontsize=8)
            elif i == 5:  # Pendulum Velocity
                (line_filt,) = ax.plot(
                    [],
                    [],
                    color="black",
                    linewidth=1,
                    linestyle="--",
                    label="Filtered",
                )
                self.lines.append(line_filt)
                ax.legend(loc="upper right", fontsize=8)

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

        # Reset Data button
        y_pos = 0.68
        reset_height = 0.05
        reset_rect = plt.Rectangle(
            (0.05, y_pos - reset_height),
            0.9,
            reset_height,
            facecolor="lightcoral",
            edgecolor="darkred",
            linewidth=2,
            transform=self.control_ax.transAxes,
            picker=True,
        )
        self.control_ax.add_patch(reset_rect)
        self.control_ax.text(
            0.5,
            y_pos - reset_height / 2,
            "RESET DATA",
            ha="center",
            va="center",
            fontsize=9,
            fontweight="bold",
            transform=self.control_ax.transAxes,
        )
        self.reset_button = {
            "rect": reset_rect,
            "y_min": y_pos - reset_height,
            "y_max": y_pos,
        }

        # Save CSV button
        y_pos -= 0.06
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

        # Mode buttons section
        y_pos = 0.54
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

        # Generate mode list dynamically
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
                self.send_json_command("MODE", mode_id)
                print(f"Sent JSON: MODE={mode_id}")
                return

        # Check if Reset Data button was clicked
        if hasattr(self, "reset_button"):
            if (
                0.05 <= x <= 0.95
                and self.reset_button["y_min"] <= y <= self.reset_button["y_max"]
            ):
                print("Reset Data button clicked!")
                self.reset_data()
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

                # Send JSON command
                self.send_json_command("SREF", ref_value)
                print(f"Sent JSON: SREF={ref_value:.3f}")
                return

        # Check apply button
        if hasattr(self, "apply_button"):
            ab = self.apply_button
            if 0.05 <= x <= 0.95 and ab["y_min"] <= y <= ab["y_max"]:
                self.send_json_command("AREF")
                print("Sent JSON: AREF")
                return

    def send_json_command(self, cmd, value=None):
        """Send JSON command to microcontroller"""
        if self.serial_port and self.serial_port.is_open:
            try:
                # Build JSON command
                command = {"cmd": cmd}
                if value is not None:
                    command["value"] = value

                # Convert to JSON string and send
                json_str = json.dumps(command)
                self.serial_port.write((json_str + "\n").encode("utf-8"))
                print(f"→ {json_str}")
            except Exception as e:
                print(f"Error sending command: {e}")

    def update_status_display(self):
        """Update status text in control panel"""
        cal_x_str = self.cal_status_map.get(self.cal_x_status, "Unknown")
        cal_t_str = self.cal_status_map.get(self.cal_t_status, "Unknown")
        mode_str = self.control_modes.get(self.control_mode, "?")

        status = f"""Device: {self.device_name}
Version: {self.device_version}

Mode: {mode_str}
Cal X: {cal_x_str}
Cal T: {cal_t_str}

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
                        "Current_Vel",
                        "PWM_Freq",
                        "Cart_Pos",
                        "Cart_Pos_Filt",
                        "Cart_Vel",
                        "Cart_Vel_Filt",
                        "Pendulum_Pos",
                        "Pendulum_Pos_Filt",
                        "Pendulum_Vel",
                        "Pendulum_Vel_Filt",
                    ]
                )

                # Write data rows
                for i in range(len(self.time_data)):
                    writer.writerow(
                        [
                            self.time_data[i],
                            self.real_time[i],
                            self.target_vel[i],
                            self.current_vel[i],
                            self.pwm_freq[i],
                            self.cart_pos[i],
                            self.cart_pos_filt[i],
                            self.cart_vel[i],
                            self.cart_vel_filt[i],
                            self.pendulum_pos[i],
                            self.pendulum_pos_filt[i],
                            self.pendulum_vel[i],
                            self.pendulum_vel_filt[i],
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
                            self.process_json_line(line)

                time.sleep(0.01)

            except Exception as e:
                print(f"Serial read error: {e}")
                self.running = False
                break

    def process_json_line(self, line):
        """Process incoming JSON line"""
        try:
            # Parse JSON
            data = json.loads(line)

            # Check message type
            if "type" in data:
                # INFO message
                if data["type"] == "INFO":
                    self.device_name = data.get("name", "Unknown")
                    self.device_version = data.get("version", "Unknown")
                    print(f"ℹ INFO: {data.get('name')} v{data.get('version')}")
                    print(f"  Author: {data.get('author', 'Unknown')}")
                    print(
                        f"  Date: {data.get('date', 'Unknown')} {data.get('time', '')}"
                    )

            elif "status" in data:
                # RESPONSE message
                status = data.get("status", "unknown")
                msg = data.get("msg", "")
                if status == "ok":
                    print(f"✓ {msg}")
                else:
                    print(f"✗ ERROR: {msg}")
                    if "value" in data:
                        print(f"  Value: {data['value']}")

            elif "ts" in data:
                # STATUS message
                self.parse_status_data(data)

            else:
                # Unknown JSON format
                print(f"⚠ Unknown JSON: {line}")

        except json.JSONDecodeError as e:
            # Not valid JSON
            print(f"⚠ Invalid JSON: {line}")
            print(f"  Error: {e}")

    def parse_status_data(self, data):
        """
        Parse STATUS JSON data
        Expected fields: ts, mode, cal_x_status, cal_t_status, mot_target_vel,
                        mot_current_vel, mot_acc, mot_pwm, cart_pos, cart_vel,
                        cart_pos_filt, cart_vel_filt, pend_pos, pend_vel,
                        pend_pos_filt, pend_vel_filt, ref_pos, ref_shadow
        """
        try:
            current_time = time.time()
            if self.start_time is None:
                self.start_time = current_time

            # Extract all values with defaults
            timestamp = data.get("ts", 0)
            control_mode = data.get("mode", 0)
            cal_x_status = data.get("cal_x_status", 0)
            cal_t_status = data.get("cal_t_status", 0)
            target_vel = data.get("mot_target_vel", 0.0)
            current_vel = data.get("mot_current_vel", 0.0)
            motor_acc = data.get("mot_acc", 0.0)
            pwm_freq = data.get("mot_pwm", 0)
            cart_pos = data.get("cart_pos", 0.0)
            cart_vel = data.get("cart_vel", 0.0)
            cart_pos_filt = data.get("cart_pos_filt", 0.0)
            cart_vel_filt = data.get("cart_vel_filt", 0.0)
            pendulum_pos = data.get("pend_pos", 0.0)
            pendulum_vel = data.get("pend_vel", 0.0)
            pendulum_pos_filt = data.get("pend_pos_filt", 0.0)
            pendulum_vel_filt = data.get("pend_vel_filt", 0.0)
            ref_pos = data.get("ref_pos", 0.0)
            shadow_ref = data.get("ref_shadow", 0.0)

            # Create data dict for plotting
            plot_data = {
                "timestamp": timestamp,
                "real_time": current_time - self.start_time,
                "control_mode": control_mode,
                "cal_x_status": cal_x_status,
                "cal_t_status": cal_t_status,
                "target_vel": target_vel,
                "current_vel": current_vel,
                "motor_acc": motor_acc,
                "pwm_freq": pwm_freq,
                "cart_pos": cart_pos,
                "cart_vel": cart_vel,
                "cart_pos_filt": cart_pos_filt,
                "cart_vel_filt": cart_vel_filt,
                "pendulum_pos": pendulum_pos,
                "pendulum_vel": pendulum_vel,
                "pendulum_pos_filt": pendulum_pos_filt,
                "pendulum_vel_filt": pendulum_vel_filt,
                "ref_pos": ref_pos,
                "shadow_ref": shadow_ref,
            }

            # Update status variables
            self.control_mode = plot_data["control_mode"]
            self.cal_x_status = plot_data["cal_x_status"]
            self.cal_t_status = plot_data["cal_t_status"]
            self.ref_pos = plot_data["ref_pos"]
            self.shadow_ref = plot_data["shadow_ref"]

            # Add to queue for plotting
            self.data_queue.put(plot_data)

        except Exception as e:
            print(f"Error parsing status data: {e}")

    def update_plot(self, frame):
        """Update plot with new data from queue"""
        data_updated = False
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
                self.time_data.append(data["timestamp"])
                self.real_time.append(data["real_time"])
                self.target_vel.append(data["target_vel"])
                self.current_vel.append(data["current_vel"])
                self.motor_acc.append(data["motor_acc"])
                self.pwm_freq.append(data["pwm_freq"])
                self.cart_pos.append(data["cart_pos"])
                self.cart_vel.append(data["cart_vel"])
                self.cart_pos_filt.append(data["cart_pos_filt"])
                self.cart_vel_filt.append(data["cart_vel_filt"])

                # Pendulum position with unwrapping
                pend_pos = data["pendulum_pos"]
                if len(self.pendulum_pos) > 0:
                    # Calculate difference from last position
                    last_pos = self.pendulum_pos[-1]
                    diff = pend_pos - last_pos

                    # Unwrap if there's a discontinuity (jump > π)
                    if diff > np.pi:
                        pend_pos -= 2 * np.pi
                    elif diff < -np.pi:
                        pend_pos += 2 * np.pi

                    # Accumulate unwrapped position
                    if not hasattr(self, "pendulum_pos_offset"):
                        self.pendulum_pos_offset = 0
                    self.pendulum_pos_offset += (pend_pos - last_pos) - diff

                self.pendulum_pos.append(data["pendulum_pos"])
                self.pendulum_vel.append(data["pendulum_vel"])
                self.pendulum_pos_filt.append(data["pendulum_pos_filt"])
                self.pendulum_vel_filt.append(data["pendulum_vel_filt"])
                data_updated = True
            except queue.Empty:
                break

        if data_updated and len(self.real_time) > 1:
            time_array = np.array(self.real_time)

            # Unwrap pendulum position for display
            pendulum_pos_unwrapped = np.unwrap(self.pendulum_pos)
            pendulum_pos_filt_unwrapped = np.unwrap(self.pendulum_pos_filt)

            # Update all plots
            self.lines[0].set_data(time_array, self.target_vel)
            self.lines[1].set_data(time_array, self.current_vel)
            self.lines[2].set_data(time_array, self.pwm_freq)
            self.lines[3].set_data(time_array, self.cart_pos)
            self.lines[4].set_data(time_array, self.cart_pos_filt)
            self.lines[5].set_data(time_array, self.cart_vel)
            self.lines[6].set_data(time_array, self.cart_vel_filt)
            self.lines[7].set_data(time_array, pendulum_pos_unwrapped)
            self.lines[8].set_data(time_array, pendulum_pos_filt_unwrapped)
            self.lines[9].set_data(time_array, self.pendulum_vel)
            self.lines[10].set_data(time_array, self.pendulum_vel_filt)

            data_lists = [
                list(self.target_vel) + list(self.current_vel),
                list(self.pwm_freq),
                list(self.cart_pos) + list(self.cart_pos_filt),
                list(self.cart_vel) + list(self.cart_vel_filt),
                list(pendulum_pos_unwrapped) + list(pendulum_pos_filt_unwrapped),
                list(self.pendulum_vel) + list(self.pendulum_vel_filt),
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

        print("=" * 60)
        print("Plotting started. Close the window to stop.")
        print("Click on control panel buttons to send JSON commands.")
        print("=" * 60)

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

    print("=" * 60)
    print("Inverted Pendulum Real-Time Plotter - JSON Edition")
    print("=" * 60)

    # 1. Create and show setup menu
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

    # Small pause to ensure Tkinter finishes completely
    time.sleep(0.5)

    # 2. Start plotter with selected settings
    print(
        f"Iniciando plotter com: Porta={settings['port']}, "
        f"Baudrate={settings['baudrate']}, Pontos={settings['max_points']}"
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
