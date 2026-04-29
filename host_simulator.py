#!/usr/bin/env python3
"""
Windowed RS-485 host simulator.

Sends payload 00 01 with CRC-32/ISO-HDLC appended in little-endian order,
waits for the recording_complete response, then receives and saves the
recorded video file.
"""

import binascii
import json
import math
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    messagebox.showerror(
        "Missing dependency",
        "Install pyserial first:\n\npython -m pip install pyserial",
    )
    raise


BAUDRATE = 2000000
COMMAND = b"\x00\x01"
EXPECTED_REPLY = b"recording_complete"
TIMEOUT_SECONDS = 35
INTER_BYTE_DELAY_US = 50

FILE_FRAME_MAGIC = b"VSSU"
FILE_FRAME_META = 1
FILE_FRAME_CHUNK = 2
FILE_FRAME_END = 3
CRC_SIZE_BYTES = 4
FILE_FRAME_HEADER_SIZE = len(FILE_FRAME_MAGIC) + 1 + 4
FILE_IDLE_TIMEOUT_SECONDS = 10
RECEIVED_DIR = Path("received")


def crc32_iso_hdlc(data: bytes) -> int:
    """CRC-32/ISO-HDLC: poly 0x04C11DB7, init/xorout 0xffffffff, reflected."""
    return binascii.crc32(data) & 0xFFFFFFFF


def append_crc32_iso_hdlc(data: bytes) -> bytes:
    return data + crc32_iso_hdlc(data).to_bytes(CRC_SIZE_BYTES, "little")


def file_crc32_iso_hdlc(path: Path) -> int:
    crc = 0
    with open(path, "rb") as file:
        while True:
            chunk = file.read(64 * 1024)
            if not chunk:
                break
            crc = binascii.crc32(chunk, crc)
    return crc & 0xFFFFFFFF


def safe_filename(name: str) -> str:
    filename = Path(name).name.strip()
    return filename or "received_video.h264"


def wait_us(delay_us: float) -> None:
    if delay_us <= 0:
        return
    end_time = time.perf_counter() + delay_us / 1_000_000
    while time.perf_counter() < end_time:
        pass


class HostSimulatorApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("RS-485 Host Simulator")
        self.root.geometry("820x560")
        self.root.minsize(720, 460)

        self.serial_port: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()
        self.worker: Optional[threading.Thread] = None
        self.deadline: Optional[float] = None
        self.log_queue: queue.Queue[str] = queue.Queue()

        self.port_var = tk.StringVar()
        self.baudrate_var = tk.StringVar(value=str(BAUDRATE))
        self.status_var = tk.StringVar(value="No command")
        self.timer_var = tk.StringVar(value=str(TIMEOUT_SECONDS))
        self.connected_var = tk.StringVar(value="Disconnected")

        self._build_ui()
        self.refresh_ports()
        self._drain_logs()
        self._update_countdown()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)

        connection = ttk.LabelFrame(self.root, text="Connection")
        connection.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 6))
        connection.columnconfigure(1, weight=1)

        ttk.Label(connection, text="Port").grid(row=0, column=0, padx=(8, 4), pady=8)
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=4, pady=8)

        ttk.Button(connection, text="Refresh", command=self.refresh_ports).grid(
            row=0, column=2, padx=4, pady=8
        )

        ttk.Label(connection, text="Baud").grid(row=0, column=3, padx=(12, 4), pady=8)
        self.baud_entry = ttk.Entry(connection, textvariable=self.baudrate_var, width=10)
        self.baud_entry.grid(row=0, column=4, padx=4, pady=8)

        self.connect_button = ttk.Button(connection, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=5, padx=(10, 8), pady=8)

        self.connected_label = ttk.Label(connection, textvariable=self.connected_var)
        self.connected_label.grid(row=1, column=0, columnspan=6, sticky="w", padx=8, pady=(0, 8))

        controls = ttk.LabelFrame(self.root, text="Command")
        controls.grid(row=1, column=0, sticky="ew", padx=10, pady=6)
        controls.columnconfigure(3, weight=1)

        self.start_button = ttk.Button(controls, text="Start", command=self.start_command, state="disabled")
        self.start_button.grid(row=0, column=0, padx=8, pady=10)

        ttk.Label(controls, text="Response").grid(row=0, column=1, padx=(12, 4), pady=10)
        self.status_label = tk.Label(
            controls,
            textvariable=self.status_var,
            width=18,
            relief="groove",
            bg="#d9d9d9",
            fg="#111111",
            padx=8,
            pady=4,
        )
        self.status_label.grid(row=0, column=2, padx=4, pady=10)

        ttk.Label(controls, text="Countdown").grid(row=0, column=3, sticky="e", padx=(20, 4), pady=10)
        self.timer_label = tk.Label(
            controls,
            textvariable=self.timer_var,
            width=8,
            relief="groove",
            bg="#f2f2f2",
            fg="#111111",
            padx=8,
            pady=4,
        )
        self.timer_label.grid(row=0, column=4, padx=(4, 8), pady=10)

        logs = ttk.LabelFrame(self.root, text="Logs")
        logs.grid(row=2, column=0, sticky="nsew", padx=10, pady=(6, 10))
        logs.columnconfigure(0, weight=1)
        logs.rowconfigure(0, weight=1)

        self.log_text = tk.Text(logs, wrap="word", height=18, state="disabled")
        self.log_text.grid(row=0, column=0, sticky="nsew")

        scrollbar = ttk.Scrollbar(logs, orient="vertical", command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scrollbar.set)

    def refresh_ports(self) -> None:
        ports = list(list_ports.comports())
        values = [port.device for port in ports]
        self.port_combo["values"] = values

        if values and self.port_var.get() not in values:
            self.port_var.set(values[0])
        elif not values:
            self.port_var.set("")

        self.log("Available COM/serial ports:")
        if ports:
            for port in ports:
                self.log(f"  {port.device}: {port.description} [{port.hwid}]")
        else:
            self.log("  <none>")

    def toggle_connection(self) -> None:
        if self.serial_port and self.serial_port.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("No port", "Select a COM/serial port first.")
            return

        try:
            baudrate = int(self.baudrate_var.get())
        except ValueError:
            messagebox.showerror("Invalid baudrate", "Baudrate must be an integer.")
            return

        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
        except serial.SerialException as exc:
            self.serial_port = None
            self.set_status("Connect error", "red")
            self.log(f"\nERROR RS-485 error: {exc}")
            return

        self.connected_var.set(f"Connected: {self.serial_port.name} @ {baudrate} 8N1")
        self.connect_button.configure(text="Disconnect")
        self.start_button.configure(state="normal")
        self.log(f"\nPort opened: {self.serial_port.name}")
        self.log(f"Parameters: {baudrate} 8N1, timeout={self.serial_port.timeout}")

    def disconnect(self) -> None:
        if self.worker and self.worker.is_alive():
            messagebox.showwarning("Busy", "Wait for the current command to finish.")
            return

        if self.serial_port:
            try:
                self.serial_port.close()
            except serial.SerialException as exc:
                self.log(f"Close error: {exc}")
        self.serial_port = None
        self.connected_var.set("Disconnected")
        self.connect_button.configure(text="Connect")
        self.start_button.configure(state="disabled")
        self.log("Port closed")

    def start_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Connect to a port first.")
            return
        if self.worker and self.worker.is_alive():
            return

        self.deadline = time.monotonic() + TIMEOUT_SECONDS
        self.timer_var.set(str(TIMEOUT_SECONDS))
        self.set_status("Waiting", "gray")
        self.start_button.configure(state="disabled")

        self.worker = threading.Thread(target=self._send_and_wait_worker, daemon=True)
        self.worker.start()

    def _send_and_wait_worker(self) -> None:
        assert self.serial_port is not None

        frame = append_crc32_iso_hdlc(COMMAND)
        response = b""
        file_saved = False

        self.log(f"\n{'=' * 50}")
        self.log(f"Sending command on {self.serial_port.name} ({self.serial_port.baudrate} baud)")
        self.log(f"{'=' * 50}")

        try:
            with self.serial_lock:
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()

                self.log(f"\n[SEND] Payload: {COMMAND!r}")
                self.log(f"       CRC-32/ISO-HDLC: 0x{crc32_iso_hdlc(COMMAND):08x}")
                self.log(f"       Frame: {frame.hex()}")

                written = 0
                for byte in frame:
                    written += self.serial_port.write(bytes([byte]))
                    self.serial_port.flush()
                    wait_us(INTER_BYTE_DELAY_US)

                self.log(f"       Bytes written to COM port: {written}")
                self.log("       Command sent. Waiting for response after video recording.")

                self.log(f"\n[WAIT] Waiting for response (timeout {TIMEOUT_SECONDS} sec)...")
                start_time = time.monotonic()

                while time.monotonic() - start_time < TIMEOUT_SECONDS:
                    if self.serial_port.in_waiting > 0:
                        chunk = self.serial_port.read(self.serial_port.in_waiting)
                        response += chunk
                        self.log(f"[RECV] Received: {chunk!r}")
                        if EXPECTED_REPLY in response:
                            break
                    time.sleep(0.01)

                if EXPECTED_REPLY in response:
                    reply_idx = response.find(EXPECTED_REPLY)
                    reply_end = reply_idx + len(EXPECTED_REPLY)
                    file_buffer = response[reply_end:]
                    self.log(f"\nOK Full response: {response[:reply_end]!r}")
                    self.log(f"  Size: {reply_end} bytes")
                    if file_buffer:
                        self.log(f"[FILE] Initial file bytes after response: {len(file_buffer)}")
                    self.root.after(0, lambda: self.set_status("Receiving file", "green"))
                    self.deadline = None
                    self.root.after(0, lambda: self.timer_var.set("0"))
                    saved_path = self._receive_file(file_buffer)
                    if saved_path:
                        file_saved = True
                        self.log(f"\nOK File saved: {saved_path}")

        except serial.SerialException as exc:
            self.log(f"\nERROR RS-485 error: {exc}")
            self.root.after(0, lambda: self.set_status("Error", "red"))
            self.root.after(0, self._finish_command)
            return

        if file_saved:
            self.root.after(0, lambda: self.set_status("Saved", "green"))
        elif EXPECTED_REPLY in response:
            self.root.after(0, lambda: self.set_status("File error", "red"))
        elif response:
            self.log(f"\n! Response received, but without expected {EXPECTED_REPLY!r}: {response!r}")
            self.log(f"  Size: {len(response)} bytes")
            self.log(f"  Hex: {response.hex()}")
            self.root.after(0, lambda: self.set_status("Unexpected", "red"))
        else:
            self.log("\nERROR Response not received before timeout")
            self.root.after(0, lambda: self.set_status("Timeout", "red"))

        self.root.after(0, self._finish_command)

    def _extract_file_frame(self, buffer: bytearray) -> Optional[tuple[int, bytes]]:
        magic_idx = buffer.find(FILE_FRAME_MAGIC)
        if magic_idx < 0:
            if len(buffer) > len(FILE_FRAME_MAGIC):
                del buffer[:-len(FILE_FRAME_MAGIC) + 1]
            return None

        if magic_idx > 0:
            self.log(f"[FILE] Dropped {magic_idx} byte(s) before frame magic")
            del buffer[:magic_idx]

        if len(buffer) < FILE_FRAME_HEADER_SIZE:
            return None

        frame_type = buffer[len(FILE_FRAME_MAGIC)]
        payload_len = int.from_bytes(
            buffer[len(FILE_FRAME_MAGIC) + 1:FILE_FRAME_HEADER_SIZE],
            "little",
        )
        frame_len = FILE_FRAME_HEADER_SIZE + payload_len + CRC_SIZE_BYTES
        if len(buffer) < frame_len:
            return None

        payload_start = FILE_FRAME_HEADER_SIZE
        payload_end = payload_start + payload_len
        payload = bytes(buffer[payload_start:payload_end])
        received_crc = int.from_bytes(buffer[payload_end:frame_len], "little")
        expected_crc = crc32_iso_hdlc(payload)
        del buffer[:frame_len]

        if received_crc != expected_crc:
            raise ValueError(
                f"Bad frame CRC: received=0x{received_crc:08x}, expected=0x{expected_crc:08x}"
            )

        return frame_type, payload

    def _receive_file(self, initial_buffer: bytes) -> Optional[Path]:
        assert self.serial_port is not None

        buffer = bytearray(initial_buffer)
        output_file = None
        output_path: Optional[Path] = None
        expected_name = ""
        expected_size = 0
        expected_crc = 0
        received_size = 0
        last_log_percent = -1
        idle_start = time.monotonic()

        self.log("[FILE] Waiting for file transfer frames...")

        try:
            while True:
                frame = self._extract_file_frame(buffer)
                if frame is None:
                    if self.serial_port.in_waiting > 0:
                        chunk = self.serial_port.read(self.serial_port.in_waiting)
                        buffer.extend(chunk)
                        idle_start = time.monotonic()
                    elif time.monotonic() - idle_start > FILE_IDLE_TIMEOUT_SECONDS:
                        raise TimeoutError(f"No file data for {FILE_IDLE_TIMEOUT_SECONDS} sec")
                    else:
                        time.sleep(0.005)
                    continue

                frame_type, payload = frame

                if frame_type == FILE_FRAME_META:
                    metadata = json.loads(payload.decode("utf-8"))
                    expected_name = safe_filename(str(metadata["name"]))
                    expected_size = int(metadata["size"])
                    expected_crc = int(metadata["crc32"])

                    RECEIVED_DIR.mkdir(parents=True, exist_ok=True)
                    output_path = RECEIVED_DIR / expected_name
                    suffix = 1
                    while output_path.exists():
                        output_path = RECEIVED_DIR / (
                            f"{Path(expected_name).stem}_{suffix}{Path(expected_name).suffix}"
                        )
                        suffix += 1

                    output_file = open(output_path, "wb")
                    self.log(
                        f"[FILE] Metadata: name={expected_name}, "
                        f"size={expected_size}, crc=0x{expected_crc:08x}"
                    )

                elif frame_type == FILE_FRAME_CHUNK:
                    if output_file is None:
                        raise ValueError("Chunk received before metadata")
                    output_file.write(payload)
                    received_size += len(payload)
                    percent = int(received_size * 100 / expected_size) if expected_size else 100
                    if percent != last_log_percent and (
                        percent % 10 == 0 or received_size == expected_size
                    ):
                        self.log(f"[FILE] Received {received_size}/{expected_size} bytes ({percent}%)")
                        last_log_percent = percent

                elif frame_type == FILE_FRAME_END:
                    if output_file is None or output_path is None:
                        raise ValueError("End frame received before metadata")
                    output_file.close()
                    output_file = None

                    actual_size = output_path.stat().st_size
                    actual_crc = file_crc32_iso_hdlc(output_path)
                    if actual_size != expected_size:
                        raise ValueError(f"Bad file size: {actual_size}, expected {expected_size}")
                    if actual_crc != expected_crc:
                        raise ValueError(
                            f"Bad file CRC: received=0x{actual_crc:08x}, expected=0x{expected_crc:08x}"
                        )

                    self.log(
                        f"[FILE] Transfer complete: {actual_size} bytes, "
                        f"crc=0x{actual_crc:08x}"
                    )
                    return output_path

                else:
                    raise ValueError(f"Unknown file frame type: {frame_type}")

        except (OSError, ValueError, TimeoutError, json.JSONDecodeError) as exc:
            self.log(f"\nERROR File receive failed: {exc}")
            return None
        finally:
            if output_file is not None:
                output_file.close()

    def _finish_command(self) -> None:
        self.deadline = None
        self.timer_var.set("0")
        if self.serial_port and self.serial_port.is_open:
            self.start_button.configure(state="normal")

    def set_status(self, text: str, color: str) -> None:
        palette = {
            "gray": ("#d9d9d9", "#111111"),
            "green": ("#38a169", "#ffffff"),
            "red": ("#e53e3e", "#ffffff"),
        }
        bg, fg = palette[color]
        self.status_var.set(text)
        self.status_label.configure(bg=bg, fg=fg)

    def _update_countdown(self) -> None:
        if self.deadline is not None:
            remaining = max(0, math.ceil(self.deadline - time.monotonic()))
            self.timer_var.set(str(remaining))
        self.root.after(200, self._update_countdown)

    def log(self, message: str) -> None:
        self.log_queue.put(message)

    def _drain_logs(self) -> None:
        try:
            while True:
                message = self.log_queue.get_nowait()
                self.log_text.configure(state="normal")
                self.log_text.insert("end", message + "\n")
                self.log_text.see("end")
                self.log_text.configure(state="disabled")
        except queue.Empty:
            pass
        self.root.after(50, self._drain_logs)

    def on_close(self) -> None:
        if self.worker and self.worker.is_alive():
            if not messagebox.askyesno("Busy", "A command is running. Close anyway?"):
                return
        if self.serial_port:
            try:
                self.serial_port.close()
            except serial.SerialException:
                pass
        self.root.destroy()


def main() -> int:
    root = tk.Tk()
    HostSimulatorApp(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
