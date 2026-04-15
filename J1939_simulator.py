import ctypes
import os
import platform
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk

USBCAN_II = 4
DEFAULT_DEVICE_TYPE = USBCAN_II
DEFAULT_DEVICE_INDEX = 0
DEFAULT_CAN_INDEX = 0
DEFAULT_DLL_NAME = "ECanVci.dll"
TIMING0_250K = 0x01
TIMING1_250K = 0x1C

PGN_CCVS = 0x00FEF1  # SPN 84: wheel-based vehicle speed
PGN_ETC2 = 0x00F005  # SPN 523: transmission current gear
PGN_LC = 0x00FE41  # SPN 2369/2367: turn signals
PGN_DC1 = 0x00FE4E  # SPN 1821: position of doors
PGN_CLASS_IV_V_SWITCH = 0x00FF00  # Camera view switching (Class IV/V)

PRIORITY_DEFAULT = 6
SOURCE_ADDRESS = 0x00


def j1939_id(priority: int, pgn: int, source_address: int) -> int:
    return ((priority & 0x7) << 26) | ((pgn & 0x3FFFF) << 8) | (source_address & 0xFF)


def build_ccvs_data(speed_kmh: float) -> list[int]:
    raw_speed = int(max(0.0, min(float(speed_kmh), 250.996)) * 256)
    data = [0xFF] * 8
    data[1] = raw_speed & 0xFF
    data[2] = (raw_speed >> 8) & 0xFF
    return data


def build_etc2_data(gear_choice: str) -> list[int]:
    gear_map = {
        "Reverse": 124,  # -1 + 125 offset
        "Neutral": 125,  # 0 + 125 offset
        "Drive": 126,  # 1 + 125 offset
        "Park": 251,  # per J1939 SPN 523 definition
    }
    data = [0xFF] * 8
    data[3] = gear_map.get(gear_choice, 125)
    return data


def build_lc_data(left_active: bool, right_active: bool) -> list[int]:
    data = [0xFF] * 8
    # SPN 2369 Right turn command at 2.5 (byte 2 bits 5-6)
    # SPN 2367 Left turn command at 2.7 (byte 2 bits 7-8)
    right_cmd = 0b01 if right_active else 0b00
    left_cmd = 0b01 if left_active else 0b00
    data[1] = (data[1] & 0x0F) | (right_cmd << 4) | (left_cmd << 6)
    return data


def build_dc1_data(door_position: str) -> list[int]:
    door_map = {
        "Open (0000)": 0x0,
        "Closing (0001)": 0x1,
        "Closed (0010)": 0x2,
        "Error (1110)": 0xE,
        "Not available (1111)": 0xF,
    }
    data = [0xFF] * 8
    value = door_map.get(door_position, 0xF)
    data[0] = (data[0] & 0xF0) | (value & 0x0F)
    return data


def build_class_iv_v_switch_data(camera_view_command: str) -> list[int]:
    command_map = {
        "Retain (0x00)": 0x00,
        "Switch to Class IV (0x04)": 0x04,
        "Switch to Class V (0x05)": 0x05,
    }
    data = [0xFF] * 8
    data[0] = command_map.get(camera_view_command, 0x00)
    return data


class CAN_OBJ(ctypes.Structure):
    _fields_ = [
        ("ID", ctypes.c_uint),
        ("TimeStamp", ctypes.c_uint),
        ("TimeFlag", ctypes.c_ubyte),
        ("SendType", ctypes.c_ubyte),
        ("RemoteFlag", ctypes.c_ubyte),
        ("ExternFlag", ctypes.c_ubyte),
        ("DataLen", ctypes.c_ubyte),
        ("Data", ctypes.c_ubyte * 8),
        ("Reserved", ctypes.c_ubyte * 3),
    ]


class INIT_CONFIG(ctypes.Structure):
    _fields_ = [
        ("AccCode", ctypes.c_uint),
        ("AccMask", ctypes.c_uint),
        ("Reserved", ctypes.c_uint),
        ("Filter", ctypes.c_ubyte),
        ("Timing0", ctypes.c_ubyte),
        ("Timing1", ctypes.c_ubyte),
        ("Mode", ctypes.c_ubyte),
    ]


@dataclass
class DeviceConfig:
    dll_path: str
    device_type: int
    device_index: int
    can_index: int
    timing0: int
    timing1: int


class USBCANDevice:
    def __init__(self, config: DeviceConfig) -> None:
        self.config = config
        self.dll = ctypes.WinDLL(config.dll_path)
        self._bind_functions()

    def _bind_functions(self) -> None:
        self.dll.OpenDevice.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
        self.dll.OpenDevice.restype = ctypes.c_uint
        self.dll.CloseDevice.argtypes = [ctypes.c_uint, ctypes.c_uint]
        self.dll.CloseDevice.restype = ctypes.c_uint
        self.dll.InitCAN.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(INIT_CONFIG)]
        self.dll.InitCAN.restype = ctypes.c_uint
        self.dll.StartCAN.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
        self.dll.StartCAN.restype = ctypes.c_uint
        self.dll.Transmit.argtypes = [
            ctypes.c_uint,
            ctypes.c_uint,
            ctypes.c_uint,
            ctypes.POINTER(CAN_OBJ),
            ctypes.c_ulong,
        ]
        self.dll.Transmit.restype = ctypes.c_ulong

    def open(self) -> None:
        result = self.dll.OpenDevice(self.config.device_type, self.config.device_index, 0)
        if result == 0:
            raise RuntimeError("OpenDevice failed.")
        init_config = INIT_CONFIG(
            AccCode=0,
            AccMask=0xFFFFFFFF,
            Reserved=0,
            Filter=0,
            Timing0=self.config.timing0,
            Timing1=self.config.timing1,
            Mode=0,
        )
        if self.dll.InitCAN(
            self.config.device_type,
            self.config.device_index,
            self.config.can_index,
            ctypes.byref(init_config),
        ) == 0:
            raise RuntimeError("InitCAN failed.")
        if self.dll.StartCAN(self.config.device_type, self.config.device_index, self.config.can_index) == 0:
            raise RuntimeError("StartCAN failed.")

    def close(self) -> None:
        self.dll.CloseDevice(self.config.device_type, self.config.device_index)

    def send(self, frame_id: int, data: list[int]) -> int:
        can_obj = CAN_OBJ()
        can_obj.ID = frame_id
        can_obj.TimeStamp = 0
        can_obj.TimeFlag = 0
        can_obj.SendType = 0
        can_obj.RemoteFlag = 0
        can_obj.ExternFlag = 1
        can_obj.DataLen = len(data)
        for index, value in enumerate(data):
            can_obj.Data[index] = value
        return int(
            self.dll.Transmit(
                self.config.device_type,
                self.config.device_index,
                self.config.can_index,
                ctypes.byref(can_obj),
                1,
            )
        )


class SimulatorApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("CMS CAN Simulator")
        self.device: USBCANDevice | None = None
        self.send_job: str | None = None
        self.is_connected = False
        self._build_ui()

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=12)
        main.grid(sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)

        self.status_text = tk.StringVar(value="Status: Disconnected")
        ttk.Label(main, textvariable=self.status_text).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 8))

        ttk.Label(main, text="DLL path").grid(row=1, column=0, sticky="w")
        self.dll_path = tk.StringVar(value=DEFAULT_DLL_NAME)
        ttk.Entry(main, textvariable=self.dll_path).grid(row=1, column=1, sticky="ew")

        ttk.Label(main, text="Speed (km/h)").grid(row=2, column=0, sticky="w")
        self.speed_kmh = tk.StringVar(value="0")
        ttk.Entry(main, textvariable=self.speed_kmh).grid(row=2, column=1, sticky="ew")

        ttk.Label(main, text="Source address (0-255 or 0x00-0xFF)").grid(row=3, column=0, sticky="w")
        self.source_address = tk.StringVar(value=f"0x{SOURCE_ADDRESS:02X}")
        ttk.Entry(main, textvariable=self.source_address).grid(row=3, column=1, sticky="ew")

        ttk.Label(main, text="Gear (SPN 523)").grid(row=4, column=0, sticky="w")
        self.gear = tk.StringVar(value="Neutral")
        ttk.Combobox(main, textvariable=self.gear, values=["Reverse", "Neutral", "Drive", "Park"], state="readonly").grid(
            row=4, column=1, sticky="ew"
        )

        self.turn_left = tk.BooleanVar(value=False)
        self.turn_right = tk.BooleanVar(value=False)
        ttk.Checkbutton(main, text="Left turn active", variable=self.turn_left).grid(row=5, column=0, sticky="w")
        ttk.Checkbutton(main, text="Right turn active", variable=self.turn_right).grid(row=5, column=1, sticky="w")

        ttk.Label(main, text="Enabled PGNs").grid(row=6, column=0, sticky="w")
        pgn_frame = ttk.Frame(main)
        pgn_frame.grid(row=6, column=1, sticky="w")
        self.ccvs_enabled = tk.BooleanVar(value=True)
        self.etc2_enabled = tk.BooleanVar(value=True)
        self.lc_enabled = tk.BooleanVar(value=True)
        self.dc1_enabled = tk.BooleanVar(value=True)
        self.class_iv_v_enabled = tk.BooleanVar(value=False)
        ttk.Checkbutton(pgn_frame, text="CCVS", variable=self.ccvs_enabled).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(pgn_frame, text="ETC2", variable=self.etc2_enabled).grid(row=0, column=1, sticky="w", padx=(8, 0))
        ttk.Checkbutton(pgn_frame, text="LC", variable=self.lc_enabled).grid(row=1, column=0, sticky="w")
        ttk.Checkbutton(pgn_frame, text="DC1", variable=self.dc1_enabled).grid(row=1, column=1, sticky="w", padx=(8, 0))
        ttk.Checkbutton(pgn_frame, text="Class IV/V", variable=self.class_iv_v_enabled).grid(
            row=2, column=0, sticky="w", columnspan=2
        )

        ttk.Label(main, text="Door position (SPN 1821)").grid(row=7, column=0, sticky="w")
        self.door_position = tk.StringVar(value="Closed (0010)")
        ttk.Combobox(
            main,
            textvariable=self.door_position,
            values=["Open (0000)", "Closing (0001)", "Closed (0010)", "Error (1110)", "Not available (1111)"],
            state="readonly",
        ).grid(row=7, column=1, sticky="ew")

        ttk.Label(main, text="Class IV/V camera command").grid(row=8, column=0, sticky="w")
        self.camera_view_command = tk.StringVar(value="Retain (0x00)")
        ttk.Combobox(
            main,
            textvariable=self.camera_view_command,
            values=["Retain (0x00)", "Switch to Class IV (0x04)", "Switch to Class V (0x05)"],
            state="readonly",
        ).grid(row=8, column=1, sticky="ew")

        ttk.Label(main, text="Send interval (ms)").grid(row=9, column=0, sticky="w")
        self.interval_ms = tk.IntVar(value=250)
        ttk.Entry(main, textvariable=self.interval_ms).grid(row=9, column=1, sticky="ew")

        ttk.Label(main, text="CCVS ID/Data").grid(row=10, column=0, sticky="w")
        self.ccvs_text = tk.StringVar(value="")
        ttk.Label(main, textvariable=self.ccvs_text).grid(row=10, column=1, sticky="w")

        ttk.Label(main, text="ETC2 ID/Data").grid(row=11, column=0, sticky="w")
        self.etc2_text = tk.StringVar(value="")
        ttk.Label(main, textvariable=self.etc2_text).grid(row=11, column=1, sticky="w")

        ttk.Label(main, text="LC ID/Data").grid(row=12, column=0, sticky="w")
        self.lc_text = tk.StringVar(value="")
        ttk.Label(main, textvariable=self.lc_text).grid(row=12, column=1, sticky="w")

        ttk.Label(main, text="DC1 ID/Data").grid(row=13, column=0, sticky="w")
        self.dc1_text = tk.StringVar(value="")
        ttk.Label(main, textvariable=self.dc1_text).grid(row=13, column=1, sticky="w")

        ttk.Label(main, text="Class IV/V ID/Data").grid(row=14, column=0, sticky="w")
        self.class_iv_v_text = tk.StringVar(value="")
        ttk.Label(main, textvariable=self.class_iv_v_text).grid(row=14, column=1, sticky="w")

        buttons = ttk.Frame(main)
        buttons.grid(row=15, column=0, columnspan=2, pady=8, sticky="ew")
        self.connect_button = ttk.Button(buttons, text="Connect", command=self.connect)
        self.connect_button.grid(row=0, column=0, padx=4)
        self.disconnect_button = ttk.Button(buttons, text="Disconnect", command=self.disconnect)
        self.disconnect_button.grid(row=0, column=1, padx=4)
        self.start_button = ttk.Button(buttons, text="Start Periodic", command=self.start_periodic)
        self.start_button.grid(row=0, column=2, padx=4)
        self.stop_button = ttk.Button(buttons, text="Stop Periodic", command=self.stop_periodic)
        self.stop_button.grid(row=0, column=3, padx=4)

        self._update_button_states()
        self.refresh_preview()

    def resolve_dll_path(self) -> str:
        path = self.dll_path.get().strip() or DEFAULT_DLL_NAME
        return os.path.abspath(path)

    def connect(self) -> None:
        if platform.system() != "Windows":
            messagebox.showerror("Unsupported OS", "This simulator requires Windows because it loads ECanVci.dll.")
            return
        try:
            config = DeviceConfig(
                dll_path=self.resolve_dll_path(),
                device_type=DEFAULT_DEVICE_TYPE,
                device_index=DEFAULT_DEVICE_INDEX,
                can_index=DEFAULT_CAN_INDEX,
                timing0=TIMING0_250K,
                timing1=TIMING1_250K,
            )
            self.device = USBCANDevice(config)
            self.device.open()
            self.is_connected = True
            self.status_text.set(f"Status: Connected ({config.dll_path})")
        except Exception as exc:
            self.device = None
            self.is_connected = False
            messagebox.showerror("Connection error", str(exc))
        self._update_button_states()

    def disconnect(self) -> None:
        self.stop_periodic()
        if self.device:
            try:
                self.device.close()
            except Exception:
                pass
        self.device = None
        self.is_connected = False
        self.status_text.set("Status: Disconnected")
        self._update_button_states()

    def _transmit_current_frames(self) -> None:
        if not self.device:
            return
        for frame_id, data in self.current_frames():
            self.device.send(frame_id, data)

    def start_periodic(self) -> None:
        if not self.device or self.send_job is not None:
            return
        self._schedule_send()
        self._update_button_states()

    def stop_periodic(self) -> None:
        if self.send_job is not None:
            self.root.after_cancel(self.send_job)
            self.send_job = None
        self._update_button_states()

    def _schedule_send(self) -> None:
        try:
            interval = max(10, int(self.interval_ms.get()))
        except tk.TclError:
            interval = 250
        self.send_job = self.root.after(interval, self._send_and_reschedule)

    def _send_and_reschedule(self) -> None:
        self._transmit_current_frames()
        self._schedule_send()

    def current_source_address(self) -> int:
        text = self.source_address.get().strip()
        if not text:
            return SOURCE_ADDRESS
        try:
            value = int(text, 0)
        except ValueError:
            return SOURCE_ADDRESS
        return max(0, min(value, 0xFF))

    def current_ccvs_frame(self) -> tuple[int, list[int]]:
        frame_id = j1939_id(PRIORITY_DEFAULT, PGN_CCVS, self.current_source_address())
        try:
            speed = float(self.speed_kmh.get().strip())
        except ValueError:
            speed = 0.0
        return frame_id, build_ccvs_data(speed)

    def current_etc2_frame(self) -> tuple[int, list[int]]:
        frame_id = j1939_id(PRIORITY_DEFAULT, PGN_ETC2, self.current_source_address())
        return frame_id, build_etc2_data(self.gear.get())

    def current_lc_frame(self) -> tuple[int, list[int]]:
        frame_id = j1939_id(PRIORITY_DEFAULT, PGN_LC, self.current_source_address())
        return frame_id, build_lc_data(self.turn_left.get(), self.turn_right.get())

    def current_dc1_frame(self) -> tuple[int, list[int]]:
        frame_id = j1939_id(PRIORITY_DEFAULT, PGN_DC1, self.current_source_address())
        return frame_id, build_dc1_data(self.door_position.get())

    def current_class_iv_v_frame(self) -> tuple[int, list[int]]:
        frame_id = j1939_id(PRIORITY_DEFAULT, PGN_CLASS_IV_V_SWITCH, self.current_source_address())
        return frame_id, build_class_iv_v_switch_data(self.camera_view_command.get())

    def current_frames(self) -> list[tuple[int, list[int]]]:
        frames: list[tuple[int, list[int]]] = []
        if self.ccvs_enabled.get():
            frames.append(self.current_ccvs_frame())
        if self.etc2_enabled.get():
            frames.append(self.current_etc2_frame())
        if self.lc_enabled.get():
            frames.append(self.current_lc_frame())
        if self.dc1_enabled.get():
            frames.append(self.current_dc1_frame())
        if self.class_iv_v_enabled.get():
            frames.append(self.current_class_iv_v_frame())
        return frames

    def _format_preview_text(self, enabled: bool, frame_id: int, data: list[int]) -> str:
        if not enabled:
            return "Disabled"
        return f"0x{frame_id:08X} / {' '.join(f'{byte:02X}' for byte in data)}"

    def refresh_preview(self) -> None:
        ccvs_id, ccvs_data = self.current_ccvs_frame()
        etc2_id, etc2_data = self.current_etc2_frame()
        lc_id, lc_data = self.current_lc_frame()
        dc1_id, dc1_data = self.current_dc1_frame()
        class_iv_v_id, class_iv_v_data = self.current_class_iv_v_frame()
        self.ccvs_text.set(self._format_preview_text(self.ccvs_enabled.get(), ccvs_id, ccvs_data))
        self.etc2_text.set(self._format_preview_text(self.etc2_enabled.get(), etc2_id, etc2_data))
        self.lc_text.set(self._format_preview_text(self.lc_enabled.get(), lc_id, lc_data))
        self.dc1_text.set(self._format_preview_text(self.dc1_enabled.get(), dc1_id, dc1_data))
        self.class_iv_v_text.set(
            self._format_preview_text(self.class_iv_v_enabled.get(), class_iv_v_id, class_iv_v_data)
        )
        self.root.after(200, self.refresh_preview)

    def _update_button_states(self) -> None:
        if self.is_connected:
            self.connect_button.state(["disabled"])
            self.disconnect_button.state(["!disabled"])
            if self.send_job is None:
                self.start_button.state(["!disabled"])
                self.stop_button.state(["disabled"])
            else:
                self.start_button.state(["disabled"])
                self.stop_button.state(["!disabled"])
        else:
            self.connect_button.state(["!disabled"])
            self.disconnect_button.state(["disabled"])
            self.start_button.state(["disabled"])
            self.stop_button.state(["disabled"])


def main() -> None:
    root = tk.Tk()
    SimulatorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
