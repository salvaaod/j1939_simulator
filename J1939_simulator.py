import ctypes
import os
import platform
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk

try:
    import winreg
except ImportError:
    winreg = None

USBCAN_II = 4
DEFAULT_DEVICE_TYPE = USBCAN_II
DEFAULT_DEVICE_INDEX = 0
DEFAULT_CAN_INDEX = 0
DEFAULT_DLL_NAME = "ECanVci.dll"
TIMING0_250K = 0x01
TIMING1_250K = 0x1C

PGN_CCVS = 0x00FEF1  # SPN 84: wheel-based vehicle speed
PGN_ETC2 = 0x00F005  # SPN 523: transmission current gear
PGN_OEL = 0x00FDCC  # SPN 2876: operator external light controls (turn signals)

PRIORITY_DEFAULT = 6
PRIORITY_OEL = 3
SOURCE_ADDRESS = 0x00
DEFAULT_INTERVAL_MS = 250
REGISTRY_SUBKEY = r"Software\J1939Simulator"
REG_VALUE_WINDOW_GEOMETRY = "window_geometry"
REG_VALUE_SPEED = "speed_kmh"
REG_VALUE_SOURCE_ADDRESS = "source_address"
REG_VALUE_SEND_INTERVAL_MS = "send_interval_ms"


def j1939_id(priority: int, pgn: int, source_address: int) -> int:
    return ((priority & 0x7) << 26) | ((pgn & 0x3FFFF) << 8) | (source_address & 0xFF)


def build_ccvs_data(speed_kmh: float, brake_active: bool) -> list[int]:
    raw_speed = int(max(0.0, min(float(speed_kmh), 250.996)) * 256)
    data = [0x00] * 8
    data[1] = raw_speed & 0xFF
    data[2] = (raw_speed >> 8) & 0xFF
    # SPN 597: Brake switch, byte 4 bits 5-6
    # 0x00: brake released, 0x01: brake depressed
    brake_switch = 0b01 if brake_active else 0b00
    data[3] = brake_switch << 4
    return data


def build_etc2_data(gear_choice: str) -> list[int]:
    gear_map = {
        "Reverse": 124,  # -1 + 125 offset
        "Neutral": 125,  # 0 + 125 offset
        "Drive": 126,  # 1 + 125 offset
        "Park": 251,  # per J1939 SPN 523 definition
    }
    data = [0x00] * 8
    data[3] = gear_map.get(gear_choice, 125)
    return data


def build_lc_data(left_active: bool, right_active: bool) -> list[int]:
    data = [0x00] * 8
    # OEL / SPN 2876 in this project: byte 2 bit 1 = left, bit 2 = right
    left_cmd = 0x01 if left_active else 0x00
    right_cmd = 0x02 if right_active else 0x00
    data[1] = left_cmd | right_cmd
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
        self.root.title("J1939/FMS Simulator")
        self.device: USBCANDevice | None = None
        self.send_job: str | None = None
        self.is_connected = False
        self._loading_settings = False
        self._build_ui()
        self._load_registry_settings()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

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

        self.brake_active = tk.BooleanVar(value=False)
        ttk.Checkbutton(main, text="Brake active", variable=self.brake_active).grid(row=3, column=0, sticky="w")

        self.always_on_top = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            main,
            text="Always on top",
            variable=self.always_on_top,
            command=self._update_always_on_top,
        ).grid(row=3, column=1, sticky="w")

        ttk.Label(main, text="Source address (0-255 or 0x00-0xFF)").grid(row=4, column=0, sticky="w")
        self.source_address = tk.StringVar(value=f"0x{SOURCE_ADDRESS:02X}")
        ttk.Entry(main, textvariable=self.source_address).grid(row=4, column=1, sticky="ew")

        ttk.Label(main, text="Gear (SPN 523)").grid(row=5, column=0, sticky="w")
        self.gear = tk.StringVar(value="Neutral")
        ttk.Combobox(main, textvariable=self.gear, values=["Reverse", "Neutral", "Drive", "Park"], state="readonly").grid(
            row=5, column=1, sticky="ew"
        )

        self.turn_left = tk.BooleanVar(value=False)
        self.turn_right = tk.BooleanVar(value=False)
        ttk.Checkbutton(main, text="Left turn active", variable=self.turn_left).grid(row=6, column=0, sticky="w")
        ttk.Checkbutton(main, text="Right turn active", variable=self.turn_right).grid(row=6, column=1, sticky="w")

        ttk.Label(main, text="Send interval (ms)").grid(row=7, column=0, sticky="w")
        self.interval_ms = tk.IntVar(value=DEFAULT_INTERVAL_MS)
        ttk.Entry(main, textvariable=self.interval_ms).grid(row=7, column=1, sticky="ew")

        ttk.Label(main, text="CCVS ID/Data").grid(row=8, column=0, sticky="w")
        self.ccvs_text = tk.StringVar(value="")
        ttk.Label(main, textvariable=self.ccvs_text).grid(row=8, column=1, sticky="w")

        ttk.Label(main, text="ETC2 ID/Data").grid(row=9, column=0, sticky="w")
        self.etc2_text = tk.StringVar(value="")
        ttk.Label(main, textvariable=self.etc2_text).grid(row=9, column=1, sticky="w")

        ttk.Label(main, text="OEL ID/Data").grid(row=10, column=0, sticky="w")
        self.lc_text = tk.StringVar(value="")
        ttk.Label(main, textvariable=self.lc_text).grid(row=10, column=1, sticky="w")

        buttons = ttk.Frame(main)
        buttons.grid(row=11, column=0, columnspan=2, pady=8, sticky="ew")
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

        self.speed_kmh.trace_add("write", self._on_setting_changed)
        self.source_address.trace_add("write", self._on_setting_changed)
        self.interval_ms.trace_add("write", self._on_setting_changed)

    def _save_registry_settings(self) -> None:
        if self._loading_settings or winreg is None:
            return
        try:
            with winreg.CreateKey(winreg.HKEY_CURRENT_USER, REGISTRY_SUBKEY) as key:
                geometry = self.root.geometry()
                winreg.SetValueEx(key, REG_VALUE_WINDOW_GEOMETRY, 0, winreg.REG_SZ, geometry)
                winreg.SetValueEx(key, REG_VALUE_SPEED, 0, winreg.REG_SZ, self.speed_kmh.get())
                winreg.SetValueEx(key, REG_VALUE_SOURCE_ADDRESS, 0, winreg.REG_SZ, self.source_address.get())
                winreg.SetValueEx(key, REG_VALUE_SEND_INTERVAL_MS, 0, winreg.REG_DWORD, max(0, int(self.interval_ms.get())))
        except Exception:
            # Ignore registry write failures so the simulator can continue working.
            return

    def _load_registry_settings(self) -> None:
        if winreg is None:
            return
        self._loading_settings = True
        try:
            with winreg.OpenKey(winreg.HKEY_CURRENT_USER, REGISTRY_SUBKEY, 0, winreg.KEY_READ) as key:
                geometry = self._read_registry_value(key, REG_VALUE_WINDOW_GEOMETRY, None)
                speed = self._read_registry_value(key, REG_VALUE_SPEED, None)
                source_address = self._read_registry_value(key, REG_VALUE_SOURCE_ADDRESS, None)
                interval_ms = self._read_registry_value(key, REG_VALUE_SEND_INTERVAL_MS, None)

                if isinstance(speed, str) and speed.strip():
                    self.speed_kmh.set(speed)
                if isinstance(source_address, str) and source_address.strip():
                    self.source_address.set(source_address)
                if isinstance(interval_ms, int):
                    self.interval_ms.set(max(10, interval_ms))
                if isinstance(geometry, str) and geometry.strip():
                    self.root.geometry(geometry)
        except FileNotFoundError:
            pass
        except Exception:
            pass
        finally:
            self._loading_settings = False

    def _read_registry_value(self, key: "winreg.HKEYType", name: str, default: object) -> object:
        try:
            value, _value_type = winreg.QueryValueEx(key, name)
            return value
        except FileNotFoundError:
            return default
        except OSError:
            return default

    def _on_setting_changed(self, *_args: object) -> None:
        self._save_registry_settings()

    def _on_close(self) -> None:
        self._save_registry_settings()
        self.disconnect()
        self.root.destroy()

    def _update_always_on_top(self) -> None:
        self.root.attributes("-topmost", self.always_on_top.get())

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
            interval = DEFAULT_INTERVAL_MS
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
        return frame_id, build_ccvs_data(speed, self.brake_active.get())

    def current_etc2_frame(self) -> tuple[int, list[int]]:
        frame_id = j1939_id(PRIORITY_DEFAULT, PGN_ETC2, self.current_source_address())
        return frame_id, build_etc2_data(self.gear.get())

    def current_lc_frame(self) -> tuple[int, list[int]]:
        frame_id = j1939_id(PRIORITY_OEL, PGN_OEL, self.current_source_address())
        return frame_id, build_lc_data(self.turn_left.get(), self.turn_right.get())

    def current_frames(self) -> list[tuple[int, list[int]]]:
        return [self.current_ccvs_frame(), self.current_etc2_frame(), self.current_lc_frame()]

    def _format_preview_text(self, frame_id: int, data: list[int]) -> str:
        return f"0x{frame_id:08X} / {' '.join(f'{byte:02X}' for byte in data)}"

    def refresh_preview(self) -> None:
        ccvs_id, ccvs_data = self.current_ccvs_frame()
        etc2_id, etc2_data = self.current_etc2_frame()
        lc_id, lc_data = self.current_lc_frame()
        self.ccvs_text.set(self._format_preview_text(ccvs_id, ccvs_data))
        self.etc2_text.set(self._format_preview_text(etc2_id, etc2_data))
        self.lc_text.set(self._format_preview_text(lc_id, lc_data))
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
