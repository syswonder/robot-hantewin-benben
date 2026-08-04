from __future__ import annotations

import ctypes
import queue
import time
from collections.abc import Callable, Iterable, Mapping, Sequence
from typing import Any

from .bindings import LoginConnectionStatu, TBoxCallback, TBoxSDKLibrary
from .constants import (
    BrakeControl,
    CommunicationControlCommand,
    HoistSetCommand,
    LoginStatus,
    MapAcquisitionCommand,
    PacketCommand,
    RelocationType,
    RotationType,
    TaskControl,
    TaskType,
)
from .sdk_types import (
    SDK_ACK_CAR_DEVICE_INFO_T,
    SDK_ACK_COMMUNICATION_T,
    SDK_ACK_HOIST_QUERY_T,
    SDK_ACK_MAP_ACQUISITION_STATUS_T,
    SDK_ACK_RELOCATION_T,
    SDK_ACK_TBOX_VERSION_T,
    SDK_BACK_CHARGING_STATU_T,
    SDK_BACK_CHASSIS_STATIC_INFO_T,
    SDK_BACK_CUR_POSE_T,
    SDK_BACK_EAI_TASK_DELIVER_T,
    SDK_BACK_TASK_STATU_T,
    SDK_BACK_TBOX_ERROR_DISPLAY_T,
    SDK_BACK_UNDERPAN_STATE_T,
    SDK_COMMUNICATION_CONTROL_T,
    SDK_DYNAMIC_PARAM_LIST_T,
    SDK_EAI_TASK_STATU_T,
    SDK_HOIST_CONTROL_T,
    SDK_POSITION_T,
    SDK_POWER_MANAGE_T,
    SDK_RELOCATION_T,
    SDK_REMOUTE_CONTROL_T,
    SDK_ROTATION_CONTROL_T,
    SDK_TASK_DISTRIBUTION_T,
    UINT8_MAX,
)

StateHandler = Callable[[int, bytes, Any | None], None]
LoginHandler = Callable[[int], None]

CALLBACK_STRUCTS: dict[int, type[ctypes.Structure]] = {
    int(PacketCommand.PACKET_CMD_TBOX_ERROR_DISPLAY): SDK_BACK_TBOX_ERROR_DISPLAY_T,
    int(PacketCommand.PACKET_CMD_CHARGING_STATE): SDK_BACK_CHARGING_STATU_T,
    int(PacketCommand.PACKET_CMD_TASK_STATE): SDK_BACK_TASK_STATU_T,
    int(PacketCommand.PACKET_CMD_UNDERPAN_STATE): SDK_BACK_UNDERPAN_STATE_T,
    int(PacketCommand.PACKET_CMD_CHASSIS_STATIC_INFO): SDK_BACK_CHASSIS_STATIC_INFO_T,
    int(PacketCommand.PACKET_CMD_EMBODIED_INTELLIGENCE_TASK): SDK_BACK_EAI_TASK_DELIVER_T,
    int(PacketCommand.PACKET_CMD_POSE_INFO): SDK_BACK_CUR_POSE_T,
}


class TBoxSDKError(RuntimeError):
    def __init__(self, function_name: str, code: int):
        super().__init__(f"{function_name} failed with return code {code}")
        self.function_name = function_name
        self.code = code


class TBoxClient:
    def __init__(
        self,
        library_path: str | None = None,
        library: Any | None = None,
        encoding: str = "utf-8",
        ready_statuses: Iterable[int] | None = None,
        sleep_func: Callable[[float], None] = time.sleep,
        monotonic_func: Callable[[], float] = time.monotonic,
    ):
        self.encoding = encoding
        self._library_wrapper = None if library is not None else TBoxSDKLibrary(library_path)
        self._lib = library if library is not None else self._library_wrapper.lib
        self._ready_statuses = set(ready_statuses or (int(LoginStatus.AUTH_SUCCESS),))
        self._sleep = sleep_func
        self._monotonic = monotonic_func
        self._state_handler: StateHandler | None = None
        self._login_handler: LoginHandler | None = None
        self._state_cb_c: ctypes._CFuncPtr | None = None
        self._login_cb_c: ctypes._CFuncPtr | None = None
        self._token_bytes: bytes | None = None
        self._event_queue: queue.Queue[tuple[int, bytes, Any | None]] = queue.Queue()
        self.login_status_history: list[int] = []
        self.last_login_status: int | None = None
        self.last_task_status: SDK_BACK_TASK_STATU_T | None = None
        self.last_task_type: int | None = None
        self._ready = False

    @property
    def event_queue(self) -> queue.Queue[tuple[int, bytes, Any | None]]:
        return self._event_queue

    def register_state_callback(self, handler: StateHandler | None = None) -> None:
        self._state_handler = handler
        self._ensure_state_callback_registered()

    def register_login_callback(self, handler: LoginHandler | None = None) -> None:
        self._login_handler = handler

    def initialize(self, token: str) -> None:
        if not token:
            raise ValueError("token must not be empty")
        self._ensure_state_callback_registered()
        self._login_cb_c = LoginConnectionStatu(self._on_login_status)
        self._ready = False
        self._token_bytes = token.encode(self.encoding)
        self._lib.tbox_sdk_init(ctypes.c_char_p(self._token_bytes), self._login_cb_c)

    def wait_until_ready(self, timeout: float = 30.0, poll_interval: float = 0.05) -> bool:
        deadline = self._monotonic() + timeout
        while self._monotonic() < deadline:
            if self._ready:
                return True
            self._sleep(min(poll_interval, max(0.0, deadline - self._monotonic())))
        return self._ready

    def remote_control(
        self,
        linear_speed: float,
        angular_speed: float = 0.0,
        serious_fault_can_move: bool = False,
    ) -> None:
        self.tbox_sdk_remote_control(linear_speed, angular_speed, serious_fault_can_move)

    def tbox_sdk_remote_control(
        self,
        linear_speed: float,
        angular_speed: float = 0.0,
        serious_fault_can_move: bool = False,
    ) -> int:
        self._validate_range("linear_speed", linear_speed, -1.0, 1.0)
        self._validate_range("angular_speed", angular_speed, -1.0, 1.0)
        remote = SDK_REMOUTE_CONTROL_T(
            ctypes.c_float(linear_speed),
            ctypes.c_float(angular_speed),
            ctypes.c_uint8(1 if serious_fault_can_move else 0),
        )
        code = self._lib.tbox_sdk_remote_control(ctypes.byref(remote))
        self._check(code, "tbox_sdk_remote_control")
        return code 
        

    def stop(self) -> None:
        self.remote_control(0.0, 0.0, False)

    def enter_parallel_driving(
        self,
        refresh_count: int = 3,
        command_hz: float = 10.0,
        wait_confirm_timeout: float = 0.0,
    ) -> bool:
        if refresh_count <= 0:
            raise ValueError("refresh_count must be positive")
        if command_hz <= 0:
            raise ValueError("command_hz must be positive")
        interval = 1.0 / command_hz
        for index in range(refresh_count):
            self.remote_control(0.0, 0.0, False)
            if index != refresh_count - 1:
                self._sleep(interval)
        return self.wait_for_task_type(TaskType.PARALLEL_DRIVING, wait_confirm_timeout) if wait_confirm_timeout > 0 else False

    def wait_for_task_type(
        self,
        task_type: int | TaskType,
        timeout: float = 3.0,
        poll_interval: float = 0.05,
    ) -> bool:
        expected = int(task_type)
        deadline = self._monotonic() + timeout
        while self._monotonic() < deadline:
            if self.last_task_type == expected:
                return True
            self._sleep(min(poll_interval, max(0.0, deadline - self._monotonic())))
        return self.last_task_type == expected

    def prepare_remote_motion(
        self,
        refresh_count: int = 3,
        command_hz: float = 10.0,
        wait_parallel_timeout: float = 0.0,
        release_brake_first: bool = False,
    ) -> bool:
        if release_brake_first:
            self.brake_control(BrakeControl.PULL)
        confirmed = self.enter_parallel_driving(
            refresh_count=refresh_count,
            command_hz=command_hz,
            wait_confirm_timeout=wait_parallel_timeout,
        )
        if not release_brake_first:
            self.brake_control(BrakeControl.PULL)
        return confirmed

    def drive_forward_distance(
        self,
        distance_m: float = 1.0,
        speed_mps: float = 0.2,
        command_hz: float = 10.0,
        serious_fault_can_move: bool = False,
        prepare: bool = True,
        wait_parallel_timeout: float = 0.0,
        release_brake_first: bool = False,
    ) -> None:
        if distance_m <= 0:
            raise ValueError("distance_m must be positive")
        self._validate_range("speed_mps", speed_mps, 0.001, 1.0)
        if command_hz <= 0:
            raise ValueError("command_hz must be positive")
        duration = distance_m / speed_mps
        interval = 1.0 / command_hz
        if prepare:
            self.prepare_remote_motion(
                command_hz=command_hz,
                wait_parallel_timeout=wait_parallel_timeout,
                release_brake_first=release_brake_first,
            )
        deadline = self._monotonic() + duration
        try:
            while self._monotonic() < deadline:
                self.remote_control(speed_mps, 0.0, serious_fault_can_move)
                remaining = deadline - self._monotonic()
                if remaining > 0:
                    self._sleep(min(interval, remaining))
        finally:
            self.stop()
            self.stop()
            self.stop()

    def rotation_control(self, rotation_type: int | RotationType, rotation_angle: float, speed_ratio: float) -> None:
        self.tbox_sdk_rotation_control(rotation_type, rotation_angle, speed_ratio)

    def tbox_sdk_rotation_control(
        self,
        rotation_type: int | RotationType,
        rotation_angle: float,
        speed_ratio: float,
    ) -> None:
        rotation = SDK_ROTATION_CONTROL_T(int(rotation_type), ctypes.c_float(rotation_angle), ctypes.c_float(speed_ratio))
        self._check(self._lib.tbox_sdk_rotation_control(ctypes.byref(rotation)), "tbox_sdk_rotation_control")

    def hoist_control(self, ctrl: int, height: int, speed: int) -> None:
        self._validate_range("speed", speed, 0, 255)
        hoist = SDK_HOIST_CONTROL_T(int(ctrl), int(height), int(speed))
        self._check(self._lib.tbox_sdk_hoist_control(ctypes.byref(hoist)), "tbox_sdk_hoist_control")

    def hoist_query(self, cmd: int = 0) -> SDK_ACK_HOIST_QUERY_T:
        result = SDK_ACK_HOIST_QUERY_T()
        self._check(self._lib.tbox_sdk_hoist_query(int(cmd), ctypes.byref(result)), "tbox_sdk_hoist_query")
        return result

    def hoist_set(self, command: int | HoistSetCommand) -> None:
        self._check(self._lib.tbox_sdk_hoist_set(int(command)), "tbox_sdk_hoist_set")

    def task_distribution(
        self,
        x: float,
        y: float,
        yaw: float,
        map_name: str = "",
        task_version: int = 2,
        speed_ratio: int = 3,
        move_mode: int = 0,
        floor: int = 0,
        elevator_selection: int = 0,
        hoist_type: int = 0,
    ) -> None:
        task = SDK_TASK_DISTRIBUTION_T()
        task.task_version = int(task_version)
        task.x = float(x)
        task.y = float(y)
        task.yaw = float(yaw)
        task.speed_ratio = int(speed_ratio)
        task.move_mode = int(move_mode)
        task.floor = int(floor)
        task.elevator_selection = int(elevator_selection)
        task.map_name = self._encode_fixed_string(map_name, UINT8_MAX, "map_name")
        task.hoist_type = int(hoist_type)
        self._check(self._lib.tbox_sdk_task_distribution(ctypes.byref(task)), "tbox_sdk_task_distribution")

    def task_control(self, ctrl: int | TaskControl) -> None:
        self._check(self._lib.tbox_sdk_task_control(int(ctrl)), "tbox_sdk_task_control")

    def go_home(self) -> None:
        self._check(self._lib.tbox_sdk_go_home(), "tbox_sdk_go_home")

    def go_start(self) -> None:
        self._check(self._lib.tbox_sdk_go_start(), "tbox_sdk_go_start")

    def clear_fault(self) -> None:
        self._check(self._lib.tbox_sdk_clear_fault(), "tbox_sdk_clear_fault")

    def brake_control(self, brake_ctrl: int | BrakeControl) -> None:
        self._check(self._lib.tbox_sdk_brake_control(int(brake_ctrl)), "tbox_sdk_brake_control")

    def relocation(
        self,
        relocation_type: int | RelocationType,
        threshold: float = 0.4,
        left_top_x: float = 0.0,
        left_top_y: float = 0.0,
        yaw: float = 0.0,
    ) -> SDK_ACK_RELOCATION_T:
        relocation = SDK_RELOCATION_T(int(relocation_type), float(threshold), float(left_top_x), float(left_top_y), float(yaw))
        result = SDK_ACK_RELOCATION_T()
        self._check(self._lib.tbox_sdk_relocation(ctypes.byref(relocation), ctypes.byref(result)), "tbox_sdk_relocation")
        return result

    def map_acquisition(self, cmd: int | MapAcquisitionCommand) -> SDK_ACK_MAP_ACQUISITION_STATUS_T:
        result = SDK_ACK_MAP_ACQUISITION_STATUS_T()
        self._check(self._lib.tbox_sdk_map_acquisition(int(cmd), ctypes.byref(result)), "tbox_sdk_map_acquisition")
        return result

    def power_manage(
        self,
        auto_charging_switch: int,
        charging_threshold_value: int,
        working_threshold_value: int,
        charging_floor: int,
        charging_mode: int,
    ) -> None:
        power = SDK_POWER_MANAGE_T(
            int(auto_charging_switch),
            int(charging_threshold_value),
            int(working_threshold_value),
            int(charging_floor),
            int(charging_mode),
        )
        self._check(self._lib.tbox_sdk_power_manage(ctypes.byref(power)), "tbox_sdk_power_manage")

    def get_tbox_version(self) -> dict[str, str]:
        result = SDK_ACK_TBOX_VERSION_T()
        self._check(self._lib.tbox_sdk_get_tbox_version(ctypes.byref(result)), "tbox_sdk_get_tbox_version")
        return {
            name: self._decode_c_string(getattr(result, name))
            for name, _ in result._fields_
        }

    def get_vin_info(self) -> dict[str, str]:
        result = SDK_ACK_CAR_DEVICE_INFO_T()
        self._check(self._lib.tbox_sdk_send_getVinInfo(ctypes.byref(result)), "tbox_sdk_send_getVinInfo")
        return {
            "vin": self._decode_c_string(result.vin),
            "iccid": self._decode_c_string(result.iccid),
        }

    def get_communication_status(self) -> dict[str, str | int]:
        result = SDK_ACK_COMMUNICATION_T()
        self._check(self._lib.tbox_sdk_send_getCommuStatu(ctypes.byref(result)), "tbox_sdk_send_getCommuStatu")
        return {
            "communication_control": int(result.communication_control),
            "SSID": self._decode_c_string(result.SSID),
            "Wifi": self._decode_c_string(result.Wifi),
        }

    def communication_control(
        self,
        command: int | CommunicationControlCommand,
        ssid: str = "",
        password: str = "",
    ) -> None:
        ctrl = SDK_COMMUNICATION_CONTROL_T()
        ctrl.communication_control = int(command)
        ctrl.SSID = self._encode_fixed_string(ssid, UINT8_MAX, "SSID")
        ctrl.pass_wd = self._encode_fixed_string(password, UINT8_MAX, "pass_wd")
        self._check(self._lib.tbox_sdk_communication_control(ctypes.byref(ctrl)), "tbox_sdk_communication_control")

    def position(self, location_status: int, longitude: float, latitude: float, direction: int) -> None:
        pos = SDK_POSITION_T(int(location_status), float(longitude), float(latitude), int(direction))
        self._check(self._lib.tbox_sdk_position(ctypes.byref(pos)), "tbox_sdk_position")

    def eai_task_status_report(self, task_uuid: str, status: int) -> None:
        report = SDK_EAI_TASK_STATU_T()
        report.task_uuid = self._encode_fixed_string(task_uuid, 37, "task_uuid")
        report.status = int(status)
        self._check(self._lib.tbox_sdk_EAITask_statu_report(ctypes.byref(report)), "tbox_sdk_EAITask_statu_report")

    def dynamic_param_set(self, params: Mapping[str, str]) -> None:
        if len(params) > 255:
            raise ValueError("params must contain at most 255 items")
        names = [str(name).encode(self.encoding) for name in params.keys()]
        values = [str(value).encode(self.encoding) for value in params.values()]
        name_array = (ctypes.c_char_p * len(names))(*names)
        value_array = (ctypes.c_char_p * len(values))(*values)
        param_list = SDK_DYNAMIC_PARAM_LIST_T(
            len(names),
            ctypes.cast(name_array, ctypes.POINTER(ctypes.c_char_p)),
            ctypes.cast(value_array, ctypes.POINTER(ctypes.c_char_p)),
        )
        self._check(self._lib.tbox_sdk_dynamic_param_set(ctypes.byref(param_list)), "tbox_sdk_dynamic_param_set")

    def dynamic_param_get_raw(self, param_names: Sequence[str]) -> dict[str, str | None]:
        if len(param_names) > 255:
            raise ValueError("param_names must contain at most 255 items")
        names = [str(name).encode(self.encoding) for name in param_names]
        values = [None for _ in names]
        name_array = (ctypes.c_char_p * len(names))(*names)
        value_array = (ctypes.c_char_p * len(values))(*values)
        param_list = SDK_DYNAMIC_PARAM_LIST_T(
            len(names),
            ctypes.cast(name_array, ctypes.POINTER(ctypes.c_char_p)),
            ctypes.cast(value_array, ctypes.POINTER(ctypes.c_char_p)),
        )
        self._check(self._lib.tbox_sdk_dynamic_param_get(ctypes.byref(param_list)), "tbox_sdk_dynamic_param_get")
        return {
            name.decode(self.encoding, errors="replace"): self._decode_nullable_c_string(value_array[index])
            for index, name in enumerate(names)
        }

    def _ensure_state_callback_registered(self) -> None:
        if self._state_cb_c is not None:
            return
        self._state_cb_c = TBoxCallback(self._on_state_callback)
        self._lib.tbox_sdk_register_callback(self._state_cb_c)

    def _on_login_status(self, status: int) -> None:
        try:
            status_int = int(status)
            self.last_login_status = status_int
            self.login_status_history.append(status_int)
            if status_int in self._ready_statuses:
                self._ready = True
            if self._login_handler is not None:
                self._login_handler(status_int)
        except Exception:
            pass

    def _on_state_callback(self, cmd: int, data: int, data_len: int) -> int:
        try:
            payload = ctypes.string_at(data, data_len) if data and data_len > 0 else b""
            parsed = self._parse_payload(cmd, payload)
            if isinstance(parsed, SDK_BACK_TASK_STATU_T):
                self.last_task_status = parsed
                self.last_task_type = int(parsed.task_type)
            event = (int(cmd), payload, parsed)
            self._event_queue.put(event)
            if self._state_handler is not None:
                self._state_handler(*event)
            return 0
        except Exception:
            return -1

    def _parse_payload(self, cmd: int, payload: bytes) -> Any | None:
        struct_type = CALLBACK_STRUCTS.get(int(cmd))
        if struct_type is None or len(payload) < ctypes.sizeof(struct_type):
            return None
        return struct_type.from_buffer_copy(payload[: ctypes.sizeof(struct_type)])

    def _check(self, code: int, function_name: str) -> None:
        if int(code) != 0:
            raise TBoxSDKError(function_name, int(code))

    def _encode_fixed_string(self, value: str, size: int, field_name: str) -> bytes:
        data = value.encode(self.encoding)
        if len(data) >= size:
            raise ValueError(f"{field_name} must be shorter than {size} bytes after encoding")
        return data

    def _decode_c_string(self, value: Any) -> str:
        raw = value if isinstance(value, bytes) else bytes(value)
        return raw.split(b"\0", 1)[0].decode(self.encoding, errors="replace")

    def _decode_nullable_c_string(self, value: bytes | None) -> str | None:
        if value is None:
            return None
        return self._decode_c_string(value)

    @staticmethod
    def _validate_range(name: str, value: float, minimum: float, maximum: float) -> None:
        if value < minimum or value > maximum:
            raise ValueError(f"{name} must be between {minimum} and {maximum}")


__all__ = ["CALLBACK_STRUCTS", "TBoxClient", "TBoxSDKError"]
