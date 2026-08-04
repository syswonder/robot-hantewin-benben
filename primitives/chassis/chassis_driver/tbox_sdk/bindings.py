import ctypes
import os
from pathlib import Path

DEFAULT_LIBRARY_PATH = Path(__file__).resolve().parents[0] / "lib" / "libtbox_sdk_cpp.so"

if __name__ == "__main__":
    print(Path(__file__).resolve().parents[0])
    raise SystemExit

from .sdk_types import (
    SDK_ACK_CAR_DEVICE_INFO_T,
    SDK_ACK_COMMUNICATION_T,
    SDK_ACK_HOIST_QUERY_T,
    SDK_ACK_MAP_ACQUISITION_STATUS_T,
    SDK_ACK_RELOCATION_T,
    SDK_ACK_TBOX_VERSION_T,
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
)

TBoxCallback = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_uint8, ctypes.c_void_p, ctypes.c_int)
LoginConnectionStatu = ctypes.CFUNCTYPE(None, ctypes.c_uint8)

class TBoxSDKLibrary:
    def __init__(self, library_path: str | os.PathLike[str] | None = None):
        path = Path(library_path or os.environ.get("TBOX_SDK_LIB") or DEFAULT_LIBRARY_PATH).expanduser()
        self.path = path.resolve()
        self.lib = ctypes.CDLL(str(self.path), mode=ctypes.RTLD_GLOBAL)
        self._configure_prototypes()

    def _configure_prototypes(self) -> None:
        lib = self.lib

        lib.tbox_sdk_init.argtypes = [ctypes.c_char_p, LoginConnectionStatu]
        lib.tbox_sdk_init.restype = None

        lib.tbox_sdk_register_callback.argtypes = [TBoxCallback]
        lib.tbox_sdk_register_callback.restype = None

        lib.tbox_sdk_relocation.argtypes = [
            ctypes.POINTER(SDK_RELOCATION_T),
            ctypes.POINTER(SDK_ACK_RELOCATION_T),
        ]
        lib.tbox_sdk_relocation.restype = ctypes.c_int

        lib.tbox_sdk_send_getCommuStatu.argtypes = [ctypes.POINTER(SDK_ACK_COMMUNICATION_T)]
        lib.tbox_sdk_send_getCommuStatu.restype = ctypes.c_int

        lib.tbox_sdk_map_acquisition.argtypes = [
            ctypes.c_int,
            ctypes.POINTER(SDK_ACK_MAP_ACQUISITION_STATUS_T),
        ]
        lib.tbox_sdk_map_acquisition.restype = ctypes.c_int

        lib.tbox_sdk_remote_control.argtypes = [ctypes.POINTER(SDK_REMOUTE_CONTROL_T)]
        lib.tbox_sdk_remote_control.restype = ctypes.c_int

        lib.tbox_sdk_rotation_control.argtypes = [ctypes.POINTER(SDK_ROTATION_CONTROL_T)]
        lib.tbox_sdk_rotation_control.restype = ctypes.c_int

        lib.tbox_sdk_hoist_control.argtypes = [ctypes.POINTER(SDK_HOIST_CONTROL_T)]
        lib.tbox_sdk_hoist_control.restype = ctypes.c_int

        lib.tbox_sdk_hoist_query.argtypes = [ctypes.c_uint8, ctypes.POINTER(SDK_ACK_HOIST_QUERY_T)]
        lib.tbox_sdk_hoist_query.restype = ctypes.c_int

        lib.tbox_sdk_hoist_set.argtypes = [ctypes.c_uint8]
        lib.tbox_sdk_hoist_set.restype = ctypes.c_int

        lib.tbox_sdk_power_manage.argtypes = [ctypes.POINTER(SDK_POWER_MANAGE_T)]
        lib.tbox_sdk_power_manage.restype = ctypes.c_int

        lib.tbox_sdk_get_tbox_version.argtypes = [ctypes.POINTER(SDK_ACK_TBOX_VERSION_T)]
        lib.tbox_sdk_get_tbox_version.restype = ctypes.c_int

        lib.tbox_sdk_send_getVinInfo.argtypes = [ctypes.POINTER(SDK_ACK_CAR_DEVICE_INFO_T)]
        lib.tbox_sdk_send_getVinInfo.restype = ctypes.c_int

        lib.tbox_sdk_position.argtypes = [ctypes.POINTER(SDK_POSITION_T)]
        lib.tbox_sdk_position.restype = ctypes.c_int

        lib.tbox_sdk_task_control.argtypes = [ctypes.c_uint8]
        lib.tbox_sdk_task_control.restype = ctypes.c_int

        lib.tbox_sdk_go_home.argtypes = []
        lib.tbox_sdk_go_home.restype = ctypes.c_int

        lib.tbox_sdk_task_distribution.argtypes = [ctypes.POINTER(SDK_TASK_DISTRIBUTION_T)]
        lib.tbox_sdk_task_distribution.restype = ctypes.c_int

        lib.tbox_sdk_go_start.argtypes = []
        lib.tbox_sdk_go_start.restype = ctypes.c_int

        lib.tbox_sdk_clear_fault.argtypes = []
        lib.tbox_sdk_clear_fault.restype = ctypes.c_int

        lib.tbox_sdk_communication_control.argtypes = [ctypes.POINTER(SDK_COMMUNICATION_CONTROL_T)]
        lib.tbox_sdk_communication_control.restype = ctypes.c_int

        lib.tbox_sdk_EAITask_statu_report.argtypes = [ctypes.POINTER(SDK_EAI_TASK_STATU_T)]
        lib.tbox_sdk_EAITask_statu_report.restype = ctypes.c_int

        lib.tbox_sdk_dynamic_param_get.argtypes = [ctypes.POINTER(SDK_DYNAMIC_PARAM_LIST_T)]
        lib.tbox_sdk_dynamic_param_get.restype = ctypes.c_int

        lib.tbox_sdk_dynamic_param_set.argtypes = [ctypes.POINTER(SDK_DYNAMIC_PARAM_LIST_T)]
        lib.tbox_sdk_dynamic_param_set.restype = ctypes.c_int

        lib.tbox_sdk_brake_control.argtypes = [ctypes.c_uint8]
        lib.tbox_sdk_brake_control.restype = ctypes.c_int


__all__ = [
    "DEFAULT_LIBRARY_PATH",
    "LoginConnectionStatu",
    "TBoxCallback",
    "TBoxSDKLibrary",
]
