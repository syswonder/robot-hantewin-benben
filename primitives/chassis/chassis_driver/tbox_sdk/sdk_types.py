import ctypes

UINT8_MAX = 255
UINT16_MAX = 65535


class PackedStructure(ctypes.Structure):
    _pack_ = 1


class SDK_RELOCATION_T(PackedStructure):
    _fields_ = [
        ("type", ctypes.c_uint8),
        ("threshold", ctypes.c_float),
        ("left_top_x", ctypes.c_float),
        ("left_top_y", ctypes.c_float),
        ("yaw", ctypes.c_float),
    ]


class SDK_ACK_RELOCATION_T(PackedStructure):
    _fields_ = [
        ("result", ctypes.c_uint8),
        ("score", ctypes.c_uint8),
        ("failure_reason", ctypes.c_char * UINT8_MAX),
    ]


class SDK_ACK_COMMUNICATION_T(PackedStructure):
    _fields_ = [
        ("communication_control", ctypes.c_uint8),
        ("SSID", ctypes.c_char * UINT8_MAX),
        ("Wifi", ctypes.c_char * UINT8_MAX),
    ]


class SDK_ACK_MAP_ACQUISITION_STATUS_T(PackedStructure):
    _fields_ = [
        ("status", ctypes.c_uint8),
        ("substate", ctypes.c_uint8),
    ]


class SDK_REMOUTE_CONTROL_T(PackedStructure):
    _fields_ = [
        ("linear_speed", ctypes.c_float),
        ("angular_speed", ctypes.c_float),
        ("seriousFaultCanMove", ctypes.c_uint8),
    ]


class SDK_ROTATION_CONTROL_T(PackedStructure):
    _fields_ = [
        ("type", ctypes.c_uint8),
        ("rotation_angle", ctypes.c_float),
        ("speed_ratio", ctypes.c_float),
    ]


class SDK_HOIST_CONTROL_T(PackedStructure):
    _fields_ = [
        ("ctrl", ctypes.c_uint8),
        ("height", ctypes.c_int32),
        ("speed", ctypes.c_uint8),
    ]


class SDK_ACK_HOIST_QUERY_T(PackedStructure):
    _fields_ = [
        ("height", ctypes.c_int32),
        ("ratio", ctypes.c_uint8),
        ("errcode", ctypes.c_uint16),
    ]


class SDK_HOIST_SET_T(PackedStructure):
    _fields_ = [("ctrl", ctypes.c_uint8)]


class SDK_POWER_MANAGE_T(PackedStructure):
    _fields_ = [
        ("auto_charging_switch", ctypes.c_uint8),
        ("charging_threshold_value", ctypes.c_uint8),
        ("working_threshold_value", ctypes.c_uint8),
        ("charging_floor", ctypes.c_uint8),
        ("charging_mode", ctypes.c_uint8),
    ]


class SDK_ACK_TBOX_VERSION_T(PackedStructure):
    _fields_ = [
        ("ficm_software_version", ctypes.c_uint8 * UINT8_MAX),
        ("hbox_software_version", ctypes.c_uint8 * UINT8_MAX),
        ("pms_software_version", ctypes.c_uint8 * UINT8_MAX),
        ("pcm_software_version", ctypes.c_uint8 * UINT8_MAX),
        ("ssm_software_version", ctypes.c_uint8 * UINT8_MAX),
        ("sdk_software_version", ctypes.c_uint8 * UINT8_MAX),
        ("tboxsdk_software_version", ctypes.c_uint8 * UINT8_MAX),
        ("sbox_software_version", ctypes.c_uint8 * UINT8_MAX),
        ("tbox_software_version", ctypes.c_uint8 * UINT8_MAX),
        ("kernel_software_version", ctypes.c_uint8 * UINT8_MAX),
        ("rootfst_software_version", ctypes.c_uint8 * UINT8_MAX),
        ("hvcu_software_version", ctypes.c_uint8 * UINT8_MAX),
    ]


class SDK_ACK_CAR_DEVICE_INFO_T(PackedStructure):
    _fields_ = [
        ("vin", ctypes.c_char * 18),
        ("iccid", ctypes.c_char * 21),
    ]


class SDK_POSITION_T(PackedStructure):
    _fields_ = [
        ("location_status", ctypes.c_uint8),
        ("longitude", ctypes.c_float),
        ("latitude", ctypes.c_float),
        ("direction", ctypes.c_uint16),
    ]


class SDK_TASK_DISTRIBUTION_T(PackedStructure):
    _fields_ = [
        ("task_version", ctypes.c_uint8),
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("yaw", ctypes.c_float),
        ("speed_ratio", ctypes.c_uint32),
        ("move_mode", ctypes.c_uint32),
        ("floor", ctypes.c_int8),
        ("elevator_selection", ctypes.c_uint8),
        ("map_name", ctypes.c_char * UINT8_MAX),
        ("hoist_type", ctypes.c_uint8),
    ]


class SDK_COMMUNICATION_CONTROL_T(PackedStructure):
    _fields_ = [
        ("communication_control", ctypes.c_int),
        ("SSID", ctypes.c_char * UINT8_MAX),
        ("pass_wd", ctypes.c_char * UINT8_MAX),
    ]


class SDK_EAI_TASK_STATU_T(PackedStructure):
    _fields_ = [
        ("task_uuid", ctypes.c_char * 37),
        ("status", ctypes.c_uint8),
    ]


class SDK_DYNAMIC_PARAM_LIST_T(PackedStructure):
    _fields_ = [
        ("param_size", ctypes.c_uint8),
        ("param_name", ctypes.POINTER(ctypes.c_char_p)),
        ("param_val", ctypes.POINTER(ctypes.c_char_p)),
    ]


class SDK_BACK_TBOX_ERROR_DISPLAY_T(PackedStructure):
    _fields_ = [("error_msg", ctypes.c_char * UINT16_MAX)]


class SDK_BACK_CHARGING_STATU_T(PackedStructure):
    _fields_ = [
        ("is_auto_charging", ctypes.c_uint8),
        ("is_emergency_charging", ctypes.c_uint8),
        ("charging_station_state", ctypes.c_uint8),
        ("charging_state", ctypes.c_uint8),
        ("electricity", ctypes.c_uint8),
        ("infraed_info", ctypes.c_uint16),
        ("pms_real_info", ctypes.c_uint16),
        ("remainning_service_time", ctypes.c_uint32),
        ("Rearcover_open", ctypes.c_uint8),
    ]


class SDK_BACK_TASK_STATU_T(PackedStructure):
    _fields_ = [
        ("uuid", ctypes.c_char * 37),
        ("map_name", ctypes.c_char * UINT8_MAX),
        ("floor", ctypes.c_uint8),
        ("distance", ctypes.c_float),
        ("task_type", ctypes.c_uint8),
        ("task_state", ctypes.c_uint8),
        ("run_state", ctypes.c_uint8),
        ("sub_state", ctypes.c_uint32),
    ]


class SDK_BACK_UNDERPAN_STATE_T(PackedStructure):
    _fields_ = [
        ("signal_intensity", ctypes.c_uint8),
        ("sim_status", ctypes.c_uint8),
        ("plant_is_connect", ctypes.c_uint8),
        ("milg", ctypes.c_uint32),
        ("speed", ctypes.c_uint8),
        ("network_type", ctypes.c_uint8),
        ("wifi_signal", ctypes.c_uint8),
        ("charging_floor", ctypes.c_uint8),
        ("ipc_status", ctypes.c_uint8),
    ]


class SDK_BACK_CHASSIS_STATIC_INFO_T(PackedStructure):
    _fields_ = [
        ("batter_type", ctypes.c_uint8),
        ("batter_calibration", ctypes.c_uint8),
        ("wifi_mac", ctypes.c_uint8 * UINT8_MAX),
        ("ap_mac", ctypes.c_uint8 * UINT8_MAX),
        ("wifi_ip", ctypes.c_uint8 * UINT8_MAX),
    ]


class SDK_BACK_EAI_TASK_DELIVER_T(PackedStructure):
    _fields_ = [
        ("kind", ctypes.c_uint8),
        ("dataLen", ctypes.c_uint16),
        ("data", ctypes.c_char * UINT16_MAX),
    ]


class SDK_BACK_CUR_POSE_T(PackedStructure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("yaw", ctypes.c_float),
    ]


RemoteControl = SDK_REMOUTE_CONTROL_T
RotationControl = SDK_ROTATION_CONTROL_T


__all__ = [name for name in globals() if name.startswith("SDK_")] + [
    "PackedStructure",
    "RemoteControl",
    "RotationControl",
    "UINT8_MAX",
    "UINT16_MAX",
]
