#pragma once
#include <cstdint>
enum TCP_CMD
{
    PACKET_CMD_LOGIN = 1,                         // 注册
    PACKET_CMD_QRCODE_RECORD_CTL = 2,             // 录点控制
    PACKET_CMD_HEARTBEAT = 4,                     // 心跳
    PACKET_CMD_LOGOUT = 5,                        // 登出
    PACKET_CMD_DTU_UPDATE = 0x07,                 // DTU升级
    PACKET_CMD_DTU_PARAM_GET = 0x08,              // DTU参数获取 查询后采用0xF0进行上报
    PACKET_CMD_DTU_PARAM_SET = 0x09,              // DTU参数设置
    PACKET_CMD_PGM_UPDATE_NOTIC = 0x0A,           // PGM更新通知
    PACKET_CMD_RELOCATION = 0x0B,                 // 重定位
    PACKET_CMD_NONNET_WALKMODE = 0x0D,            // 无网络行走
    PACKET_CMD_CAMERA_SHARE_SWITCH = 0x10,        // 分流开关
    PACKET_CMD_CAMERA_SHARE_SWITCH_CHECK = 0x11,  // 分流开关查询
    PACKET_CMD_COMMUNICATION_CHECK_STATUS = 0x12, // 通讯开关控制状态查询

    PACKET_CMD_FUNC_AREA_SETTINGS = 0x14,   // 功能区设置
    PACKET_CMD_FUNC_AREA_QUERY = 0x15,      // 功能区查询
    PACKET_CMD_MAP_ACQUISITION_CTL = 0x16,  // 采图控制
    PACKET_CMD_FLOOR_REGISTER = 0x21,       // 楼层登记
    PACKET_CMD_FLOOR_DOOR_CTRL = 0x22,      // 梯门控制请求
    PACKET_CMD_BUILDING_INFO = 0x23,        // 楼栋信息
    PACKET_CMD_BUILDING_ELEVATOR = 0x24,    // 楼栋与电梯绑定
    PACKET_CMD_BUILDING_MAP = 0x25,         // 楼栋与地图绑定
    PACKET_CMD_LIFT_INFO_QUERY = 0x27,      // 电梯状态查询
    PACKET_CMD_DISPATCH_MODE_SWITCH = 0x30, // 调度模式开关
    PACKET_CMD_DISPATCH_ORDER_DOWN = 0x31,  // 调度任务下发

    PACKET_CMD_REMOTE_CONTROL = 0x37,       // 遥控控制
    PACKET_CMD_ROTATION_CONTROL = 0x38,     // 遥控旋转

    PACKET_CMD_VIDEO_SWITCH = 0x41,          // 录像功能开关
    PACKET_CMD_VIDEO_SWITCH_SEARCH = 0x42,   // 录像功能开关查询
    PACKET_CMD_SPECIAL_POINT_SETTING = 0x45, // 特殊点位设置
    PACKET_CMD_SCREEN_OPERATION = 0x46,      // 屏幕操作反馈
    PACKET_CMD_GOTO_AVOIDANCE_POINT = 0x47,  // 去避让点
    PACKET_CMD_HOIST_UP_RESULT = 0x48,       // 举升任务结果
    PACKET_CMD_HOIST_CTRL = 0x49,            // 举升控制
    PACKET_CMD_HOIST_QUERY = 0x4A,           // 举升高度查询
    PACKET_CMD_HOIST_SET = 0x4B,             // 举升电机状态设置

    PACKET_CMD_FICM_ERROR_UPLOAD = 0x50,           // FICM故障上传
    PACKET_CMD_FICM_VERSION = 0x51,                // FICM版本信息
    PACKET_CMD_DRIVING_RECORD_SWITCH = 0x52,       // 行车记录控制
    PACKET_CMD_DRIVING_FILE_MANAGE = 0x53,         // 行车记录文件管理
    PACKET_CMD_DRIVING_FILE_FIND = 0x54,           // 行车记录文件查询指令
    PACKET_CMD_POWER_MANAGE = 0x55,                // 电源管理控制
    PACKET_CMD_TIME_TASK_SYNC = 0x56,              // 定时任务同步
    PACKET_CMD_INFRARED_SWITCH = 0x57,             // 红外开关控制
    PACKET_CMD_TBOX_VERSION = 0x58,                // 版本信息获取
    PACKET_CMD_TBOX_VIN_ICCID = 0x59,              // VIN/ICCID码获取
    PACKET_CMD_POSITION = 0x5A,                    // 位置数据
    PACKET_CMD_TASK_CTRL = 0x5B,                   // 任务控制
    PACKET_CMD_BACK_PILE = 0x5C,                   // 回桩任务
    PACKET_CMD_TIMER_ON_OFF = 0x5D,                // 定时开关机任务
    PACKET_CMD_ELEVATOR_NUMBHER = 0x5E,            // 电梯统号设置
    PACKET_CMD_ELEVATOR_POINT = 0x5F,              // 梯门点设置
    PACKET_CMD_TASK_DISTRIBUTION = 0x60,           // 任务下发
    PACKET_CMD_GO_START = 0x61,                    // 回开机点
    PACKET_CMD_RESTORE_FACTORY = 0x62,             // 恢复出厂
    PACKET_CMD_TRACK_TO_POINT = 0x63,              // 循迹到具体点
    PACKET_CMD_TRACKING_TASK = 0x64,               // 循迹任务
    PACKET_CMD_CABINET_DOOR_STATE = 0x65,          // 柜门状态
    PACKET_CMD_CLEAR_ERROR_CODE = 0x66,            // 碰撞故障清除
    PACKET_CMD_WIFI_CTL = 0x67,                    // WIFI开关
    PACKET_CMD_LIGHT_CTL = 0x68,                   // 灯光控制
    PACKET_CMD_VIDEO_UPLOAD_CTL = 0x6A,            // 录像控制(二维码)
    PACKET_CMD_EMBODIED_INTELLIGENCE_STATU = 0x6C, // 具身智能任务状态
    PACKET_CMD_PARAM_SET = 0x77,                   // 参数设置 上行
    PACKET_CMD_PARAM_GET = 0x78,                   // 参数查询 下行
    PACKET_CMD_REFLASH_FILE = 0x7A,                // 刷新文件下发
    PACKET_CMD_FILE_DOWNLOAD = 0x7D,               // 文件下发
    PACKET_CMD_BRAKE_CONTROL = 0x83,               // 刹车控制
    PACKET_CMD_AREA_SWEEPING_MODE_SETTING = 0xA6,  // 清扫区设置
    PACKET_CMD_TASK_SWEEPING = 0xA7,               // 清扫任务下发
    PACKET_CMD_TOUCH_UP_RECORD = 0xA8,             // 续扫标记记录
    PACKET_CMD_AREA_SWEEPING_MODE_SEARCH = 0xA9,   // 区域清扫模式查询
    PACKET_CMD_MAPPING_TABLE = 0xC0,               // 特检院楼层映射表下发
    PACKET_CMD_AVOIDANCE_POINT_SETTING = 0xC5,     // 避让点设置
    PACKET_CMD_AVOIDANCE_POINT_SEARCH = 0xC6,      // 避让点查询
    PACKET_CMD_NOTICE_START = 0xCF,                // 区分上下行
    PACKET_CMD_POINT_ARRIVED = 0xD0,               // 点位到达
    PACKET_CMD_FUNC_AREA_ARRIVED_STATUS = 0xD1,    // 功能区到达状态
    PACKET_CMD_SWEEPING_CTL = 0xD3,                // 清扫控制
    PACKET_CMD_DISPATCH_CTL_INSTRUCTION = 0xDA,    // 调度控制指令
    PACKET_CMD_TBOX_ERROR_DISPLAY = 0xE0,          // 故障信息显示
    PACKET_CMD_QRCODE_LIST_UPLOAD = 0xE1,          // 录点点位列表
    PACKET_CMD_CHARGING = 0xE3,                    // 运作请求
    PACKET_CMD_CHARGING_STATE = 0xE4,              // 电量状态
    PACKET_CMD_TASK_STATE = 0xE5,                  // 任务状态上传
    PACKET_CMD_FLOOR_STATE = 0xE6,                 // 电梯状态
    PACKET_CMD_REMOTE_REFRESH_STATE = 0xE7,        // 远程刷新状态
    PACKET_CMD_UNDERPAN_STATE = 0xE8,              // 底盘硬件状态
    PACKET_CMD_CHASSIS_STATIC_INFO = 0xE9,         // 底盘静态信息上传
    // PACKET_CMD_DOOR_SWITCH_REQ = 0xE9,          // 门禁开关请求
    PACKET_CMD_CHASSIS_ABNORMAL_REPORT = 0xEA,     // 底盘异常信息上报
    PACKET_CMD_FUNCTIONAL_AREA_WAIT_REPORT = 0xEB, // 功能区等待上报
    PACKET_CMD_EMBODIED_INTELLIGENCE_TASK = 0xED,  // 具身智能任务
    PACKET_CMD_POSE_INFO = 0xEE,                 // 位姿上报
    PACKET_CMD_DTU_PARAM_REPORT = 0xF0,            // DTU参数上报
    PACKET_CMD_PROTOCOL_END = 0xFF,                // 协议结尾
};
enum class TBOX_CONNECT_E : uint8_t
{
    DISCONNECT = 0,         // 未连接
    CONNECTED = 1,          // 连接成功
    LOGIN_SUCCEED = 1 << 1, // 登录成功
};