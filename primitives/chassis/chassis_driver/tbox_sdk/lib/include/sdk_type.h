#pragma once
// #include <unistd.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C"
{
#endif
#ifndef UINT8_MAX
#define UINT8_MAX (255)
#endif
#pragma pack(push, 1) // 一个字节一个字节的对齐

    /*
        重定位类型枚举定义(0x0B)
        */
    typedef enum
    {
        RELOCATION_TYPE_POSE = 0x01,             // 位姿重定位
        RELOCATION_TYPE_LAST_SHUTDOWN = 0x02,    // 上次关机点重定位
        RELOCATION_TYPE_FULL_MAP = 0x03,         // 全图重定位
        RELOCATION_TYPE_WALL_CODE = 0x04,        // 墙码重定位
        RELOCATION_TYPE_PILE_CODE = 0x05,        // 桩码重定位
        RELOCATION_TYPE_CEILING = 0x06,          // 天花板重定位
        RELOCATION_TYPE_REFLECTIVE_STRIP = 0x07, // 反光条码重定位
        RELOCATION_TYPE_AUX_DISABLE = 0x0F       // 关闭辅助重定位
    } RELOCATION_TYPE_E;
    /**
     * @brief 重定位结构体
     *
     * 此结构体用于携带重定位相关的参数。
     *
     * @param type: 重定位类型，见RELOCATION_TYPE_E定义。
     * @param threshold: 阈值
     * @param left_top_x: 左上角X坐标
     * @param left_top_y: 左上角Y坐标
     * @param yaw: 目标方向角
     */
    typedef struct
    {
        uint8_t type;     // 重定位类型,见枚举RELOCATION_TYPE_E定义
        float threshold;  // 阈值(实际值0.00-0.50)
        float left_top_x; // X轴坐标(米)
        float left_top_y; // Y轴坐标(米)
        float yaw;        // 目标方向角(度)
    } SDK_RELOCATION_T, *PSDK_RELOCATION_T;

    /* 重定位返回结构体 */
    typedef struct
    {
        uint8_t result;                 // 结果: 0x0-成功, 0x1-失败, 0x2-码点为空
        uint8_t score;                  // 得分
        char failure_reason[UINT8_MAX]; // 失败原因(柔性数组,实际长度由failure_reason_len决定)
    } SDK_ACK_RELOCATION_T, *PSDK_ACK_RELOCATION_T;
    /*-----------------------------------------------------------------------------------------------------*/

    /**
     * @brief 通信控制与状态结构体
     * @details 用于配置和获取TBOX的通信模块状态，包括4G、WiFi、热点等网络功能
     *          通过位域控制多个通信模块的开关状态
     * @param communication_control
     * @brief 通信模块控制位域
     *
     * * @details 通过位操作控制各个通信模块的启用/禁用状态
     *          每位代表一个通信模块的开关状态（0：关闭，1：开启）
     *
     * @par 位域定义：
     * | 位 | 名称       | 说明                                   |
     * |----|------------|----------------------------------------|
     * | 0  | 4G模块     | 控制4G移动数据网络的开关                |
     * | 1  | WiFi客户端 | 控制WiFi连接功能的开关                  |
     * | 2  | WiFi网卡   | 控制WiFi网卡硬件的开关                  |
     * | 3  | 外网连接   | 控制是否允许访问外部互联网              |
     * | 4  | 热点功能   | 控制是否启用WiFi热点（AP模式）          |
     * | 5-7| 保留位     | 保留供未来扩展使用，必须设置为0         |
     *
     * @note 使用说明：
     *       1. 通过位运算设置或检查各个位的状态
     *       2. 示例：开启4G和WiFi客户端
     *          `communication_control = (1 << 0) | (1 << 1);`
     *       3. 某些功能之间存在依赖关系：
     *          - WiFi客户端需要WiFi网卡开启（位2=1）
     *          - 热点功能需要WiFi网卡开启（位2=1）
     *          - 外网连接需要至少一个网络模块开启
     *
     * @warning 注意事项：
     *       1. 改变通信状态可能导致当前网络连接中断
     *       2. 请勿同时启用WiFi客户端和热点功能（硬件限制）
     *       3. 关闭所有通信模块将导致TBOX无法远程连接
     * @param SSID
     * @brief 机器人底盘热点SSID（Service Set Identifier）
     * @details 当热点功能开启时，TBOX作为AP提供的WiFi网络名称
     *
     * @par 特性：
     * - 最大长度：255字节（UINT8_MAX定义）
     * - 格式：可打印ASCII字符
     * - 默认值：通常为"TBOX_"+设备序列号后6位
     *
     * @note 使用说明：
     *       1. 仅当communication_control的位4（热点功能）为1时有效
     *       2. 用于移动设备或上位机连接TBOX热点
     *       3. 可通过此字段读取当前热点的SSID，或设置新的SSID
     *
     * @warning 安全提示：
     *       1. 建议设置具有一定复杂度的SSID，避免被轻易识别
     *       2. SSID中避免包含敏感信息（如设备序列号、VIN等）
     * @param Wifi
     * @brief 机器人连接的WiFi网络SSID
     * @details TBOX作为WiFi客户端连接的外部网络名称
     *
     * @par 特性：
     * - 最大长度：255字节（UINT8_MAX定义）
     * - 格式：可打印ASCII字符
     * - 空字符串表示未连接任何WiFi网络
     *
     * @note 使用说明：
     *       1. 仅当communication_control的位1（WiFi客户端）为1时有效
     *       2. 用于记录或设置TBOX要连接的WiFi网络
     *       3. 读取此字段可获取当前连接的WiFi网络名称
     *       4. 设置此字段后需要调用相应配置接口使设置生效
     *
     * @warning 注意事项：
     *       1. WiFi密码通过其他安全接口设置，不在此结构体中
     *       2. 更改WiFi网络可能导致网络切换和短暂中断
     *       3. 确保目标WiFi网络在TBOX的信号覆盖范围内
     *
     * @see communication_control 中的WiFi客户端控制位
     * @see tbox_sdk_Send_getCommuStatu
     */
    typedef struct
    {
        uint8_t communication_control;
        char SSID[UINT8_MAX];
        char Wifi[UINT8_MAX];
    } SDK_ACK_COMMUNICATION_T, *PSDK_ACK_COMMUNICATION_T;
    /*-----------------------------------------------------------------------------------------------------*/

    /**
     * @brief 采图控制命令枚举
     * @details 用于控制建图采集过程
     */
    typedef enum
    {
        E_MAP_ACQUISITION_INCREMENTAL = 0x01, // 增量建图
        E_MAP_ACQUISITION_FINISH = 0x02,      // 采图结束
        E_MAP_ACQUISITION_CLEAR = 0x03,       // 清除当前地图重新建图
    } SDK_MAP_ACQUISITION_CMD_E;

    /*采图控制返回值*/
    typedef struct
    {
        uint8_t status;   // 状态: 0-成功, 1-等待中, 2-运行中, 3-失败
        uint8_t substate; // 子状态: 1-保存失败, 2-重影, 3-边缘不完整, 4-地图不正
    } SDK_ACK_MAP_ACQUISITION_STATUS_T, *PSDK_ACK_MAP_ACQUISITION_STATUS_T;
    /*-----------------------------------------------------------------------------------------------------*/
    /*遥控指令控制*/
    typedef struct
    {
        float linear_speed;          // 线速度，精度0.001m/s， 范围-1~1 m/s负数向后，正数向前
        float angular_speed;         // 角速度，精度0.001m/s，范围：-1~1 m/s负数向右，正数向左
        uint8_t seriousFaultCanMove; // 严重故障（1，2级故障）下是否允许移动
    } SDK_REMOUTE_CONTROL_T, *PSDK_REMOUTE_CONTROL_T;
    ;
    /*-----------------------------------------------------------------------------------------------------*/
    /*旋转控制*/
    typedef struct
    {
        uint8_t type;         // 旋转类型：0:以地图基准方向，旋转到指定角度, 1:当前位置旋转角度
        float rotation_angle; // 旋转角弧度，（精度0.000001）
        float speed_ratio;    // 旋转速率，速度调制参数范围是1-5（0.1 到0.5）
    } SDK_ROTATION_CONTROL_T, *PSDK_ROTATION_CONTROL_T;
    /*-----------------------------------------------------------------------------------------------------*/
    /*举升控制*/
    typedef struct
    {
        uint8_t ctrl;       // 控制命令，0x0:位置控制比例;0x1:位置控制高度;
        int32_t height;    // 升降高度，ctrl=0时，值范围为：0-100;cltrl=1s时，为实际高度,单位毫米(可给负数，若零位不是在最底部时)
        uint8_t speed;      // 升降速度，1-100,毫米/秒
    } SDK_HOIST_CONTROL_T, *PSDK_HOIST_CONTROL_T;
    /*-----------------------------------------------------------------------------------------------------*/
    /*举升高度读取反馈*/
    typedef struct
    {
        int32_t height;  // 升降高度，单位毫米（以零位为基点）
        uint8_t ratio;   // 总高度的比例，范围：0~100
        uint16_t errcode; /* 故障码：0x0001--编码器故障
                                    0x0002 编码器故障 UVW 报警
                                    0x0003 位置超差
                                    0x0004 失速
                                    0x0005 电流采样(中点)故障
                                    0x0006 过载
                                    0x0007 欠压
                                    0x0008 过压
                                    0x0009 过流
                                    0x000A 放电报警瞬时功率大
                                    0x000B 放电回路频繁动作平均功率大
                                    0x000C 参数读写异常
                                    0x000D 输入口功能定义重复
                                    0x000E 通讯看门狗触发
                                    0x000F 电机过温报警
                                    0x0012 驱动器过温
                                    0x0013 绝对值电池欠压*/

    } SDK_ACK_HOIST_QUERY_T, *PSDK_ACK_HOIST_QUERY_T;
    /*-----------------------------------------------------------------------------------------------------*/
    /*举升电机设置*/
    typedef struct
    {
        uint8_t ctrl;       // 控制命令，0x0:故障清除;0x1:零位设置;
    } SDK_HOIST_SET_T, *PSDK_HOIST_SET_T;
    /*-----------------------------------------------------------------------------------------------------*/
    /**
     * @brief 电源管理结构体
     * @details 用于控制设备的电源管理功能
     * @param auto_charging_switch: 低电回充开关 (0: 关, 1: 开)
     * @param charging_threshold_value: 自动充电阈值 (0: 25%, 1: 30%, 2: 40%, 3: 50%, 4: 60%)
     * @param working_threshold_value: 工作阈值 (0: 25%, 1: 30%, 2: 40%, 3: 50%, 4: 60%)
     * @param charging_floor: 充电楼层
     * @param charging_mode: 充电模式 (0x0: 自动充电, 0x1: 快换换电)
     */
    typedef struct
    {
        uint8_t auto_charging_switch;     // 低电回充开关 0:关 1:开
        uint8_t charging_threshold_value; // 自动充电阈值 0:25% 1:30% 2:40% 3:50% 4:60%
        uint8_t working_threshold_value;  // 工作阈值 0:25% 1:30% 2:40% 3:50% 4:60%
        uint8_t charging_floor;           // 充电楼层
        uint8_t charging_mode;            // 充电模式 0x0:自动充电 0x1:快换换电
    } SDK_POWER_MANAGE_T, *PSDK_POWER_MANAGE_T;

    /*-----------------------------------------------------------------------------------------------------*/

    /*版本信息获取*/
    typedef struct
    {
        uint8_t ficm_software_version[UINT8_MAX];    // FICM软件版本
        uint8_t hbox_software_version[UINT8_MAX];    // HBox软件版本
        uint8_t pms_software_version[UINT8_MAX];     // PMS软件版本
        uint8_t pcm_software_version[UINT8_MAX];     // BCM软件版本
        uint8_t ssm_software_version[UINT8_MAX];     // SSM软件版本
        uint8_t sdk_software_version[UINT8_MAX];     // HBox SDK软件版本
        uint8_t tboxsdk_software_version[UINT8_MAX]; // Tbox SDK版本号
        uint8_t sbox_software_version[UINT8_MAX];    // SBox软件版本号
        uint8_t tbox_software_version[UINT8_MAX];    // TBox版本号
        uint8_t kernel_software_version[UINT8_MAX];  // 系统内核版本
        uint8_t rootfst_software_version[UINT8_MAX]; // 文件系统版本号
        uint8_t hvcu_software_version[UINT8_MAX];    // HVCU版本
    } SDK_ACK_TBOX_VERSION_T, *PSDK_ACK_TBOX_VERSION_T;
    /*-----------------------------------------------------------------------------------------------------*/
    /**
     * @brief 车辆VIN/ICCID信息返回结构体
     *
     * 此结构体用于查询车辆VIN/ICCID信息
     * @param vin
     * @param iccid
     */
    typedef struct
    {
        char vin[18];
        char iccid[21];
    } SDK_ACK_CAR_DEVICE_INFO_T, *PSDK_ACK_CAR_DEVICE_INFO_T;
/*-----------------------------------------------------------------------------------------------------*/
// 定位状态位掩码定义
#define LOCATION_VALID_MASK 0x01 // Bit0: 0-有效定位,1-无效定位
#define LATITUDE_TYPE_MASK 0x02  // Bit1: 0-北纬,1-南纬
#define LONGITUDE_TYPE_MASK 0x04 // Bit2: 0-东经,1-西经

// 定位状态位偏移
#define LOCATION_VALID_OFFSET 0
#define LATITUDE_TYPE_OFFSET 1
#define LONGITUDE_TYPE_OFFSET 2

// 获取定位状态各字段的宏
#define GET_LOCATION_VALID(status) (((status) >> LOCATION_VALID_OFFSET) & 0x01)
#define GET_LATITUDE_TYPE(status) (((status) >> LATITUDE_TYPE_OFFSET) & 0x01)
#define GET_LONGITUDE_TYPE(status) (((status) >> LONGITUDE_TYPE_OFFSET) & 0x01)
    /**
     * @brief 定位信息结构体
     * @details 用于表示设备的定位状态和位置信息
     * @param location_status: 定位状态
     * @param longitude: 经度，以度为单位的经度值乘以10^6，精确到百万分之一度
     * @param latitude: 纬度，以度为单位的纬度值乘以10^6，精确到百万分之一度
     * @param direction: 方向，有效值范围：0～359，正北为0，顺时针
     */
    typedef struct
    {
        uint8_t location_status; // 定位状态
        float longitude;         // 经度，以度为单位的经度值乘以10^6，精确到百万分之一度
        float latitude;          // 纬度，以度为单位的纬度值乘以10^6，精确到百万分之一度
        uint16_t direction;      // 方向，有效值范围：0～359，正北为0，顺时针
    } SDK_POSITION_T, *PSDK_POSITION_T;

    /*-----------------------------------------------------------------------------------------------------*/
    /**
     * @brief TBOX任务分发结构体
     *
     * 该结构体用于向TBOX下发移动任务指令，包含目标点坐标、移动参数等配置信息。
     * 注意：不同TBOX版本支持的参数可能有所不同，请根据实际版本设置相应字段。
     *
     * 字段说明：
     *
     * @param task_version: TBOX版本标识
     *   - 1: TBOX1.0版本
     *   - 2: TBOX2.0版本
     *   - 注意：必须准确设置，否则可能导致参数解析错误
     *
     * @param x: 目标点X坐标（单位：米）
     *   - 地图坐标系下的横向坐标
     *   - 使用地图定义的坐标系（如UTM、局部坐标系等）
     *   - 需确保坐标点在地图有效范围内
     *
     * @param y: 目标点Y坐标（单位：米）
     *   - 地图坐标系下的纵向坐标
     *   - 与x坐标共同确定目标位置
     *
     * @param yaw: 目标点朝向角（单位：弧度）
     *   - 机器人在目标点的最终朝向
     *   - 范围：-π ~ π
     *   - 0度：朝向X轴正方向
     *   - π/2：朝向Y轴正方向
     *   - 注意：对某些移动模式可能忽略此参数
     *
     * @param speed_ratio: 移动速度比例
     *   - 控制机器人的移动速度，数值越大速度越快
     *   - 范围：0-5
     *   - 默认：3（中等速度）
     *   - 0：最低速度（调试/安全模式）
     *   - 5：最高速度（谨慎使用）
     *   - 注意：实际速度受机器人最大速度限制
     *
     * @param move_mode: 移动方式
     *   - 指定机器人到达目标点的移动策略
     *   - 0：默认移动方式（正向行驶）
     *   - 5：安全避障模式
     *   - 注意：不同TBOX版本支持的移动模式可能不同
     *
     * @param floor: 目标点楼层
     *   - 多楼层环境下的目标楼层编号
     *   - 通常：0为地面层，正数为地上楼层，负数为地下楼层
     *   - 示例：-1：地下一层，1：一层，2：二层
     *   - 注意：单楼层应用可设置为0
     *
     * @param elevator_selection: 电梯门选择策略
     *   - 多电梯环境下的电梯选择策略
     *   - 0：同门进出（出发和目标楼层使用同一电梯门）
     *   - 1：异门进出（出发和目标楼层使用不同电梯门）
     *   - 注意：仅TBOX2.0及以上版本有效，TBOX1.0可忽略此参数
     *
     * @param map_name: 目标地图名称
     *   - 指定任务执行的地图文件
     *   - 命名规则：楼栋名_楼层_日期.bin
     *   - 示例："gs_1_20260108.bin"（GS楼栋1层2026年1月8日地图）
     *   - 最大长度：255字节（UINT8_MAX定义）
     *   - 注意：TBOX1.0可不赋值，使用默认地图
     *   - 警告：地图文件需预先加载到TBOX，否则任务会失败
     *
     * @param hoist_type: 顶升任务类型
     *   - 控制机器人顶升机构的升降动作
     *   - 0：无顶升任务（默认）
     *   - 1：上升任务（顶升机构上升）
     *   - 2：下降任务（顶升机构下降）
     *   - 注意：仅TBOX2.0顶升车型有效，非顶升车型请设置为0
     *   - 警告：错误设置可能导致设备损坏或安全事故
     */
    typedef struct
    {
        uint8_t task_version;
        float x;
        float y;
        float yaw;
        uint32_t speed_ratio;
        uint32_t move_mode;
        int8_t floor;
        uint8_t elevator_selection;
        char map_name[UINT8_MAX];
        uint8_t hoist_type;

    } SDK_TASK_DISTRIBUTION_T, *PSDK_TASK_DISTRIBUTION_T;

    /*-----------------------------------------------------------------------------------------------------*/

    /**
     * @brief 设备通信控制指令枚举值定义
     *
     * 此枚举定义了各种通信控制操作的指令，包括开启和关闭热点、WiFi、4G及外网。
     */
    typedef enum
    {
        COMMUNICATION_CTRL_HOST_CLOSE = 0,     ///< 关闭热点
        COMMUNICATION_CTRL_HOST_OPEN = 1,      ///< 开启热点
        COMMUNICATION_CTRL_WIFI_CLOSE = 2,     ///< 关闭WiFi
        COMMUNICATION_CTRL_WIFI_OPEN,          ///< 开启WiFi
        COMMUNICATION_CTRL_4G_CLOSE,           ///< 关闭4G网络
        COMMUNICATION_CTRL_4G_OPEN,            ///< 开启4G网络
        COMMUNICATION_CTRL_EXTERNAL_NET_CLOSE, ///< 关闭外网访问
        COMMUNICATION_CTRL_EXTERNAL_NET_OPEN,  ///< 开启外网访问
        COMMUNICATION_CTRL_HOST_MODIFY,        ///< 修改热点名称（仅需账号）
        COMMUNICATION_CTRL_WIFI_CONNECT        ///< WiFi连接（需要账号和密码）
    } SDK_COMMUNICATION_CTRL_CMD_E;

    /**
     * @brief 设备通信控制结构体
     *
     * 此结构体用于携带设备通信控制的指令及相关参数。
     *
     * @param communication_control 指令类型，使用SDK_COMMUNICATION_CTRL_CMD_E枚举定义。
     * @param SSID 热点或WiFi的SSID名称，最大长度为UINT8_MAX。
     * @param pass_wd 热点或WiFi的密码，最大长度为UINT8_MAX。
     */
    typedef struct
    {
        SDK_COMMUNICATION_CTRL_CMD_E communication_control; ///< 通信控制指令
        char SSID[UINT8_MAX];                               ///< 热点或WiFi名称
        char pass_wd[UINT8_MAX];                            ///< 热点或WiFi密码
    } SDK_COMMUNICATION_CONTROL_T, *PSDK_COMMUNICATION_CONTROL_T;
    /*-----------------------------------------------------------------------------------------------------*/
    typedef struct
    {
        char task_uuid[37];
        uint8_t status;
    } SDK_EAI_TASK_STATU_T, *PSDK_EAI_TASK_STATU_T;
    /*-----------------------------------------------------------------------------------------------------*/

    typedef struct
    {
        uint8_t param_size;
        char **param_name;
        char **param_val;
    } SDK_DYNAMIC_PARAM_LIST_T, *PSDK_DYNAMIC_PARAM_LIST_T;

    /**************************************************************************************************************
     * 下行对外接口
     **************************************************************************************************************/
    /* 故障信息显示结构体 */
    typedef struct
    {
        char error_msg[UINT16_MAX];
    } SDK_BACK_TBOX_ERROR_DISPLAY_T, *PSDK_BACK_TBOX_ERROR_DISPLAY_T;

    /**
     * @brief 充电状态回调结构体
     *
     * 用于接收TBOX返回的充电状态信息，在收到PACKET_CMD_CHARGING_STATE命令时使用
     *
     * 字段说明：
     * @param is_auto_charging: 是否自动充电（0-未连接；1-连接）
     * @param is_emergency_charging: 是否应急充电（0-非应急；1-应急充电）
     * @param charging_station_state: 充电桩状态（0-离线；1-在线；2-故障）
     * @param charging_state: 充电状态（0-未充电；1-自动充电中；2-应急充电中；3-充满）
     * @param electricity: 电量百分比（0-100）
     * @param infraed_info: 红外信息（各bit位表示不同红外传感器状态）
     * @param pms_real_info: PMS实时信息（电源管理状态）
     * @param remainning_service_time: 剩余使用时间（单位秒）
     * @param Rearcover_open: 后盖检测状态（0-未打开；1-打开）
     */
    typedef struct
    {
        uint8_t is_auto_charging;
        uint8_t is_emergency_charging;
        uint8_t charging_station_state;
        uint8_t charging_state;
        uint8_t electricity;
        uint16_t infraed_info;
        uint16_t pms_real_info;
        uint32_t remainning_service_time;
        uint8_t Rearcover_open;
    } SDK_BACK_CHARGING_STATU_T, *PSDK_BACK_CHARGING_STATU_T;

    /**
     * @param uuid
     * 任务ID
     * @param map_name
     * 地图名称
     * @param distance
     * 目标点距离
     * @param task_type
     * 任务类型
     * @param task_state
     * 任务状态
     * @param runc_state
     * 运行状态
     * @param sub_state
     * 子状态
     */
    typedef struct
    {
        char uuid[37];
        char map_name[UINT8_MAX];
        uint8_t floor;
        float distance;
        uint8_t task_type;
        uint8_t task_state;
        uint8_t run_state;
        uint32_t sub_state;
    } SDK_BACK_TASK_STATU_T, *PSDK_BACK_TASK_STATU_T;

    /*底盘硬件状态返回值*/
    typedef struct
    {
        uint8_t signal_intensity; // 4G信号强度
        uint8_t sim_status;       // SIM卡状态: 0x0-连接, 0x1-未连接
        uint8_t plant_is_connect; // 平台连接状态: 0-未连接, 1-连接
        uint32_t milg;            // 里程信息(单位:米)
        uint8_t speed;            // 实际速度(单位:米/秒)
        uint8_t network_type;     // 网络类型: 0x0-无网络, 0x1-4G, 0x2-wifi
        uint8_t wifi_signal;      // WiFi信号强度
        uint8_t charging_floor;   // 充电楼层(若无充电楼层信息则发送1)
        uint8_t ipc_status;       // 工控机连接状态: 0x0-不显示, 0x1-连接, 0x2-未连接
    } SDK_BACK_UNDERPAN_STATE_T, *PSDK_BACK_UNDERPAN_STATE_T;

    /* 底盘静态信息返回值 */
    typedef struct
    {
        uint8_t batter_type;         // 电池包类型:0x0:识别异常；0x1:铅酸;0x2:钴酸锂（带通讯）;0x3:三元（带通讯）;0x4:锂电（带通讯）; 0x5:锂电
        uint8_t batter_calibration;  // 电池包校准状态,0x0:未校准；0x1:已校准
        uint8_t wifi_mac[UINT8_MAX]; // WiFi MAC地址
        uint8_t ap_mac[UINT8_MAX];   // AP MAC地址
        uint8_t wifi_ip[UINT8_MAX];  // WiFi IP地址
    } SDK_BACK_CHASSIS_STATIC_INFO_T, *PSDK_BACK_CHASSIS_STATIC_INFO_T;

    typedef struct
    {
        uint8_t kind;          // 任务类型:1：取件
        uint16_t dataLen;      // 数据长度，kind=1时，长度=1
        char data[UINT16_MAX]; // 数据内容，kind=1时，数据是柜号 (BYTE)
    } SDK_BACK_EAI_TASK_DELIVER_T, *PSDK_BACK_EAI_TASK_DELIVER_T;

    /*当前位姿信息*/
    typedef struct
    {
        float x;        // 地图坐标系中的X轴方向位置，精度0.01，单位：米；
        float y;        // 地图坐标系中的Y轴方向位置，精度0.01，单位：米；
        float yaw;      // 偏航角，机器人与原点位置方向的一个角度差值,精度0.01,单位：角弧度；
    } SDK_BACK_CUR_POSE_T, *PSDK_BACK_CUR_POSE_T;
#pragma pack(pop)
#ifdef __cplusplus
}
#endif
