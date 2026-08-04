#ifndef TBOX_SDK_H
#define TBOX_SDK_H

#include "sdk_type.h"
#include <string>
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef _WIN32
#define TBOX_SDK_API __declspec(dllexport)
#else
#define TBOX_SDK_API __attribute__((visibility("default")))
#endif
    /** @brief 状态上报回调函数指针类型
     *
     * @description 此回调函数用于异步接收设备状态上报或命令响应。
     * 调用方根据cmd参数确定上报的数据类型，并将data指针转换为对应的结构体。
     *
     * @param cmd    命令码/状态类型，用于标识上报数据的结构类型
     *               常见取值：
     *               - 0xE4: PACKET_CMD_CHARGING_STATE (对应 SDK_BACK_CHARGING_STATU_T)
     *               - 0xE5: PACKET_CMD_TASK_STATE (对应 SDK_BACK_TASK_STATU_T)
     *               完整命令码定义参考 @see PACET_CMD_XXX 0xCF后命令
     * @param data   指向上报数据的通用指针，需要根据cmd参数进行类型转换。
     *               使用示例：
     *               @code
     *               switch (cmd) {
     *                   case TCP_CMD::PACKET_CMD_CHARGING_STATE: {
     *                       SDK_BACK_CHARGING_STATU_T *status = (SDK_BACK_CHARGING_STATU_T*)data;
     *                       // 处理充电设备状态
     *                       break;
     *                   }
     *                   case TCP_CMD::PACKET_CMD_TASK_STATE: {
     *                       SDK_BACK_TASK_STATU_T *gps = (SDK_BACK_TASK_STATU_T*)data;
     *                       // 处理任务状态数据
     *                       break;
     *                   }
     *               }
     *               @endcode
     *               注意：data指向的内存仅在回调函数执行期间有效，
     *               如需持久保存数据，请进行深度拷贝。
     * // 定义回调函数
     * int myCallback(uint8_t cmd, const void* data, int data_len) {
     *     if(data_len == 0)
     *     {
     *         return -1;
     *     }
     *     if (cmd == PACKET_CMD_CHARGING_STATE) {
     *         const SDK_BACK_CHARGING_STATU_T* status = (const SDK_BACK_CHARGING_STATU_T*)data;
     *         printf("设备温度: %d°C\n", status->temperature);
     *         return 0;
     *     }
     *     return -1; // 未知命令
     * }
     *
     * @warning 回调函数执行时间应尽可能短，
     *          避免阻塞其他状态上报或导致数据丢失。
     * // 注册回调
     * tbox_sdk_register_callback(myCallback);
     */
    typedef int (*TBoxCallback)(
        uint8_t cmd,
        const void *data,
        int data_len);
    /*
     * @brief 登录应答。
     * 该函数用于获取机器人连接状态。
     * @param statu 连接状态（具体定义见protocol.h/TBOX_CONNECT_E）。
     */
    typedef enum
    {
        CONNECTED = 1,     // 连接成功
        DISCONNECT = 2,    // 断开连接
        LOGIN_SUCCESS = 3, // 登录成功
        LOGIN_FAILED = 4,  // 登录失败
        UNAUTHORIZED = 5,  // 未授权
        AUTH_FAILED = 6,   // 授权失败
        AUTH_EXPIRED = 7,  // 授权过期
        AUTH_SUCCESS = 8   // 授权成功
    } LoginStatus;

    typedef void (*LoginConnectionStatu)(uint8_t statu);
    /**
     * @brief TBOX-SDK 初始化与注册登录接口
     * @details 完成TBOX与云平台的初始连接、设备注册及会话建立。
     *          调用此接口后，SDK将尝试与指定服务器建立连接并完成登录流程。
     *          登录结果通过回调函数异步返回。
     *
     * @param[in] token     SDK授权码
     *                      - 类型：const char *
     *                      - 该授权码由SDK厂商提供
     *
     * @param[in] func      登录状态回调函数指针
     *                      - 类型：LoginConnectionStatu
     *                      - 调用时机：连接建立/断开、登录成功/失败时触发
     *                      - 回调参数：LoginStatus枚举，包含以下状态：
     *                          DISCONNECT    // 连接成功
     *                          CONNECTED     // 连接失败
     *                          LOGIN_SUCCESS      // 登录成功
     *                          LOGIN_FAILED       // 登录失败
     *                          AUTH_FAIL          // 授权失败
     *                          AUTH_EXPIRED       // 授权过期
     *                          AUTH_SUCCESS       // 授权成功
     *                      - 注意：回调函数应避免耗时操作，建议仅做状态更新
     *
     * @note 使用说明：
     *       1. 此为SDK入口函数，应在应用程序启动时第一时间调用
     *       2. 所有参数应在调用前完成有效性校验
     *       3. 函数非阻塞，实际连接在后台线程完成
     *       4. 重复调用可能产生未定义行为，请确保单次初始化
     *
     * @warning 注意事项：
     *       1. 回调函数可能在其他线程被调用，需注意线程安全
     *       2. 网络异常时SDK会自动重连，重连策略请参考相关文档
     *
     */
    TBOX_SDK_API void tbox_sdk_init(const char *token, LoginConnectionStatu func);

    /*
     * @brief 重定位接口
     * @details 用于执行机器人的重定位操作。
     * @param[in] relocation 指向重定位结构体的指针。
     * @param[out] result 重定位结果，指向PSDK_ACK_RELOCATION_T结构体指针
     * @return 运行结果0-成功，-1：失败 -2:返回值空，-3：输入参数错误
     */
    TBOX_SDK_API int tbox_sdk_relocation(PSDK_RELOCATION_T relocation, PSDK_ACK_RELOCATION_T result);

    /**
     * @brief 获取TBOX通信状态接口
     * @details 获取当前TBOX各个通信模块的开关状态、热点SSID和连接的WiFi网络
     *          可用于监控网络状态、诊断连接问题或获取当前配置
     *
     * @return 运行结果0-成功，-1：失败
     *
     * @note 使用说明：
     *       1. 此函数为同步调用，立即返回当前状态
     *       2. 返回的状态为瞬时状态，可能随时间变化
     *       3. 通信状态变化可通过相关回调接口实时获取
     *
     * @par result：返回结构体字段说明：
     * - communication_control：各通信模块的当前开关状态
     * - SSID：当前启用的热点SSID（如热点功能关闭则为空或默认值）
     * - Wifi：当前连接的WiFi网络SSID（如未连接则为空字符串）
     *
     * @warning 线程安全：
     *       1. 此函数内部已做线程同步处理，可多线程安全调用
     *       2. 但返回的结构体为局部拷贝，后续修改不影响实际状态
     *
     * @code
     * // 示例：获取并打印通信状态
     * SDK_COMMUNICATION_T comm_status;
     * int res = tbox_sdk_send_getCommuStatu(&comm_status);
     *
     * printf("通信控制字: 0x%02X\n", comm_status.communication_control);
     * printf("4G状态: %s\n", (comm_status.communication_control & 0x01) ? "开启" : "关闭");
     * printf("WiFi客户端: %s\n", (comm_status.communication_control & 0x02) ? "开启" : "关闭");
     * printf("热点SSID: %s\n", comm_status.SSID);
     * printf("连接WiFi: %s\n", strlen(comm_status.Wifi) > 0 ? comm_status.Wifi : "未连接");
     * @endcode
     *
     * @see SDK_COMMUNICATION_T 详细字段说明
     * @see tbox_sdk_SetCommuConfig 设置通信配置接口
     */
    TBOX_SDK_API int tbox_sdk_send_getCommuStatu(PSDK_ACK_COMMUNICATION_T result);

    /*
     * @brief 地图采集接口
     * @details 机器人地图采集操作，包括增量建图、结束采图以及清除当前地图重新建图；
     * @param[in] cmd 命令定义见SDK_MAP_ACQUISITION_CMD_E。
     * @param[out] result 指向地图采集反馈指针PSDK_ACK_MAP_ACQUISITION_STATUS_T。
     * @return 返回值： 0：操作成功；-1：操作失败；-2:返回值空
     */
    TBOX_SDK_API int tbox_sdk_map_acquisition(SDK_MAP_ACQUISITION_CMD_E cmd, PSDK_ACK_MAP_ACQUISITION_STATUS_T result);
    
    /*
    * @brief 遥控控制
     * @details 控制机器人运动的线速度、角速度以及是否可移动；
     * @param[in] remoteCtrl 指向PSDK_REMOUTE_CONTROL_T的指针
     * @param[out] 无
     * @return 返回值： 0：操作成功；-1：操作失败；
    */
    TBOX_SDK_API int tbox_sdk_remote_control(PSDK_REMOUTE_CONTROL_T remoteCtrl);
    /*
    * @brief 旋转控制
     * @details 控制机器人旋转方式，旋转角度和速度；
     * @param[in] rotationCtrl 指向PSDK_ROTATION_CONTROL_T的指针
     * @param[out] 无
     * @return 返回值： 0：操作成功；-1：操作失败；
    */
    TBOX_SDK_API int tbox_sdk_rotation_control(PSDK_ROTATION_CONTROL_T rotationCtrl);
    /*
    * @brief 举升控制
     * @details 具身智能控制举升柱升降高度；
     * @param[in] hoistCtrl 指向PSDK_HOIST_CONTROL_T的指针
     * @param[out] 无
     * @return 返回值： 0：操作成功；-1：操作失败；
    */
    TBOX_SDK_API int tbox_sdk_hoist_control(PSDK_HOIST_CONTROL_T hoistCtrl);
    /*
    * @brief 举升状态查询
     * @details 具身智能查询举升柱升降高度；
     * @param[in] cmd：查询命令，0x0:位置读取          
     * @param[out] result：指向PSDK_ACK_HOIST_QUERY_T的指针 
     * @return 返回值： 0：操作成功；-1：操作失败；
    */
    TBOX_SDK_API int tbox_sdk_hoist_query(uint8_t cmd, PSDK_ACK_HOIST_QUERY_T result);
    /*
    * @brief 举升电机状态设置
     * @details 具身智能控制举升电机的清除故障码、设置零位；
     * @param[in] hoistSet: 0x0:故障清除;0x1:零位设置;
     * @param[out] 无
     * @return 返回值： 0：操作成功；-1：操作失败；
    */
    TBOX_SDK_API int tbox_sdk_hoist_set(uint8_t hoistSet);
    /*
     * @brief 电源管理接口
     * @details 用于控制机器人的低电量回充电源管理设置
     * @param[in] powerctrl 指向PSDK_POWER_MANAGE_T的指针。
     * @return 返回值： 0：操作成功；-1：操作失败
     */
    TBOX_SDK_API int tbox_sdk_power_manage(PSDK_POWER_MANAGE_T powerctrl);

    /*
     * @brief 获取TBOX版本信息接口
     * @details 用于获取当前TBOX的版本信息。
     * @param[out] result 指向PSDK_ACK_TBOX_VERSION_T的指针。
     * @return  0：操作成功；-1：操作失败；-2:返回值空
     */
    TBOX_SDK_API int tbox_sdk_get_tbox_version(PSDK_ACK_TBOX_VERSION_T result);
    /**
     * @brief 获取机器人硬件标识信息（VIN 和 ICCID）
     * @details 同步获取车辆的唯一识别码（VIN）以及 4G 模块 SIM 卡的集成电路卡识别码（ICCID）。
     * 此信息通常用于云平台设备绑定、身份鉴权及流量查询。
     * * @return 返回值： 0：操作成功；-1：操作失败
     * * @note 使用说明：
     * 1. 该接口为同步调用，数据通常来源于 SDK 缓存或底层固件。
     * 2. 若 TBOX 未插入 SIM 卡或模块未启动，iccid 字段可能返回全 0 或空字符串。
     * 3. VIN 码为固定 17 位字符串，ICCID 通常为 19 或 20 位字符串。
     * * @par result返回结构体SDK_CAR_DEVICE_INFO_T字段：
     * - vin: 车辆唯一身份识别码（符合 GB 16735 标准）。
     * - iccid: SIM 卡唯一标识符，用于追踪移动网络身份。
     * * @code
     * // 示例：获取并显示设备信息
     * SDK_CAR_DEVICE_INFO_T dev_info;
     * int res = tbox_sdk_send_getVinInfo(&dev_info);
     * if (strlen(dev_info.vin) > 0) {
     * printf("机器人 VIN: %s\n", dev_info.vin);
     * printf("SIM 卡 ICCID: %s\n", dev_info.iccid);
     * } else {
     * printf("获取设备信息失败或信息未初始化\n");
     * }
     * @endcode
     * * @see SDK_CAR_DEVICE_INFO_T
     */
    TBOX_SDK_API int tbox_sdk_send_getVinInfo(PSDK_ACK_CAR_DEVICE_INFO_T result);

    /*
     * @brief 位置信息上报接口
     * @details 用于上报机器人的当前位置信息。
     * @param[in] pos 指向位置信息结构体的指针P_SDK_POSITION_T。
     * @return  0：操作成功；-1：操作失败
     */
    TBOX_SDK_API int tbox_sdk_position(PSDK_POSITION_T pos);

    /**
     * @brief TBOX 任务控制接口
     * @details 对当前正在执行的任务进行实时控制，包括取消、暂停、恢复等操作。
     *          此接口提供对已下发任务的运行时管理能力，适用于任务中断、临时调整等场景。
     *
     * @param[in] ctrl 任务控制命令，取值范围：0-2
     *                - 0：任务取消 (Task Cancel)
     *                  立即停止当前任务，机器人停止运动并清除任务队列。
     *                  使用场景：紧急停止、任务作废、重新规划等。
     *                - 1：任务暂停 (Task Pause)
     *                  暂停当前任务执行，机器人保持当前位置和状态。
     *                  使用场景：临时避让、人工干预、等待资源等。
     *                - 2：任务恢复 (Task Resume)
     *                  从暂停状态恢复任务执行，机器人继续执行未完成的任务。
     *                  使用场景：暂停后继续、人工确认后恢复等。
     *
     * @return  0：操作成功；-1：操作失败
     */
    TBOX_SDK_API int tbox_sdk_task_control(uint8_t ctrl);

    /**
     * @brief 执行回桩任务
     * @details 控制机器人自动返回充电桩进行充电或停放。
     *          此接口触发后，机器人将执行以下流程：
     *          1. 规划从当前位置到充电桩的最优路径
     *          2. 沿规划路径自主导航至充电桩
     *          3. 精确对接充电桩并进行充电连接
     *          4. 进入充电状态或待机状态
     *
     * @note 使用说明：
     *       1. 调用前需确保：
     *          - 充电桩已在地图中正确标定
     *          - 机器人当前位置已知且定位正常
     *          - 充电桩区域无障碍物阻挡
     *       2. 回桩任务会中断当前正在执行的任务
     *       3. 回桩成功后，机器人将自动进入充电模式
     *       4. 任务执行状态可通过任务状态回调实时监控
     *
     * @warning 注意事项：
     *       1. 低电量（如<20%）时建议尽快执行回桩任务
     *       2. 回桩过程中如遇障碍物，机器人会尝试绕行或暂停
     *       3. 充电桩被占用或故障时，回桩任务将失败
     *       4. 紧急情况下可通过 tbox_sdk_task_control(0) 取消回桩
     *
     * @return 0：操作成功；-1：操作失败
     */
    TBOX_SDK_API int tbox_sdk_go_home();

    /**
     * @brief TBOX 任务下发至目标点接口
     * @details 向TBOX下发导航、移动、顶升等控制任务。
     *          任务格式根据TBOX版本（task_version）自动适配。
     *          任务下发后，TBOX将执行对应的路径规划和控制指令。
     *
     * @param[in] task 任务参数结构体，包含目标点、移动参数等配置
     *
     * @return 执行状态码
     * @retval 0  任务下发成功
     * @retval -1 失败（如坐标超出范围、版本不支持等）
     *
     * @note 使用说明：
     *       1. 调用前请确保TBOX已成功登录（tbox_sdk_init）
     *       2. 任务为异步执行，接口返回仅表示任务是否成功下发
     *       3. 同一时刻只能有一个任务在执行，新任务会覆盖未完成的任务
     *       4. 任务执行状态可通过相关回调接口获取
     *
     * @warning 注意事项：
     *       1. 任务坐标需在当前地图坐标系内，否则会执行失败
     *       2. 连续下发任务需间隔至少100ms，避免任务队列溢出
     *       3. 部分参数仅对特定TBOX版本有效，请根据实际版本设置
     *
     * @see sdk_type.h
     * @see SDK_TASK_DISTRIBUTION_T
     */
    TBOX_SDK_API int tbox_sdk_task_distribution(PSDK_TASK_DISTRIBUTION_T task);

    /**
     * @brief 执行返回开机点任务
     * @details 控制机器人自动返回预设的开机点（初始位置）进行待机或休眠。
     *          此接口触发后，机器人将执行以下流程：
     *          1. 停止当前所有任务（如导航、充电等）
     *          2. 规划从当前位置到开机点的最优路径
     *          3. 沿规划路径自主导航至开机点
     *          4. 在开机点精确停靠并进入待机状态
     *
     * @note 使用说明：
     *       1. 调用前需确保：
     *          - 开机点已在地图中正确标定
     *          - 机器人当前位置已知且定位正常
     *          - 开机点区域无障碍物阻挡
     *       2. 回开机点任务会中断当前正在执行的任务
     *       3. 到达开机点后，机器人将保持待机状态，可随时接受新任务
     *       4. 任务执行状态可通过任务状态回调实时监控
     *     	 5. 如需重启可执行回开机点任务保证开机重定位正常
     *
     * @warning 注意事项：
     *       1. 机器人可能不会在开机点进行充电，如需充电请调用tbox_sdk_go_home()
     *       2. 回开机点过程中如遇障碍物，机器人会尝试绕行或暂停
     *       3. 开机点被占用或无法到达时，任务将失败
     *       4. 紧急情况下可通过tbox_sdk_task_control(0)取消任务
     *
     * @return 0：操作成功；-1：操作失败
     */
    TBOX_SDK_API int tbox_sdk_go_start();

    /*
     * @brief 清除故障信息
     * @details 用于清除机器人当前的故障记录，恢复到正常状态。
     * @return 0：操作成功；-1：操作失败
     */
    TBOX_SDK_API int tbox_sdk_clear_fault();

    /**
     * @brief 设置 TBOX 通信配置接口
     * @details 用于控制 TBOX 各通信模块（4G、WiFi、热点等）的开关状态，以及配置热点 SSID 和 WiFi 连接参数。
     * 该接口通过 `SDK_COMMUNICATION_CONTROL_T` 结构体下发指令。
     * * @param[in] ctrl 指向通信控制结构体的指针，包含指令类型及必要的 SSID/密码信息。
     * * @return 执行结果状态码
     * @retval  0：操作成功；
     * @retval -1：操作失败
     * * @note 使用说明：
     * 1. 这是一个同步/半同步接口，建议在配置生效后通过 tbox_sdk_send_getCommuStatu() 核实。
     * 2. 修改 WiFi 或热点配置可能会导致当前的 IP 连接短暂中断。
     * 3. 如果仅需开关模块而不修改配置，SSID 和 pass_wd 字段可保持为空。
     * * @warning 注意事项：
     * 1. 密码字段建议在内存中使用后立即抹除，避免敏感信息残留。
     * 2. 频繁调用切换通信状态可能导致底层硬件模块异常，建议调用间隔不小于 1s。
     * * @code
     * // 示例：设置 WiFi 连接
     * SDK_COMMUNICATION_CONTROL_T wifi_config;
     * memset(&wifi_config, 0, sizeof(wifi_config));
     * wifi_config.communication_control = SDK_COMMUNICATION_CTRL_CMD_E::CONNECT_WIFI;
     * strncpy(wifi_config.SSID, "Office_WiFi", sizeof(wifi_config.SSID));
     * strncpy(wifi_config.pass_wd, "12345678", sizeof(wifi_config.pass_wd));
     * * if(tbox_sdk_communication_control(&wifi_config) == 0) {
     * printf("WiFi 配置指令发送成功\n");
     * }
     * @endcode
     * * @see SDK_COMMUNICATION_CONTROL_T
     * @see tbox_sdk_send_getCommuStatu
     */
    TBOX_SDK_API int tbox_sdk_communication_control(PSDK_COMMUNICATION_CONTROL_T ctrl);

    /*
     * @brief EAI任务状态上报
     * @details 用于上报EAI任务的执行状态。
     * @param[in] statu 指向任务状态结构体PSDK_EAI_TASK_STATU_T的指针。
     * @return 0：操作成功；-1：操作失败
     */
    TBOX_SDK_API int tbox_sdk_EAITask_statu_report(PSDK_EAI_TASK_STATU_T statu);

    /*
     * @brief 获取动态参数列表
     * @details 用于获取当前可用的动态参数列表。
     * @param[out] list 指向动态参数列表结构体的指针，用于存储获取到的参数信息。
     * @return 获取结果状态码
     * @retval 0  获取成功，list 中包含有效数据
     * @retval -1 失败
     * @retval -2 返回值空
     * @retval -3 传入参数错误
     * 
     */
    TBOX_SDK_API int tbox_sdk_dynamic_param_get(PSDK_DYNAMIC_PARAM_LIST_T list);

    /**
     * @brief 设置动态参数列表
     * @details 用于设置当前的动态参数列表。
     * @param[in] list 指向动态参数列表结构体的指针，包含要设置的参数信息。
     * @return 设置结果状态码
     * @retval 0  设置成功
     * @retval -1 失败
     * @retval -2 返回值空
     * @retval -3 传入参数错误
     */
    TBOX_SDK_API int tbox_sdk_dynamic_param_set(PSDK_DYNAMIC_PARAM_LIST_T list);

    /*
     * @brief 刹车控制接口
     * @details 用于控制机器人的刹车状态。
     * @param[in] brake_ctrl 指向刹车控制参数 0:拉起；1：释放。
     * @return 0：操作成功；-1：操作失败
     */
    TBOX_SDK_API int tbox_sdk_brake_control(uint8_t brake_ctrl);

    /**
     * @brief 注册TBOX状态上报回调函数
     * @details 注册用户自定义的回调函数，用于接收TBOX的异步状态上报、数据推送和命令响应。
     *          SDK在接收到TBOX上报的各种状态信息（如任务状态、充电状态、设备状态等）时，
     *          会通过此回调函数通知应用程序。
     *
     * @param[in] cb 用户定义的回调函数指针
     *              - 类型：TBoxCallback
     *
     * @note 使用说明：
     *       1. 建议在调用tbox_sdk_init成功登录前注册回调函数
     *       2. 回调函数将在SDK内部线程中被调用，需注意线程安全问题
     * @par 注册时机：
     * - 推荐顺序：注册回调 → 初始化 → 登录成功 → 开始业务
     *
     */
    TBOX_SDK_API void tbox_sdk_register_callback(TBoxCallback cb);

#ifdef __cplusplus
}

#endif
#endif