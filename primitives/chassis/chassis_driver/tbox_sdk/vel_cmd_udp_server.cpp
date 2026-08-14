#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cmath>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

// ================= 配置 =================
static const int         UDP_PORT                = 11451;
static const char*       CMD_VEL_TOPIC           = "/cmd_vel/input/manual";
static const char*       MODE_TOPIC              = "/Mode";
static const int8_t      MANUAL_MODE_VALUE       = 1;
static const double      CONT_TIMEOUT_SEC        = 0.075;   // type=1 连续模式超时 75ms

static const double      SINGLE_LINEAR_SPEED     = 0.5;     // forward_m 模式默认线速度 (m/s)
static const double      SINGLE_ANGULAR_SPEED    = 0.5;     // rotate_deg 模式默认角速度 (rad/s)
static const double      DEFAULT_SINGLE_DURATION = 1.0;     // 速度直通模式默认时长 (s)
// ========================================

// 指令数据结构
struct Command
{
  double linear_x, linear_y, linear_z;
  double angular_x, angular_y, angular_z;
  double duration_sec;
  double forward_m;
  double rotate_deg;
  int32_t type_code;  // 0=单次, 1=连续
};

class CmdVelUDPServer
{
  ros::NodeHandle nh_;
  ros::Publisher  pub_;
  ros::Publisher  mode_pub_;
  ros::Subscriber mode_sub_;

  // 最新指令槽（接收线程写入，执行线程读取）
  Command latest_cmd_;
  bool has_new_cmd_ = false;
  std::mutex cmd_mutex_;
  std::condition_variable cmd_cv_;
  std::atomic<bool> running_{true};

  // 当前模式：0=自动导航, 1=manual
  std::atomic<int8_t> current_mode_{1};

  std::thread recv_thread_;
  std::thread exec_thread_;

public:
  CmdVelUDPServer()
    : pub_(nh_.advertise<geometry_msgs::Twist>(CMD_VEL_TOPIC, 1))
    , mode_pub_(nh_.advertise<std_msgs::Int8>(MODE_TOPIC, 1))
    , mode_sub_(nh_.subscribe(MODE_TOPIC, 1, &CmdVelUDPServer::modeCallback, this))
  {
  }

  ~CmdVelUDPServer()
  {
    running_ = false;
    cmd_cv_.notify_all();
    if (recv_thread_.joinable())  recv_thread_.join();
    if (exec_thread_.joinable())  exec_thread_.join();
  }

  void start()
  {
    recv_thread_ = std::thread(&CmdVelUDPServer::receiveLoop, this);
    exec_thread_ = std::thread(&CmdVelUDPServer::executeLoop, this);
    ROS_INFO("CmdVel UDP server started: recv + exec threads");
  }

private:
  // ---------------- Mode话题回调：监听外部模式切换 ----------------
  void modeCallback(const std_msgs::Int8::ConstPtr& msg)
  {
    int8_t old_mode = current_mode_;
    current_mode_ = msg->data;

    // 只在外部切换时输出日志（避免与内部切换重复）
    if (old_mode != current_mode_)
    {
      ROS_INFO("Mode changed externally to %d", (int)current_mode_);
    }
  }

  // ---------------- 辅助函数：切换到manual模式 ----------------
  void switchToManualMode()
  {
    std_msgs::Int8 mode_msg;
    mode_msg.data = MANUAL_MODE_VALUE;
    mode_pub_.publish(mode_msg);
    current_mode_ = MANUAL_MODE_VALUE;  // 立即更新本地状态
    ROS_INFO("Switched to manual control mode");
  }

  // ---------------- 辅助函数：切换到自动导航模式 ----------------
  void switchToAutonomousMode()
  {
    // 先将manual通道速度归零
    geometry_msgs::Twist zero;
    pub_.publish(zero);

    // 再切换mode
    std_msgs::Int8 mode_msg;
    mode_msg.data = 0;
    mode_pub_.publish(mode_msg);
    current_mode_ = 0;  // 立即更新本地状态
    ROS_INFO("Switched to autonomous navigation mode");
  }

  // ---------------- UDP 接收线程 ----------------
  void receiveLoop()
  {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
      ROS_ERROR("Failed to create UDP socket");
      return;
    }

    // 0.5s 接收超时，便于轮询 running_
    timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 500000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(UDP_PORT);

    if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
    {
      ROS_ERROR("Failed to bind UDP socket on port %d", UDP_PORT);
      close(sock);
      return;
    }

    ROS_INFO("UDP server listening on 0.0.0.0:%d", UDP_PORT);

    uint8_t buffer[76];
    while (running_)
    {
      socklen_t addr_len = sizeof(addr);
      ssize_t n = recvfrom(sock, buffer, 76, 0,
                           reinterpret_cast<sockaddr*>(&addr), &addr_len);
      if (n < 0)
        continue;  // 超时或错误，继续循环

      if (n != 76)
      {
        ROS_WARN("Ignoring packet of length %zd", n);
        continue;
      }

      // 解析: 前72字节 9个float64(小端), 后4字节 int32
      double   move_cmd[9];
      int32_t  type_code;
      std::memcpy(move_cmd, buffer,       72);
      std::memcpy(&type_code, buffer + 72, 4);

      // 构造指令并更新最新指令槽（覆盖旧指令）
      Command cmd;
      cmd.linear_x    = move_cmd[0];
      cmd.linear_y    = move_cmd[1];
      cmd.linear_z    = move_cmd[2];
      cmd.angular_x   = move_cmd[3];
      cmd.angular_y   = move_cmd[4];
      cmd.angular_z   = move_cmd[5];
      cmd.duration_sec= move_cmd[6];
      cmd.forward_m   = move_cmd[7];
      cmd.rotate_deg  = move_cmd[8];
      cmd.type_code   = type_code;

      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        latest_cmd_ = cmd;
        has_new_cmd_ = true;
      }
      cmd_cv_.notify_one();
    }

    close(sock);
  }

  // ---------------- 执行线程（处理指令并发送消息）----------------
  void executeLoop()
  {
    // 连续模式的状态跟踪
    bool in_continuous_mode = false;
    ros::Time last_continuous_time;

    while (running_)
    {
      Command cmd;
      bool has_cmd = false;

      // 1. 尝试获取最新指令（带超时等待）
      {
        std::unique_lock<std::mutex> lock(cmd_mutex_);
        if (cmd_cv_.wait_for(lock, std::chrono::milliseconds(10),
                               [this]{ return has_new_cmd_ || !running_; }))
        {
          if (!running_) break;
          if (has_new_cmd_)
          {
            cmd = latest_cmd_;
            has_new_cmd_ = false;
            has_cmd = true;
          }
        }
      }

      // 2. 如果收到新指令，执行相应逻辑
      if (has_cmd)
      {
        // 2.1 检查并确保Mode为1（manual模式）
        if (current_mode_ != 1)
        {
          switchToManualMode();
        }

        // 2.2 执行控制命令
        if (cmd.type_code == 0)
        {
          // 单次控制
          executeSingleCommand(cmd);
          in_continuous_mode = false;

          // 单次控制结束后，切换到自动导航模式
          switchToAutonomousMode();
          ROS_INFO("Single command completed, switched to autonomous mode");
        }
        else if (cmd.type_code == 1)
        {
          // 连续控制：立即发送
          executeContinuousCommand(cmd);
          in_continuous_mode = true;
          last_continuous_time = ros::Time::now();
        }
      }
      else
      {
        // 3. 无新指令，检查连续模式超时
        if (in_continuous_mode)
        {
          ros::Time now = ros::Time::now();
          if ((now - last_continuous_time).toSec() > CONT_TIMEOUT_SEC)
          {
            in_continuous_mode = false;
            // 连续模式超时，切换到自动导航模式
            switchToAutonomousMode();
            ROS_INFO("Continuous mode timeout, switched to autonomous mode");
          }
        }
      }
    }
  }

  // ---------------- 单次控制处理 ----------------
  void executeSingleCommand(const Command& cmd)
  {
    geometry_msgs::Twist twist;
    double duration = DEFAULT_SINGLE_DURATION;

    // 优先级1: 前进/后退一定距离
    if (cmd.forward_m != 0.0)
    {
      double speed = (cmd.linear_x == 0.0) ? SINGLE_LINEAR_SPEED : std::abs(cmd.linear_x);
      twist.linear.x = (cmd.forward_m > 0) ? speed : -speed;
      duration = std::abs(cmd.forward_m) / speed;
    }
    // 优先级2: 原地旋转一定角度
    else if (cmd.rotate_deg != 0.0)
    {
      double speed = (cmd.angular_z == 0.0) ? SINGLE_ANGULAR_SPEED : std::abs(cmd.angular_z);
      twist.angular.z = (cmd.rotate_deg > 0) ? speed : -speed;
      duration = std::abs(cmd.rotate_deg * M_PI / 180.0) / speed;
    }
    // 优先级3: 直接速度直通
    else
    {
      twist.linear.x  = cmd.linear_x;
      twist.linear.y  = cmd.linear_y;
      twist.linear.z  = cmd.linear_z;
      twist.angular.x = cmd.angular_x;
      twist.angular.y = cmd.angular_y;
      twist.angular.z = cmd.angular_z;
      if (cmd.duration_sec > 0)
        duration = cmd.duration_sec;
    }

    // 发送一次控制指令
    pub_.publish(twist);
    ROS_INFO("Single command sent, duration=%.2fs", duration);

    // 等待duration时间
    ros::Duration(duration).sleep();

    ROS_INFO("Single command duration ended");
  }

  // ---------------- 连续控制处理 ----------------
  void executeContinuousCommand(const Command& cmd)
  {
    geometry_msgs::Twist twist;
    twist.linear.x  = cmd.linear_x;
    twist.linear.y  = cmd.linear_y;
    twist.linear.z  = cmd.linear_z;
    twist.angular.x = cmd.angular_x;
    twist.angular.y = cmd.angular_y;
    twist.angular.z = cmd.angular_z;

    // 发送一次控制指令
    pub_.publish(twist);
  }

  // ---------------- 成员变量 ----------------

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_vel_udp_server");
  CmdVelUDPServer server;
  server.start();
  ros::spin();
  return 0;
}

