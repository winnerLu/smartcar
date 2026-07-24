// car_base 主节点:STM32 底盘串口通信 + 里程计 + TF。
//
// 订阅:  /cmd_vel        (geometry_msgs/Twist)   -> 发 11 字节控制帧
// 发布:  /wheel/odom     (nav_msgs/Odometry)     <- 积分反馈 vx/wz
//        /imu/data_raw   (sensor_msgs/Imu)        <- IMU(到货后启用)
//        /battery        (std_msgs/Float32)       <- 电池电压(待标定)
// TF:    odom -> base_link
//
// 关键设计:
//   - 差速运动学在 STM32 内部完成,本节点直接积分车体速度 vx/wz,
//     里程计不需要轮距/轮径。
//   - 200ms 未收到 /cmd_vel 则自动下发零速(超时保护)。
//   - 角速度符号/幅值、电压均可通过参数标定(见标定记录)。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "car_base/protocol.hpp"
#include "car_base/serial_port.hpp"

using namespace std::chrono_literals;

namespace car_base
{

class CarBaseNode : public rclcpp::Node
{
public:
  CarBaseNode()
  : Node("car_base")
  {
    // ---- 参数 ----
    device_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_ = declare_parameter<int>("baud", 115200);
    cmd_vx_sign_ = declare_parameter<int>("cmd_vx_sign", 1);      // 线速度控制符号(实测)
    cmd_wz_sign_ = declare_parameter<int>("cmd_wz_sign", 1);      // 实测 +1
    odom_vx_sign_ = declare_parameter<int>("odom_vx_sign", 1);    // 里程计线速度符号(实测)
    odom_wz_sign_ = declare_parameter<int>("odom_wz_sign", 1);    // 实测 +1
    odom_wz_scale_ = declare_parameter<double>("odom_wz_scale", 1.0);  // 待标定(反馈约 3 倍)
    odom_vx_scale_ = declare_parameter<double>("odom_vx_scale", 1.0);  // 待标定
    voltage_scale_ = declare_parameter<double>("voltage_scale", 1.0);  // 待标定
    cmd_timeout_ms_ = declare_parameter<int>("cmd_timeout_ms", 200);
    max_vx_ = declare_parameter<double>("max_vx", 0.5);   // 线速度硬上限,防溢出/危险速度
    max_wz_ = declare_parameter<double>("max_wz", 2.0);   // 角速度硬上限
    debug_tx_ = declare_parameter<bool>("debug_tx", false);  // 打印发送帧十六进制
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");  // 订阅话题
    publish_imu_ = declare_parameter<bool>("publish_imu", false);     // IMU 未到货,默认关
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    // ---- 串口 ----
    std::string err;
    if (!serial_.open_port(device_, baud_, err)) {
      RCLCPP_ERROR(get_logger(), "打开串口 %s 失败: %s", device_.c_str(), err.c_str());
      RCLCPP_ERROR(get_logger(), "检查: 设备名/权限(dialout 组)/是否上电");
      throw std::runtime_error("串口打开失败");
    }
    RCLCPP_INFO(get_logger(), "已打开串口 %s @ %d", device_.c_str(), baud_);

    // ---- 发布/订阅 ----
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("wheel/odom", 20);
    battery_pub_ = create_publisher<std_msgs::msg::Float32>("battery", 5);
    if (publish_imu_) {
      imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 20);
    }
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 订阅话题可配:默认cmd_vel；仅仲裁时为cmd_vel_raw；
    // 仲裁+碰撞监控时为cmd_vel_safe。
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      std::bind(&CarBaseNode::on_cmd_vel, this, std::placeholders::_1));

    last_cmd_time_ = now();
    last_odom_time_ = now();

    // ---- 定时器 ----
    // 读串口:高频轮询
    read_timer_ = create_wall_timer(5ms, std::bind(&CarBaseNode::on_read, this));
    // 发控制帧:20Hz(含超时保护)
    send_timer_ = create_wall_timer(50ms, std::bind(&CarBaseNode::on_send, this));

    RCLCPP_INFO(get_logger(), "car_base 节点已启动");
  }

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(cmd_mtx_);
    target_vx_ = msg->linear.x;
    target_wz_ = msg->angular.z;
    last_cmd_time_ = now();
  }

  void on_send()
  {
    double vx, wz;
    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      // 超时保护:超过 cmd_timeout_ms 未收到新指令则归零
      const double dt_ms = (now() - last_cmd_time_).seconds() * 1000.0;
      if (dt_ms > cmd_timeout_ms_) {
        target_vx_ = 0.0;
        target_wz_ = 0.0;
      }
      vx = target_vx_;
      wz = target_wz_;
    }
    // 限幅:兜底防止过大速度(误操作/Nav2 异常)导致 STM32 内部溢出或危险运动
    vx = std::clamp(vx, -max_vx_, max_vx_);
    wz = std::clamp(wz, -max_wz_, max_wz_);
    auto frame = build_ctrl_frame(vx, wz, cmd_wz_sign_, cmd_vx_sign_);
    if (debug_tx_ && (vx != 0.0 || wz != 0.0)) {
      char hex[64];
      int p = 0;
      for (size_t i = 0; i < frame.size(); ++i) {
        p += snprintf(hex + p, sizeof(hex) - p, "%02X ", frame[i]);
      }
      RCLCPP_INFO(get_logger(), "TX vx=%.3f wz=%.3f -> %s", vx, wz, hex);
    }
    if (!serial_.write_all(frame.data(), frame.size())) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "串口写失败");
    }
  }

  void on_read()
  {
    uint8_t buf[256];
    ssize_t n = serial_.read_some(buf, sizeof(buf));
    if (n <= 0) {return;}
    auto frames = reader_.feed(buf, static_cast<size_t>(n));
    for (const auto & fb : frames) {
      process_feedback(fb);
    }
  }

  void process_feedback(const Feedback & fb)
  {
    const rclcpp::Time t = now();
    const double dt = (t - last_odom_time_).seconds();
    last_odom_time_ = t;
    if (dt <= 0.0 || dt > 1.0) {
      // 首帧或异常间隔,跳过积分
      return;
    }

    const double vx = odom_vx_sign_ * fb.vx * odom_vx_scale_;
    const double wz = odom_wz_sign_ * fb.wz * odom_wz_scale_;

    // 差速里程计积分(2D)
    const double delta_theta = wz * dt;
    // 用中点航向积分,降低误差
    const double mid_theta = theta_ + delta_theta * 0.5;
    x_ += vx * std::cos(mid_theta) * dt;
    y_ += vx * std::sin(mid_theta) * dt;
    theta_ += delta_theta;

    // 归一化到 [-pi, pi]
    while (theta_ > M_PI) {theta_ -= 2.0 * M_PI;}
    while (theta_ < -M_PI) {theta_ += 2.0 * M_PI;}

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);

    // ---- 发布 odom ----
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.angular.z = wz;
    // 简单协方差(EKF 会重估;给非零对角避免退化)
    odom.pose.covariance[0] = 0.01;    // x
    odom.pose.covariance[7] = 0.01;    // y
    odom.pose.covariance[35] = 0.05;   // yaw
    odom.twist.covariance[0] = 0.01;
    odom.twist.covariance[35] = 0.05;
    odom_pub_->publish(odom);

    // ---- 发布 TF ----
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = t;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = x_;
      tf.transform.translation.y = y_;
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(tf);
    }

    // ---- 电池 ----
    std_msgs::msg::Float32 bat;
    bat.data = static_cast<float>(fb.voltage_raw * voltage_scale_);
    battery_pub_->publish(bat);

    // ---- IMU(到货后启用)----
    if (publish_imu_ && imu_pub_) {
      sensor_msgs::msg::Imu imu;
      imu.header.stamp = t;
      imu.header.frame_id = base_frame_;
      imu.linear_acceleration.x = fb.acc[0];
      imu.linear_acceleration.y = fb.acc[1];
      imu.linear_acceleration.z = fb.acc[2];
      imu.angular_velocity.x = fb.gyro[0];
      imu.angular_velocity.y = fb.gyro[1];
      imu.angular_velocity.z = fb.gyro[2];
      // 无姿态解算,方向协方差置 -1 表示不可用
      imu.orientation_covariance[0] = -1.0;
      imu_pub_->publish(imu);
    }
  }

  // 参数
  std::string device_, odom_frame_, base_frame_, cmd_vel_topic_;
  int baud_, cmd_vx_sign_, cmd_wz_sign_, odom_vx_sign_, odom_wz_sign_, cmd_timeout_ms_;
  double max_vx_, max_wz_;
  bool debug_tx_;
  double odom_wz_scale_, odom_vx_scale_, voltage_scale_;
  bool publish_imu_, publish_tf_;

  // 串口 + 协议
  SerialPort serial_;
  FrameReader reader_;

  // 指令状态
  std::mutex cmd_mtx_;
  double target_vx_ = 0.0, target_wz_ = 0.0;
  rclcpp::Time last_cmd_time_;

  // 里程计状态
  double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
  rclcpp::Time last_odom_time_;

  // ROS 接口
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr read_timer_, send_timer_;
};

}  // namespace car_base

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<car_base::CarBaseNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("car_base"), "节点异常退出: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
