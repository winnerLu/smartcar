// car_camera 节点:USB 摄像头(罗技 C270)采集与发布。
//
// 发布:  /camera/image_raw    (sensor_msgs/Image, bgr8)   <- cv_bridge 转换
//        /camera/camera_info  (sensor_msgs/CameraInfo)    <- 支持从 camera_info_url 加载标定
// TF:    base_link -> camera_link (由 launch 里的 static_transform_publisher 发布)
//
// 关键设计:
//   - C270 为标准 UVC 免驱摄像头,用 cv::VideoCapture 直接读取(V4L2 后端)。
//   - MJPG 模式 640x480@30fps 最稳;YUYV 在高分辨率下受 USB2 带宽限制会掉帧。
//   - 通过 camera_info_manager 支持 camera_info_url 参数加载标定文件;
//     未提供 url 时内参置零,仅填尺寸与 frame_id。
//   - cap.read() 阻塞到下一帧,故 timer 实际以摄像头帧率运行;本节点无其它订阅,
//     单线程 executor 即可。后续若加订阅需改成独立抓帧线程。

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"
#include "camera_info_manager/camera_info_manager.hpp"
#include "opencv2/videoio.hpp"

using namespace std::chrono_literals;

namespace car_camera
{

class CarCameraNode : public rclcpp::Node
{
public:
  CarCameraNode()
  : Node("car_camera")
  {
    // ---- 参数 ----
    video_device_ = declare_parameter<std::string>("video_device", "/dev/camera_c270");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_link");
    image_width_ = declare_parameter<int>("image_width", 640);
    image_height_ = declare_parameter<int>("image_height", 480);
    framerate_ = declare_parameter<int>("framerate", 30);
    pixel_format_ = declare_parameter<std::string>("pixel_format", "mjpg");  // mjpg / yuyv
    // camera_name 用于 camera_info_manager 识别配置文件
    declare_parameter<std::string>("camera_name", "logitech_c270");
    const std::string camera_name = get_parameter("camera_name").as_string();
    const std::string camera_info_url = declare_parameter<std::string>("camera_info_url", "");

    // ---- 打开摄像头 ----
    // 用字符串设备路径 + V4L2 后端;设备名由 udev 固定为 /dev/camera_c270
    cap_.open(video_device_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "打开摄像头 %s 失败", video_device_.c_str());
      RCLCPP_ERROR(get_logger(), "检查: 设备名/权限/USB 接入/udev 规则是否部署");
      throw std::runtime_error("摄像头打开失败");
    }

    // ---- 配置帧格式(C270 不一定全接受,以回读值为准)----
    if (pixel_format_ == "mjpg") {
      cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    }
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(image_width_));
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(image_height_));
    cap_.set(cv::CAP_PROP_FPS, static_cast<double>(framerate_));

    image_width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    image_height_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    const double real_fps = cap_.get(cv::CAP_PROP_FPS);
    RCLCPP_INFO(get_logger(), "已打开 %s: %dx%d @ %.1f fps (%s)",
      video_device_.c_str(), image_width_, image_height_, real_fps, pixel_format_.c_str());

    // ---- camera_info 管理器 ----
    // 若提供了标定文件 URL,则发布真实内参;否则发布默认空内参(宽高 + frame_id)
    cinfo_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      this, camera_name, camera_info_url);
    if (camera_info_url.empty()) {
      RCLCPP_WARN(get_logger(), "未提供 camera_info_url,发布的 camera_info 内参为零,请尽快标定");
    } else if (!cinfo_->isCalibrated()) {
      RCLCPP_WARN(get_logger(), "camera_info_url 加载失败或内参无效: %s", camera_info_url.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "已从 %s 加载相机标定参数", camera_info_url.c_str());
    }

    // ---- 发布 ----
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_raw", 10);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", 10);

    // ---- 抓帧定时器 ----
    const int period_ms = std::max(1, 1000 / std::max(1, framerate_));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms), std::bind(&CarCameraNode::on_capture, this));

    RCLCPP_INFO(get_logger(), "car_camera 节点已启动");
  }

private:
  void on_capture()
  {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "抓帧失败(空帧)");
      return;
    }

    const rclcpp::Time t = now();

    // MJPG 由 OpenCV 解码为 BGR,故编码 bgr8
    cv_bridge::CvImage cv_img;
    cv_img.image = frame;
    cv_img.encoding = "bgr8";
    cv_img.header.stamp = t;
    cv_img.header.frame_id = frame_id_;
    // toImageMsg() 返回 SharedPtr;Humble 的 publish() 无 SharedPtr 重载,解引用传 const 引用
    auto img_msg = cv_img.toImageMsg();
    image_pub_->publish(*img_msg);

    // CameraInfo:从 camera_info_manager 获取,未标定则只保留尺寸与 frame_id
    sensor_msgs::msg::CameraInfo info = cinfo_->getCameraInfo();
    info.header.stamp = t;
    info.header.frame_id = frame_id_;
    info.width = image_width_;
    info.height = image_height_;
    info_pub_->publish(info);
  }

  // 参数
  std::string video_device_, frame_id_, pixel_format_;
  int image_width_ = 640;
  int image_height_ = 480;
  int framerate_ = 30;

  // OpenCV
  cv::VideoCapture cap_;

  // ROS 接口
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> cinfo_;
};

}  // namespace car_camera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<car_camera::CarCameraNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("car_camera"), "节点异常退出: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
