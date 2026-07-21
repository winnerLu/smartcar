// car_camera Apriltag 检测节点。
//
// 订阅: /camera/image_raw    (sensor_msgs/Image, bgr8)
//       /camera/camera_info  (sensor_msgs/CameraInfo) <- 必须有 fx/fy/cx/cy
// 发布: /tag_pose            (geometry_msgs/PoseStamped)
// TF:  camera_link -> tag_0  (由本节点发布)
//
// 关键设计:
//   - 基于 AprilRobotics/apriltag C 库,只识别 tag36h11 家族。
//   - 收到 camera_info 后缓存内参;缺内参时不做位姿估计,仅日志提醒。
//   - 默认只处理 ID 0(与 AprilTag_tag36h11_ID0/ 标签对应),可通过参数 tag_id 改为 -1 识别全部。
//   - 位姿估计用 OpenCV solvePnP(避免系统 apriltag 库未导出 matd_destroy 的链接问题)。
//   - CLAHE 对比度增强提升低光照场景检测率。
//   - quad_decimate=1.0 保证小标签不丢失。
//   - 坐标系约定: 相机光学坐标系,z 朝前,x 右,y 下。

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"

namespace car_camera
{

class AprilTagDetectorNode : public rclcpp::Node
{
public:
  AprilTagDetectorNode()
  : Node("apriltag_detector")
  {
    // ---- 参数 ----
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    tag_family_ = declare_parameter<std::string>("tag_family", "tag36h11");
    tag_size_ = declare_parameter<double>("tag_size", 0.120);  // 米,打印后请实测替换
    target_tag_id_ = declare_parameter<int>("tag_id", 0);      // -1 表示不筛选,检测所有 ID
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "tag_0");

    // ---- Apriltag 检测器初始化 ----
    if (tag_family_ != "tag36h11") {
      RCLCPP_ERROR(get_logger(), "本节点当前仅支持 tag36h11");
      throw std::runtime_error("不支持的 tag family");
    }
    tf_ = tag36h11_create();
    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, tf_);

    // quad_decimate=1.0 不降采样,最大灵敏度;低性能设备可调回 2.0
    td_->quad_decimate = declare_parameter<double>("quad_decimate", 1.0);
    td_->quad_sigma = declare_parameter<double>("quad_sigma", 0.0);
    td_->nthreads = declare_parameter<int>("nthreads", 2);
    td_->debug = 0;
    td_->refine_edges = 1;

    // ---- ROS 接口 ----
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10, std::bind(&AprilTagDetectorNode::on_image, this, std::placeholders::_1));
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10, std::bind(&AprilTagDetectorNode::on_camera_info, this, std::placeholders::_1));

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/tag_pose", 10);

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    RCLCPP_INFO(get_logger(), "Apriltag 检测节点已启动,等待 %s ...", camera_info_topic_.c_str());
  }

  ~AprilTagDetectorNode()
  {
    apriltag_detector_destroy(td_);
    tag36h11_destroy(tf_);
  }

private:
  void on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (has_camera_info_) {
      return;
    }
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];

    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
      fx_, 0.0, cx_,
      0.0, fy_, cy_,
      0.0, 0.0, 1.0);

    dist_coeffs_ = cv::Mat::zeros(1, static_cast<int>(msg->d.size()), CV_64F);
    for (size_t i = 0; i < msg->d.size(); ++i) {
      dist_coeffs_.at<double>(0, static_cast<int>(i)) = msg->d[i];
    }

    has_camera_info_ = true;
    RCLCPP_INFO(get_logger(), "相机内参已获取: fx=%.3f fy=%.3f cx=%.3f cy=%.3f", fx_, fy_, cx_, cy_);
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!has_camera_info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "等待 camera_info,位姿暂不估计");
    }

    cv::Mat bgr, gray;
    try {
      bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge 转换失败: %s", e.what());
      return;
    }
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

    // CLAHE 对比度增强:提升低光照/低对比度场景下的检测率
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    cv::Mat enhanced;
    clahe->apply(gray, enhanced);

    image_u8_t im{enhanced.cols, enhanced.rows, enhanced.cols, enhanced.data};

    zarray_t * detections = apriltag_detector_detect(td_, &im);

    for (int i = 0; i < zarray_size(detections); ++i) {
      apriltag_detection_t * det = nullptr;
      zarray_get(detections, i, &det);

      if (target_tag_id_ >= 0 && static_cast<int>(det->id) != target_tag_id_) {
        continue;
      }

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "检测到 tag id=%d, hamming=%d", det->id, det->hamming);

      if (!has_camera_info_) {
        continue;
      }

      // ---- 用 solvePnP 估计位姿 ----
      // Apriltag 角点顺序: 从标签左上角开始顺时针(从相机视角看)
      std::vector<cv::Point2f> image_points = {
        {static_cast<float>(det->p[0][0]), static_cast<float>(det->p[0][1])},
        {static_cast<float>(det->p[1][0]), static_cast<float>(det->p[1][1])},
        {static_cast<float>(det->p[2][0]), static_cast<float>(det->p[2][1])},
        {static_cast<float>(det->p[3][0]), static_cast<float>(det->p[3][1])},
      };

      const float half = static_cast<float>(tag_size_ / 2.0);
      std::vector<cv::Point3f> object_points = {
        {-half, -half, 0.0f},
        { half, -half, 0.0f},
        { half,  half, 0.0f},
        {-half,  half, 0.0f},
      };

      cv::Mat rvec, tvec;
      cv::solvePnP(
        object_points, image_points,
        camera_matrix_, dist_coeffs_,
        rvec, tvec,
        false, cv::SOLVEPNP_ITERATIVE);

      cv::Mat R;
      cv::Rodrigues(rvec, R);

      const double tx = tvec.at<double>(0, 0);
      const double ty = tvec.at<double>(1, 0);
      const double tz = tvec.at<double>(2, 0);

      double qx, qy, qz, qw;
      rotation_to_quaternion(R, qx, qy, qz, qw);

      // 发布 PoseStamped
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = msg->header.stamp;
      pose_msg.header.frame_id = msg->header.frame_id;
      pose_msg.pose.position.x = tx;
      pose_msg.pose.position.y = ty;
      pose_msg.pose.position.z = tz;
      pose_msg.pose.orientation.x = qx;
      pose_msg.pose.orientation.y = qy;
      pose_msg.pose.orientation.z = qz;
      pose_msg.pose.orientation.w = qw;
      pose_pub_->publish(pose_msg);

      // 发布 TF
      if (publish_tf_ && tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = msg->header.frame_id;
        tf_msg.child_frame_id = child_frame_id_;
        tf_msg.transform.translation.x = tx;
        tf_msg.transform.translation.y = ty;
        tf_msg.transform.translation.z = tz;
        tf_msg.transform.rotation.x = qx;
        tf_msg.transform.rotation.y = qy;
        tf_msg.transform.rotation.z = qz;
        tf_msg.transform.rotation.w = qw;
        tf_broadcaster_->sendTransform(tf_msg);
      }

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "tag 位姿: x=%.3f y=%.3f z=%.3f", tx, ty, tz);
    }

    apriltag_detections_destroy(detections);
  }

  static void rotation_to_quaternion(const cv::Mat & R, double & qx, double & qy, double & qz, double & qw)
  {
    const double r11 = R.at<double>(0, 0);
    const double r12 = R.at<double>(0, 1);
    const double r13 = R.at<double>(0, 2);
    const double r21 = R.at<double>(1, 0);
    const double r22 = R.at<double>(1, 1);
    const double r23 = R.at<double>(1, 2);
    const double r31 = R.at<double>(2, 0);
    const double r32 = R.at<double>(2, 1);
    const double r33 = R.at<double>(2, 2);

    const double trace = r11 + r22 + r33;
    if (trace > 0.0) {
      const double s = 0.5 / std::sqrt(trace + 1.0);
      qw = 0.25 / s;
      qx = (r32 - r23) * s;
      qy = (r13 - r31) * s;
      qz = (r21 - r12) * s;
    } else if (r11 > r22 && r11 > r33) {
      const double s = 2.0 * std::sqrt(1.0 + r11 - r22 - r33);
      qw = (r32 - r23) / s;
      qx = 0.25 * s;
      qy = (r12 + r21) / s;
      qz = (r13 + r31) / s;
    } else if (r22 > r33) {
      const double s = 2.0 * std::sqrt(1.0 + r22 - r11 - r33);
      qw = (r13 - r31) / s;
      qx = (r12 + r21) / s;
      qy = 0.25 * s;
      qz = (r23 + r32) / s;
    } else {
      const double s = 2.0 * std::sqrt(1.0 + r33 - r11 - r22);
      qw = (r21 - r12) / s;
      qx = (r13 + r31) / s;
      qy = (r23 + r32) / s;
      qz = 0.25 * s;
    }
  }

  // 参数
  std::string image_topic_, camera_info_topic_, tag_family_, child_frame_id_;
  double tag_size_ = 0.120;
  int target_tag_id_ = 0;
  bool publish_tf_ = true;

  // 相机内参
  bool has_camera_info_ = false;
  double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // Apriltag
  apriltag_family_t * tf_ = nullptr;
  apriltag_detector_t * td_ = nullptr;

  // ROS 接口
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace car_camera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<car_camera::AprilTagDetectorNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("apriltag_detector"), "节点异常退出: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
