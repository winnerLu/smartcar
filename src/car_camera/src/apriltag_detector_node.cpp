// car_camera Apriltag 检测节点 — 多标签停车板模式。
//
// 订阅: /camera/image_raw    (sensor_msgs/Image, bgr8)
//       /camera/camera_info  (sensor_msgs/CameraInfo)
// 发布: /tag_pose            (geometry_msgs/PoseStamped) ← 停车板中心位姿
// TF:  camera_link -> parking_board (由本节点发布)
//
// 关键设计:
//   - 基于 AprilRobotics/apriltag C 库,检测 tag36h11 家族。
//   - 多标签模式(tag_ids="0,1,2,3,4,5,6,7"):取所有可见标签的位姿均值作为停车板中心。
//   - 单标签模式(tag_ids="0"):与之前行为一致,直接输出标签位姿。
//   - 位姿估计用 OpenCV solvePnP,每个标签独立解算后融合。
//   - CLAHE 增强 + quad_decimate=1.0 保证小标签/低光照检测率。

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <set>
#include <sstream>

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
    tag_size_ = declare_parameter<double>("tag_size", 0.050);  // 5cm 标签
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "parking_board");

    // 多标签 ID 列表(逗号分隔,如 "0,1,2,3,4,5,6,7")
    std::string tag_ids_str = declare_parameter<std::string>("tag_ids", "0,1,2,3,4,5,6,7");
    parse_tag_ids(tag_ids_str);

    // ---- Apriltag 检测器初始化 ----
    if (tag_family_ != "tag36h11") {
      RCLCPP_ERROR(get_logger(), "本节点当前仅支持 tag36h11");
      throw std::runtime_error("不支持的 tag family");
    }
    tf_ = tag36h11_create();
    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, tf_);

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

    RCLCPP_INFO(get_logger(), "Apriltag 检测节点已启动(tag_size=%.3fm, %zu 个 ID),等待 %s ...",
      tag_size_, valid_ids_.size(), camera_info_topic_.c_str());
  }

  ~AprilTagDetectorNode()
  {
    apriltag_detector_destroy(td_);
    tag36h11_destroy(tf_);
  }

private:
  // -----------------------------------------------------------
  void parse_tag_ids(const std::string & str)
  {
    valid_ids_.clear();
    if (str.empty() || str == "-1" || str == "all") {
      // -1/all 表示检测所有 ID
      target_all_ = true;
      return;
    }
    target_all_ = false;
    std::istringstream ss(str);
    std::string token;
    while (std::getline(ss, token, ',')) {
      try {
        valid_ids_.insert(static_cast<uint32_t>(std::stoi(token)));
      } catch (...) { /* skip invalid */ }
    }
  }

  void on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (has_camera_info_) return;
    fx_ = msg->k[0]; fy_ = msg->k[4]; cx_ = msg->k[2]; cy_ = msg->k[5];
    camera_matrix_ = (cv::Mat_<double>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
    dist_coeffs_ = cv::Mat::zeros(1, static_cast<int>(msg->d.size()), CV_64F);
    for (size_t i = 0; i < msg->d.size(); ++i)
      dist_coeffs_.at<double>(0, static_cast<int>(i)) = msg->d[i];
    has_camera_info_ = true;
    RCLCPP_INFO(get_logger(), "内参: fx=%.3f fy=%.3f cx=%.3f cy=%.3f", fx_, fy_, cx_, cy_);
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!has_camera_info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "等待 camera_info...");
    }

    cv::Mat bgr, gray;
    try {
      bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge 转换失败: %s", e.what());
      return;
    }
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

    // CLAHE 增强
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    cv::Mat enhanced;
    clahe->apply(gray, enhanced);

    image_u8_t im{enhanced.cols, enhanced.rows, enhanced.cols, enhanced.data};
    zarray_t * detections = apriltag_detector_detect(td_, &im);

    // 收集所有有效标签的相机坐标系位置
    struct TagPose { double x, y, z, r11,r12,r13,r21,r22,r23,r31,r32,r33; };
    std::vector<TagPose> poses;
    bool printed_detect = false;

    for (int i = 0; i < zarray_size(detections); ++i) {
      apriltag_detection_t * det = nullptr;
      zarray_get(detections, i, &det);

      // 过滤 ID
      if (!target_all_ && valid_ids_.find(det->id) == valid_ids_.end())
        continue;

      if (!printed_detect) {
        printed_detect = true;
      }

      if (!has_camera_info_) continue;

      // solvePnP
      std::vector<cv::Point2f> img_pts = {
        {static_cast<float>(det->p[0][0]), static_cast<float>(det->p[0][1])},
        {static_cast<float>(det->p[1][0]), static_cast<float>(det->p[1][1])},
        {static_cast<float>(det->p[2][0]), static_cast<float>(det->p[2][1])},
        {static_cast<float>(det->p[3][0]), static_cast<float>(det->p[3][1])},
      };
      float half = static_cast<float>(tag_size_ / 2.0);
      std::vector<cv::Point3f> obj_pts = {
        {-half, -half, 0.0f}, {half, -half, 0.0f},
        {half,  half, 0.0f}, {-half,  half, 0.0f},
      };
      cv::Mat rvec, tvec;
      cv::solvePnP(obj_pts, img_pts, camera_matrix_, dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
      cv::Mat R;
      cv::Rodrigues(rvec, R);

      poses.push_back({
        tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0),
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2),
      });
    }

    // 发布停车板中心位姿(多标签平均)
    if (!poses.empty()) {
      double ax = 0, ay = 0, az = 0;
      // 旋转矩阵逐元素平均(SVD 正交化更适合,但这里简单平均足够)
      double ar11=0,ar12=0,ar13=0,ar21=0,ar22=0,ar23=0,ar31=0,ar32=0,ar33=0;
      for (auto & p : poses) {
        ax += p.x; ay += p.y; az += p.z;
        ar11 += p.r11; ar12 += p.r12; ar13 += p.r13;
        ar21 += p.r21; ar22 += p.r22; ar23 += p.r23;
        ar31 += p.r31; ar32 += p.r32; ar33 += p.r33;
      }
      double n = static_cast<double>(poses.size());
      ax /= n; ay /= n; az /= n;
      ar11/=n; ar12/=n; ar13/=n; ar21/=n; ar22/=n; ar23/=n; ar31/=n; ar32/=n; ar33/=n;

      double qx, qy, qz, qw;
      rotation_to_quaternion(ar11,ar12,ar13,ar21,ar22,ar23,ar31,ar32,ar33, qx,qy,qz,qw);

      // PoseStamped
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = msg->header.stamp;
      pose_msg.header.frame_id = msg->header.frame_id;
      pose_msg.pose.position.x = ax;
      pose_msg.pose.position.y = ay;
      pose_msg.pose.position.z = az;
      pose_msg.pose.orientation.x = qx;
      pose_msg.pose.orientation.y = qy;
      pose_msg.pose.orientation.z = qz;
      pose_msg.pose.orientation.w = qw;
      pose_pub_->publish(pose_msg);

      // TF
      if (publish_tf_ && tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = msg->header.frame_id;
        tf_msg.child_frame_id = child_frame_id_;
        tf_msg.transform.translation.x = ax;
        tf_msg.transform.translation.y = ay;
        tf_msg.transform.translation.z = az;
        tf_msg.transform.rotation.x = qx;
        tf_msg.transform.rotation.y = qy;
        tf_msg.transform.rotation.z = qz;
        tf_msg.transform.rotation.w = qw;
        tf_broadcaster_->sendTransform(tf_msg);
      }

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "检测到 %zu 个标签 -> 板中心: (%.3f, %.3f, %.3f)", poses.size(), ax, ay, az);
    }

    apriltag_detections_destroy(detections);
  }

  static void rotation_to_quaternion(
    double r11,double r12,double r13, double r21,double r22,double r23,
    double r31,double r32,double r33, double &qx,double &qy,double &qz,double &qw)
  {
    double trace = r11 + r22 + r33;
    if (trace > 0.0) {
      double s = 0.5 / std::sqrt(trace + 1.0);
      qw = 0.25 / s; qx = (r32 - r23) * s; qy = (r13 - r31) * s; qz = (r21 - r12) * s;
    } else if (r11 > r22 && r11 > r33) {
      double s = 2.0 * std::sqrt(1.0 + r11 - r22 - r33);
      qw = (r32 - r23) / s; qx = 0.25 * s; qy = (r12 + r21) / s; qz = (r13 + r31) / s;
    } else if (r22 > r33) {
      double s = 2.0 * std::sqrt(1.0 + r22 - r11 - r33);
      qw = (r13 - r31) / s; qx = (r12 + r21) / s; qy = 0.25 * s; qz = (r23 + r32) / s;
    } else {
      double s = 2.0 * std::sqrt(1.0 + r33 - r11 - r22);
      qw = (r21 - r12) / s; qx = (r13 + r31) / s; qy = (r23 + r32) / s; qz = 0.25 * s;
    }
  }

  // 参数
  std::string image_topic_, camera_info_topic_, tag_family_, child_frame_id_;
  double tag_size_ = 0.050;
  bool publish_tf_ = true;
  bool target_all_ = false;
  std::set<uint32_t> valid_ids_;

  // 内参
  bool has_camera_info_ = false;
  double fx_=0,fy_=0,cx_=0,cy_=0;
  cv::Mat camera_matrix_, dist_coeffs_;

  // Apriltag
  apriltag_family_t * tf_ = nullptr;
  apriltag_detector_t * td_ = nullptr;

  // ROS
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
