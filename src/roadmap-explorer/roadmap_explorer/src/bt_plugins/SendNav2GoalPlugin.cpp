/**
    Copyright 2025 Suchetan Saravanan.

    Licensed to the Apache Software Foundation (ASF) under one
    or more contributor license agreements.  See the NOTICE file
    distributed with this work for additional information
    regarding copyright ownership.  The ASF licenses this file
    to you under the Apache License, Version 2.0 (the
    "License"); you may not use this file except in compliance
    with the License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing,
    software distributed under the License is distributed on an
    "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
    KIND, either express or implied.  See the License for the
    specific language governing permissions and limitations
    under the License.
*/

#include "roadmap_explorer/bt_plugins/SendNav2GoalPlugin.hpp"
#include "roadmap_explorer/CostAssigner.hpp"
#include "roadmap_explorer/FrontierGoalSanitizer.hpp"
#include "roadmap_explorer/Parameters.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>

#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav2_util/node_utils.hpp>

namespace roadmap_explorer
{
    class SendNav2Goal : public BT::StatefulActionNode
    {
    public:
    SendNav2Goal(
        const std::string & name, const BT::NodeConfiguration & config,
        std::shared_ptr<Nav2Interface<nav2_msgs::action::NavigateToPose>> nav2_interface,
        std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros)
    : BT::StatefulActionNode(name, config)
    {
        nav2_interface_ = nav2_interface;
        ros_node_ptr_ = ros_node_ptr;
        explore_costmap_ros_ = explore_costmap_ros;
        nav2_util::declare_parameter_if_not_declared(
            ros_node_ptr_, "explorationBT.frontier_goal_boundary_margin",
            rclcpp::ParameterValue(0.20));
        nav2_util::declare_parameter_if_not_declared(
            ros_node_ptr_, "explorationBT.frontier_goal_max_projection_distance",
            rclcpp::ParameterValue(0.60));
        boundary_margin_ = std::max(
            0.0, ros_node_ptr_->get_parameter(
                "explorationBT.frontier_goal_boundary_margin").as_double());
        max_projection_distance_ = std::max(
            boundary_margin_, ros_node_ptr_->get_parameter(
                "explorationBT.frontier_goal_max_projection_distance").as_double());
        LOG_INFO("SendNav2Goal Constructor");
    }

    BT::NodeStatus onStart() override
    {
        LOG_FLOW("SendNav2Goal onStart");
        has_last_sent_goal_ = false;
        cancel_due_to_invalid_goal_ = false;
        FrontierPtr allocatedFrontier = std::make_shared<Frontier>();
        getInput("allocated_frontier", allocatedFrontier);
        geometry_msgs::msg::PoseStamped goalPose;
        if (!makeSafeGoal(allocatedFrontier, goalPose)) {
        markFrontierFailed(allocatedFrontier);
        return BT::NodeStatus::FAILURE;
        }
        if (!nav2_interface_->canSendNewGoal()) {
        LOG_WARN(
            "A Nav2 goal is still active when a new Roadmap frontier starts; "
            "cancelling the stale goal first. Status:" <<
            nav2_interface_->getGoalStatus());
        nav2_interface_->cancelAllGoals();
        cancel_due_to_invalid_goal_ = true;
        return BT::NodeStatus::RUNNING;
        }
        else
        {
        if (!nav2_interface_->sendGoal(goalPose)) {
            markFrontierFailed(allocatedFrontier);
            return BT::NodeStatus::FAILURE;
        }
        last_sent_goal_ = goalPose;
        has_last_sent_goal_ = true;
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        LOG_DEBUG("SendNav2Goal onRunning");
        FrontierPtr allocatedFrontier = std::make_shared<Frontier>();
        getInput("allocated_frontier", allocatedFrontier);
        LOG_DEBUG("Sending goal " << allocatedFrontier);
        if (nav2_interface_->getGoalStatus() == NavGoalStatus::SENDING_GOAL) {
        LOG_INFO("Nav2 goal is being sent, waiting for response...");
        return BT::NodeStatus::RUNNING;
        }
        if (nav2_interface_->getGoalStatus() == NavGoalStatus::ONGOING) {
        LOG_INFO("Nav2 goal is ongoing, waiting for completion...");
        geometry_msgs::msg::PoseStamped goalPose;
        if (!makeSafeGoal(allocatedFrontier, goalPose)) {
            LOG_WARN("The active frontier no longer has a safe goal; cancelling the old Nav2 path");
            markFrontierFailed(allocatedFrontier);
            nav2_interface_->cancelAllGoals();
            cancel_due_to_invalid_goal_ = true;
            return BT::NodeStatus::RUNNING;
        }
        LOG_TRACE(
            "Current goal pose: " << goalPose.pose.position.x << ", " << goalPose.pose.position.y << ", " << goalPose.pose.orientation.z << ", " << goalPose.pose.orientation.w << ", " << goalPose.pose.orientation.x << ", " <<
            goalPose.pose.orientation.y);
        const double update_distance = has_last_sent_goal_ ?
            std::hypot(
                goalPose.pose.position.x - last_sent_goal_.pose.position.x,
                goalPose.pose.position.y - last_sent_goal_.pose.position.y) :
            std::numeric_limits<double>::infinity();
        const double update_threshold = explore_costmap_ros_ ?
            explore_costmap_ros_->getCostmap()->getResolution() * 0.5 : 0.025;
        if (update_distance > update_threshold) {
            nav2_interface_->sendUpdatedGoal(goalPose);
            last_sent_goal_ = goalPose;
            has_last_sent_goal_ = true;
        }
        return BT::NodeStatus::RUNNING;
        }
        if (
        nav2_interface_->getGoalStatus() == NavGoalStatus::CANCELLING ||
        nav2_interface_->getGoalStatus() == NavGoalStatus::SENDING_GOAL)
        {
        return BT::NodeStatus::RUNNING;
        }
        if (
        cancel_due_to_invalid_goal_ ||
        nav2_interface_->getGoalStatus() == NavGoalStatus::CANCELLED ||
        nav2_interface_->getGoalStatus() == NavGoalStatus::REJECTED)
        {
        markFrontierFailed(allocatedFrontier);
        return BT::NodeStatus::FAILURE;
        }
        if (nav2_interface_->getGoalStatus() == NavGoalStatus::FAILED) {
        LOG_ERROR("Nav2 goal has aborted!");
        config().blackboard->set<ExplorationErrorCode>(
            "error_code_id", ExplorationErrorCode::NAV2_GOAL_ABORT);
        config().blackboard->set<FrontierPtr>(
            "latest_failed_frontier", allocatedFrontier);
        return BT::NodeStatus::FAILURE;
        }
        if (nav2_interface_->getGoalStatus() == NavGoalStatus::SUCCEEDED) {
        LOG_WARN("Nav2 goal has succeeded!");
        }
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override
    {
        LOG_INFO("SendNav2Goal onHalted");
        if (nav2_interface_->isGoalActive()) {
        LOG_WARN("Roadmap planning halted; cancelling the active Nav2 goal so no stale path remains");
        nav2_interface_->cancelAllGoals();
        }
        return;
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<FrontierPtr>("allocated_frontier")};
    }

    std::shared_ptr<Nav2Interface<nav2_msgs::action::NavigateToPose>> nav2_interface_;
    std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
    geometry_msgs::msg::PoseStamped last_sent_goal_;
    bool has_last_sent_goal_{false};
    bool cancel_due_to_invalid_goal_{false};
    double boundary_margin_{0.20};
    double max_projection_distance_{0.60};

    void markFrontierFailed(const FrontierPtr & frontier)
    {
        config().blackboard->set<ExplorationErrorCode>(
            "error_code_id", ExplorationErrorCode::NAV2_GOAL_ABORT);
        config().blackboard->set<FrontierPtr>(
            "latest_failed_frontier", frontier);
    }

    bool makeSafeGoal(
        const FrontierPtr & frontier,
        geometry_msgs::msg::PoseStamped & goal_pose)
    {
        if (!explore_costmap_ros_) {
        LOG_ERROR("Cannot validate frontier goal without the exploration costmap");
        return false;
        }
        geometry_msgs::msg::PoseStamped robot_pose;
        if (!explore_costmap_ros_->getRobotPose(robot_pose)) {
        LOG_ERROR("Cannot validate frontier goal because the robot pose is unavailable");
        return false;
        }

        auto * costmap = explore_costmap_ros_->getCostmap();
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
            *(costmap->getMutex()));
        const auto projection = projectFrontierGoalToKnownFree(
            *costmap, frontier->getGoalPoint(), robot_pose.pose.position,
            boundary_margin_, max_projection_distance_);
        if (!projection.valid) {
        LOG_WARN(
            "Rejecting frontier before Nav2 dispatch: reference=(" <<
            frontier->getGoalPoint().x << ", " << frontier->getGoalPoint().y <<
            "), no reachable known-free cell within " <<
            max_projection_distance_ << "m and " << boundary_margin_ <<
            "m map-edge margin");
        return false;
        }

        goal_pose.header.frame_id = "map";
        goal_pose.pose.position = projection.point;
        goal_pose.pose.orientation = frontier->getGoalOrientation();
        if (projection.adjusted) {
        LOG_INFO(
            "Projected frontier goal from (" << frontier->getGoalPoint().x <<
            ", " << frontier->getGoalPoint().y << ") to reachable known-free (" <<
            projection.point.x << ", " << projection.point.y <<
            "), projection=" << projection.projection_distance << "m");
        }
        return true;
    }
    };

    SendNav2GoalPlugin::SendNav2GoalPlugin()
    {
    }

    SendNav2GoalPlugin::~SendNav2GoalPlugin()
    {
    }

    void SendNav2GoalPlugin::registerNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<BTContext> context)
    {
        BT::NodeBuilder builder =
            [context](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<SendNav2Goal>(
                name,
                config,
                context->nav2_interface,
                context->node,
                context->explore_costmap_ros);
        };
        factory.registerBuilder<SendNav2Goal>("SendNav2Goal", builder);
    }
}

PLUGINLIB_EXPORT_CLASS(
    roadmap_explorer::SendNav2GoalPlugin,
    roadmap_explorer::BTPlugin)
