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


#include "roadmap_explorer/bt_plugins/BlacklistGoalPlugin.hpp"
#include "roadmap_explorer/CostAssigner.hpp"
#include "roadmap_explorer/Parameters.hpp"
#include "roadmap_explorer/TransientFrontierBlacklist.hpp"

#include <mutex>
#include <utility>
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

namespace roadmap_explorer
{
    class BlacklistGoal : public BT::SyncActionNode
    {
    public:
    BlacklistGoal(
        const std::string & name, const BT::NodeConfiguration & config,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros)
    : BT::SyncActionNode(name, config),
      explore_costmap_ros_(std::move(explore_costmap_ros))
    {
        LOG_INFO("BlacklistGoal Constructor");
    }

    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;

    void blacklistFrontier(const FrontierPtr & frontier, BT::Blackboard::Ptr blackboard)
    {
        auto blacklisted_frontiers = blackboard->get<std::shared_ptr<std::vector<FrontierPtr>>>(
            "blacklisted_frontiers");
        blacklisted_frontiers->push_back(frontier);
    }

    BT::NodeStatus tick() override
    {
        EventLoggerInstance.startEvent("BlacklistGoal");
        LOG_FLOW("MODULE BlacklistGoal");
        FrontierPtr allocatedFrontier = std::make_shared<Frontier>();
        if(!config().blackboard->get<FrontierPtr>("latest_failed_frontier", allocatedFrontier))
        {
        throw RoadmapExplorerException("Could not get latest allocated frontier from blackboard");
        }
        bool transient = false;
        config().blackboard->get<bool>(
            "latest_failed_frontier_is_transient", transient);
        if (transient && explore_costmap_ros_) {
        auto transient_blacklist =
            config().blackboard->get<std::shared_ptr<TransientFrontierBlacklist>>(
            "transient_blacklisted_frontiers");
        auto * costmap = explore_costmap_ros_->getCostmap();
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
            *(costmap->getMutex()));
        transient_blacklist->add(allocatedFrontier, costmapRevision(*costmap));
        LOG_WARN(
            "Setting transient frontier blacklist " << allocatedFrontier <<
            "; it will be released after the exploration costmap changes");
        } else {
        LOG_WARN("Setting permanent frontier blacklist " << allocatedFrontier);
        blacklistFrontier(allocatedFrontier, config().blackboard);
        }
        return BT::NodeStatus::SUCCESS;
    }
    };

    BlacklistGoalPlugin::BlacklistGoalPlugin()
    {
    }

    BlacklistGoalPlugin::~BlacklistGoalPlugin()
    {
    }

    void BlacklistGoalPlugin::registerNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<BTContext> context)
    {
        BT::NodeBuilder builder =
            [context](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<BlacklistGoal>(
                name, config, context->explore_costmap_ros);
        };
        factory.registerBuilder<BlacklistGoal>("BlacklistGoal", builder);
    }
}

PLUGINLIB_EXPORT_CLASS(
    roadmap_explorer::BlacklistGoalPlugin,
    roadmap_explorer::BTPlugin)
