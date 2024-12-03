/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*      Title     : collision_check.cpp
 *      Project   : moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 */

#include <iostream>
#include <chrono>

#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <moveit_servo/collision_check.h>
// #include <moveit_servo/make_shared_from_pool.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.collision_check");
static const double MIN_RECOMMENDED_COLLISION_RATE = 10;
constexpr size_t ROS_LOG_THROTTLE_PERIOD = 30 * 1000;  // Milliseconds to throttle logs inside loops

namespace moveit_servo
{
// Constructor for the class that handles collision checking
CollisionCheck::CollisionCheck(const rclcpp::Node::SharedPtr& node, const ServoParameters::SharedConstPtr& parameters,
                               const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : node_(node)
  , parameters_(parameters)
  , planning_scene_monitor_(planning_scene_monitor)
  , self_velocity_scale_coefficient_(-log(0.001) / parameters->self_collision_proximity_threshold)
  , scene_velocity_scale_coefficient_(-log(0.001) / parameters->scene_collision_proximity_threshold)
  , period_(1. / parameters->collision_check_rate)
{
  // Init collision request
  collision_request_.group_name = parameters_->move_group_name;
  collision_request_.distance = true;  // enable distance-based collision checking
  collision_request_.contacts = true;  // Record the names of collision pairs

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "base_cylinder";
  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = 0.4;
  collision_object.primitives[0].dimensions[1] = 0.6;
  collision_object.primitives[0].dimensions[2] = 0.2;
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x = 0.0;
  collision_object.primitive_poses[0].position.y = 0.0;
  collision_object.primitive_poses[0].position.z = -0.1;
  collision_object.primitive_poses[0].orientation.x = 0.0;
  collision_object.primitive_poses[0].orientation.y = 0.0;
  collision_object.primitive_poses[0].orientation.z = 0.0;
  collision_object.primitive_poses[0].orientation.w = 1.0;
  // planning_scene_interface_.applyCollisionObject(collision_object);

  moveit_msgs::msg::AttachedCollisionObject attached_collision_object;
  attached_collision_object.link_name = "base_link";
  attached_collision_object.object = collision_object;
  attached_collision_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  planning_scene_interface_.applyAttachedCollisionObject(attached_collision_object);

  auto locked_scene = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_);

  distance_request_.group_name = parameters_->move_group_name;
  distance_request_.max_contacts_per_body = 25;
  distance_request_.enable_nearest_points = false;
  distance_request_.enable_signed_distance = true;
  distance_request_.compute_gradient = true;
  distance_request_.acm = &locked_scene->getAllowedCollisionMatrix();
  distance_request_.type = collision_detection::DistanceRequestType::ALL;

  marker.header.frame_id = "world";
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  if (parameters_->collision_check_rate < MIN_RECOMMENDED_COLLISION_RATE)
  {
    auto& clk = *node_->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clk, ROS_LOG_THROTTLE_PERIOD,
                                "Collision check rate is low, increase it in yaml file if CPU allows");
  }

  // ROS pubs/subs
  collision_velocity_scale_pub_ =
      node_->create_publisher<std_msgs::msg::Float64>("~/collision_velocity_scale", rclcpp::SystemDefaultsQoS());

  nearest_points_0_pub_ = 
      node_->create_publisher<geometry_msgs::msg::PointStamped>("nearest_point_0", rclcpp::SystemDefaultsQoS());// Made by pipe, testing purposes
  nearest_points_1_pub_ = 
      node_->create_publisher<geometry_msgs::msg::PointStamped>("nearest_point_1", rclcpp::SystemDefaultsQoS());// Made by pipe, testing purposes
  marker_array_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>("marker_array", rclcpp::SystemDefaultsQoS());// Made by pipe, testing purposes
  robot_distance_array_pub_ =
      node_->create_publisher<haptic_station_msgs::msg::RobotDistanceArray>("robot_distance_array", rclcpp::SystemDefaultsQoS());// Made by pipe, testing purposes
  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
}

void CollisionCheck::start()
{
  

  timer_ = node_->create_wall_timer(std::chrono::duration<double>(period_), [this]() { return run(); });
}

void CollisionCheck::run()
{
  if (paused_)
  {
    return;
  }

  // Update to the latest current state
  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  current_state_->updateCollisionBodyTransforms();
  collision_detected_ = false;

  // Do a timer-safe distance-based collision detection
  collision_result_.clear();
  distance_result_.clear();
  auto locked_scene = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_);
  locked_scene->getCollisionEnv()->checkRobotCollision(collision_request_, collision_result_, *current_state_,
                                                       locked_scene->getAllowedCollisionMatrix());
  // Calculate distances                                                 
  auto start = std::chrono::high_resolution_clock::now();
  locked_scene->getCollisionEnv()->distanceRobot(distance_request_, distance_result_, *current_state_);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = end - start;
  RCLCPP_INFO(LOGGER, "Distances calculated in '%f' seconds", duration.count());

  scene_collision_distance_ = collision_result_.distance;
  collision_detected_ |= collision_result_.collision;
  collision_result_.print();

  haptic_station_msgs::msg::RobotDistanceArray robot_distance_array;

  int index = 0;
  for (auto it = distance_result_.distances.begin(); it != distance_result_.distances.end(); ++it, ++index) {
        std::pair<std::__cxx11::basic_string<char>,std::__cxx11::basic_string<char>> pair_string = it->first;
        std::vector<collision_detection::DistanceResultsData>& mvect = it->second;

        std::__cxx11::basic_string<char> link_first = pair_string.first;
        std::__cxx11::basic_string<char> link_second = pair_string.second;

        // RCLCPP_INFO(LOGGER, "Pair: '%s' , '%s'", link_first.c_str(), link_second.c_str());
        // RCLCPP_INFO(LOGGER, "Vector Size: '%li'", mvect.size());

        for(long unsigned int i = 0; i < mvect.size(); i++)
        {
          collision_detection::DistanceResultsData& dist_res = mvect[i];
          nearest_points_0_ = dist_res.nearest_points[0];
          nearest_points_1_ = dist_res.nearest_points[1];
          normal_ = dist_res.normal;
          
          marker.ns = link_second.c_str();

          marker.id = i;
          marker.type = visualization_msgs::msg::Marker::ARROW;

          marker.pose.position.x = 0;
          marker.pose.position.y = 0;
          marker.pose.position.z = 0;
          marker.scale.x = 0.02;
          marker.scale.y = 0.05;
          marker.scale.z = 0.06;

          geometry_msgs::msg::Point start_point;
          start_point.x = nearest_points_0_.x();
          start_point.y = nearest_points_0_.y();
          start_point.z = nearest_points_0_.z();
          marker.points.push_back(start_point);

          geometry_msgs::msg::Point end_point;
          end_point.x = nearest_points_1_.x();
          end_point.y = nearest_points_1_.y();
          end_point.z = nearest_points_1_.z();
          marker.points.push_back(end_point);

          marker.color.a = 1.0;
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;

          marker_array.markers.push_back(marker);
          marker.points.clear();

          haptic_station_msgs::msg::RobotDistance robot_distance;
          robot_distance.link_first = link_first.c_str();
          robot_distance.link_second = link_second.c_str();
          robot_distance.distance = dist_res.distance;
          geometry_msgs::msg::PointStamped point_first;
          point_first.header.frame_id = "world";
          point_first.point = start_point;
          robot_distance.point_first = point_first;
          geometry_msgs::msg::PointStamped point_second;
          point_second.header.frame_id = "world";
          point_second.point = end_point;
          robot_distance.point_second = point_second;
          robot_distance_array.distances.push_back(robot_distance);
        }
    }
  marker_array_pub_->publish(marker_array);
  marker_array.markers.clear();

  robot_distance_array_pub_->publish(robot_distance_array);

  //// Made by pipe, testing purposes ////
  // auto np0 = std::make_unique<geometry_msgs::msg::PointStamped>();
  // np0->header.frame_id = "world";
  // np0->point.x = nearest_points_0_.x();
  // np0->point.y = nearest_points_0_.y();
  // np0->point.z = nearest_points_0_.z();
  // nearest_points_0_pub_->publish(std::move(np0));

  // auto np1 = std::make_unique<geometry_msgs::msg::PointStamped>();
  // np1->header.frame_id = "world";
  // np1->point.x = nearest_points_1_.x();
  // np1->point.y = nearest_points_1_.y();
  // np1->point.z = nearest_points_1_.z();
  // nearest_points_1_pub_->publish(std::move(np1));
  ////////////////////////////////////////

  collision_result_.clear();
  // Self-collisions and scene collisions are checked separately so different thresholds can be used
  locked_scene->getCollisionEnvUnpadded()->checkSelfCollision(collision_request_, collision_result_, *current_state_,
                                                              locked_scene->getAllowedCollisionMatrix());
  self_collision_distance_ = collision_result_.distance;
  collision_detected_ |= collision_result_.collision;
  collision_result_.print();

  velocity_scale_ = 1;
  // If we're definitely in collision, stop immediately
  if (collision_detected_)
  {
    velocity_scale_ = 0;
  }
  else
  {
    // If we are far from a collision, velocity_scale should be 1.
    // If we are very close to a collision, velocity_scale should be ~zero.
    // When scene_collision_proximity_threshold is breached, start decelerating exponentially.
    if (scene_collision_distance_ < parameters_->scene_collision_proximity_threshold)
    {
      // velocity_scale = e ^ k * (collision_distance - threshold)
      // k = - ln(0.001) / collision_proximity_threshold
      // velocity_scale should equal one when collision_distance is at collision_proximity_threshold.
      // velocity_scale should equal 0.001 when collision_distance is at zero.
      velocity_scale_ = std::min(velocity_scale_,
                                 exp(scene_velocity_scale_coefficient_ *
                                     (scene_collision_distance_ - parameters_->scene_collision_proximity_threshold)));
    }

    if (self_collision_distance_ < parameters_->self_collision_proximity_threshold)
    {
      velocity_scale_ =
          std::min(velocity_scale_, exp(self_velocity_scale_coefficient_ *
                                        (self_collision_distance_ - parameters_->self_collision_proximity_threshold)));
    }
  }

  // publish message
  {
    auto msg = std::make_unique<std_msgs::msg::Float64>();
    msg->data = velocity_scale_;
    collision_velocity_scale_pub_->publish(std::move(msg));
  }
}

void CollisionCheck::setPaused(bool paused)
{
  paused_ = paused;
}

}  // namespace moveit_servo
