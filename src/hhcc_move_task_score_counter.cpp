/*
Copyright (c) 2019 TOYOTA MOTOR CORPORATION
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#ifdef ROS1
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
rclcpp::Node::SharedPtr node = nullptr;
#define ROS_INFO(...) RCLCPP_INFO(node->get_logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(node->get_logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(node->get_logger(), __VA_ARGS__)
#endif

#include <iostream>
#include <string>

double per_call_and_come;
double collision_per_second;
double per_area_size;

double collision_score;
double call_and_come_score;
double area_size_score;

int exit_on_end;

std::string current_state;

#ifdef ROS1
ros::Time prev_detect_cb;
#else
rclcpp::Time prev_detect_cb;
#endif

#ifdef ROS1
void cb_detect(const std_msgs::BoolConstPtr& detect)
#else
void cb_detect(const std_msgs::msg::Bool::SharedPtr detect)
#endif
{
#ifdef ROS1
    ros::Time now = ros::Time::now();
    double dt = fabs(now.toSec() - prev_detect_cb.toSec());
#else
    rclcpp::Time now = node->get_clock()->now();
    double dt = fabs(now.seconds() - prev_detect_cb.seconds());
#endif
    if (detect->data && dt < 10) {
        double score = collision_per_second * dt;
        collision_score += score;
        ROS_WARN("[HHCC] Detect collision!");
    }
    prev_detect_cb = now;
}

#ifdef ROS1
void cb_state(const std_msgs::StringConstPtr& state)
#else
void cb_state(const std_msgs::msg::String::SharedPtr state)
#endif
{
    current_state = state->data;
}

#ifdef ROS1
void cb_count(const std_msgs::Float32Ptr& count)
#else
void cb_count(const std_msgs::msg::Float32::SharedPtr count)
#endif
{
    static double prev_call_and_come_score = 0.0;
    call_and_come_score = per_call_and_come * count->data;
    if (call_and_come_score != prev_call_and_come_score) {
        ROS_WARN("[HHCC] New success of call_and_come!");
        if (area_size_score < 100.0) {
            ROS_WARN("[HHCC] However, system has detected very low reached area size score (you probably haven't moved at all...). You cannot earn score if you just stay at the entrance.");
            call_and_come_score = 0.0;
        }
        prev_call_and_come_score = call_and_come_score;
    }
}

#ifdef ROS1
void cb_area(const std_msgs::Float32Ptr& area_size)
#else
void cb_area(const std_msgs::msg::Float32::SharedPtr area_size)
#endif
{
    if (current_state == "empty") {
        static double prev_area_size_score = 0.0;
        area_size_score = area_size->data * per_area_size;
        if (area_size_score != prev_area_size_score) {
            ROS_WARN("[HHCC] Detect increase of reached area size!");
            prev_area_size_score = area_size_score;
        }
    }
}

int main(int argc, char **argv)
{
    // initialize ROS node
#ifdef ROS1
    ros::init(argc, argv, "hhcc_move_task_score_counter");
    ros::NodeHandle n("~");
#else
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("hhcc_move_task_score_counter");
#endif

#ifdef ROS1
    if (n.getParam("per_call_and_come", per_call_and_come)) {
        ROS_INFO("per_call_and_come is defined as: %f", per_call_and_come);
    } else {
        per_call_and_come = 3000.0;
        ROS_ERROR("Failed to get param 'per_call_and_come' use default '%f'", per_call_and_come);
    }
    
    if (n.getParam("collision_per_second", collision_per_second)) {
        ROS_INFO("collision_per_second is defined as: %f", collision_per_second);
    } else {
        collision_per_second = -1.0;
        ROS_ERROR("Failed to get param 'collision_per_second' use default '%f'", collision_per_second);
    }
    
    if (n.getParam("per_area_size", per_area_size)) {
        ROS_INFO("per_area_size is defined as: %f", per_area_size);
    } else {
        per_area_size = 10.0;
        ROS_ERROR("Failed to get param 'per_area_size' use default '%f'", per_area_size);
    }
    
    if (n.getParam("exit_on_end", exit_on_end)) {
        ROS_INFO("exit_on_end is defined as: %i", exit_on_end);
    } else {
        exit_on_end = 1;
        ROS_ERROR("Failed to get param 'exit_on_end' use default '%i'", exit_on_end);
    }
#else
    auto per_call_and_come = node->declare_parameter<double>("per_call_and_come", 3000.0);
    auto collision_per_second = node->declare_parameter<double>("collision_per_second", -1.0);
    auto per_area_size = node->declare_parameter<double>("per_area_size", 10.0);
    auto exit_on_end = node->declare_parameter<int>("exit_on_end", 1);
#endif

    call_and_come_score = 0.0;
    collision_score = 0.0;
    area_size_score = 0.0;

#ifdef ROS1
    ros::Time::init();
    prev_detect_cb = ros::Time::now();

    ros::Publisher pub = n.advertise<std_msgs::Float32>("/score", 1000);
    ros::Rate rate(10);
    ros::Subscriber sub = n.subscribe("/undesired_contact_detector/detect", 1, cb_detect);
    ros::Subscriber sub2 = n.subscribe("/call_and_come_evaluator/count", 1, cb_count);
    ros::Subscriber sub3 = n.subscribe("/call_and_come_evaluator/state", 1, cb_state);
    ros::Subscriber sub4 = n.subscribe("/reached_region_calculator/area_size", 1, cb_area);
#else
    auto pub = node->create_publisher<std_msgs::msg::Float32>("/score", 1000);
    auto rate = rclcpp::Rate(10);
    auto sub = node->create_subscription<std_msgs::msg::Bool>("/undesired_contact_detector/detect", 1, cb_detect);
    auto sub2 = node->create_subscription<std_msgs::msg::Float32>("/call_and_come_evaluator/count", 1, cb_count);
    auto sub3 = node->create_subscription<std_msgs::msg::String>("/call_and_come_evaluator/state", 1, cb_state);
    auto sub4 = node->create_subscription<std_msgs::msg::Float32>("/reached_region_calculator/area_size", 1, cb_area);
#endif

    double prev_score = 0.0;
    double score_offset = 0.0;
    int end_count = 0;
#ifdef ROS1
    while (ros::ok()) {
        std_msgs::Float32 msg;
#else
    while (rclcpp::ok()) {
        std_msgs::msg::Float32 msg;
#endif
        double score = collision_score + call_and_come_score + area_size_score + score_offset;
        if (score < 0.0) {
            score_offset += fabs(score);
            score = 0.0;
        }
        if (fabs(score - prev_score) > 0.1) {
            ROS_WARN("[HHCC] Your score has been changed to %i (%+i).", (int)score, (int)(score - prev_score));
            prev_score = score;
        }
        msg.data = score;
#ifdef ROS1
        pub.publish(msg);
        ros::spinOnce();
#else
        pub->publish(msg);
        rclcpp::spin_some(node);
#endif
        rate.sleep();
        if (current_state == "success" || current_state == "fail") {
            end_count++;
            if (end_count > 30 && exit_on_end != 0) {
                break;
            }
        }
    }
}
