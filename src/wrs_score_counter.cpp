/*
Copyright (c) 2020 TOYOTA MOTOR CORPORATION
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
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <string>
#include <map>
#include <boost/bind.hpp>

double task1_per_delivery;
double task1_per_correct_category;
double task1_per_correct_orientation;

double task1_delivery_score;
double task1_category_score;
double task1_orientation_score;
double task2a_score;

ros::Time prev_detect_cb;

void cb_detect(const std_msgs::BoolConstPtr& detect)
{
    static bool first_time = true;
    if (detect->data) {
        if (first_time) {
            task2a_score = 0.0;
            first_time = false;
            ROS_WARN("[WRS] You hit the object on the floor!");
        }
    }
}

void cb_hsrb_in_room2(const std_msgs::Int16::ConstPtr& count)
{
    static bool first_time = true;
    if (count->data > 0) {
        if (first_time) {
            task2a_score = 100.0;
            first_time = false;
            ROS_WARN("[WRS] Entered room 2!");
        }
    }
}

void cb_count_delivery(const std::string place, const std_msgs::Int16::ConstPtr& count)
{
    static double prev_delivery_score = 0.0;
    static std::map<std::string, int> places;

    places[place] = count->data;
    int total_count = 0;
    for(auto itr = places.begin(); itr != places.end(); ++itr) {
        total_count += itr->second;
    }
    task1_delivery_score = task1_per_delivery * (double)total_count;
    if (task1_delivery_score != prev_delivery_score) {
        ROS_WARN("[WRS] Found new delivered object!");
        prev_delivery_score = task1_delivery_score;
    }
}

void cb_count_correct_category(const std::string place, const std_msgs::Int16::ConstPtr& count)
{
    static double prev_category_score = 0.0;
    static std::map<std::string, int> places;

    places[place] = count->data;
    int total_count = 0;
    for(auto itr = places.begin(); itr != places.end(); ++itr) {
        total_count += itr->second;
    }
    task1_category_score = task1_per_correct_category * (double)total_count;
    if (task1_category_score != prev_category_score) {
        ROS_WARN("[WRS] Found new delivered object with correct category!");
        prev_category_score = task1_category_score;
    }
}

int main(int argc, char **argv)
{
    // initialize ROS node
    ros::init(argc, argv, "wrc_score_counter");
    ros::NodeHandle n("~");

    if (n.getParam("task1_per_delivery", task1_per_delivery)) {
        ROS_INFO("task1_per_delivery is defined as: %f", task1_per_delivery);
    } else {
        task1_per_delivery = 10.0;
        ROS_INFO("Failed to get param 'task1_per_delivery' use default '%f'", task1_per_delivery);
    }

    if (n.getParam("task1_per_correct_category", task1_per_correct_category)) {
        ROS_INFO("task1_per_correct_category is defined as: %f", task1_per_correct_category);
    } else {
        task1_per_correct_category = 10.0;
        ROS_INFO("Failed to get param 'task1_per_correct_category' use default '%f'", task1_per_correct_category);
    }
    
    ros::Time::init();

    //collision_score = 0.0;
    prev_detect_cb = ros::Time::now();

    ros::Publisher pub = n.advertise<std_msgs::Float32>("/score", 1000);
    ros::Publisher pubmsg = n.advertise<std_msgs::String>("message", 1000);
    ros::Rate rate(10);

    ros::Subscriber sub_drawerleft_any = n.subscribe<std_msgs::Int16>("/any_in_drawerleft_detector/count", 1, boost::bind(&cb_count_delivery, "drawerleft", _1));
    ros::Subscriber sub_drawerleft_cat = n.subscribe<std_msgs::Int16>("/shapeitems_in_drawerleft_detector/count", 1, boost::bind(&cb_count_correct_category, "drawerleft", _1));

    ros::Subscriber sub_drawertop_any = n.subscribe<std_msgs::Int16>("/any_in_drawertop_detector/count", 1, boost::bind(&cb_count_delivery, "drawertop", _1));
    ros::Subscriber sub_drawertop_cat = n.subscribe<std_msgs::Int16>("/tools_in_drawertop_detector/count", 1, boost::bind(&cb_count_correct_category, "drawertop", _1));

    ros::Subscriber sub_drawerbottom_any = n.subscribe<std_msgs::Int16>("/any_in_drawerbottom_detector/count", 1, boost::bind(&cb_count_delivery, "drawerbottom", _1));
    ros::Subscriber sub_drawerbottom_cat = n.subscribe<std_msgs::Int16>("/tools_in_drawerbottom_detector/count", 1, boost::bind(&cb_count_correct_category, "drawerbottom", _1));

    ros::Subscriber sub_containera_any = n.subscribe<std_msgs::Int16>("/any_in_containera_detector/count", 1, boost::bind(&cb_count_delivery, "containera", _1));
    ros::Subscriber sub_containera_cat = n.subscribe<std_msgs::Int16>("/kitchenitems_in_containera_detector/count", 1, boost::bind(&cb_count_correct_category, "containera", _1));

    ros::Subscriber sub_containerb_any = n.subscribe<std_msgs::Int16>("/any_in_containerb_detector/count", 1, boost::bind(&cb_count_delivery, "containerb", _1));
    ros::Subscriber sub_containerb_largemarker = n.subscribe<std_msgs::Int16>("/largemarker_in_containerb_detector/count", 1, boost::bind(&cb_count_correct_category, "containerb_largemarker", _1));
    ros::Subscriber sub_containerb_fork = n.subscribe<std_msgs::Int16>("/fork_in_containerb_detector/count", 1, boost::bind(&cb_count_correct_category, "containerb_fork", _1));
    ros::Subscriber sub_containerb_spoon = n.subscribe<std_msgs::Int16>("/spoon_in_containerb_detector/count", 1, boost::bind(&cb_count_correct_category, "containerb_spoon", _1));

    ros::Subscriber sub_traya_any = n.subscribe<std_msgs::Int16>("/any_in_traya_detector/count", 1, boost::bind(&cb_count_delivery, "traya", _1));
    ros::Subscriber sub_traya_cat = n.subscribe<std_msgs::Int16>("/foods_in_traya_detector/count", 1, boost::bind(&cb_count_correct_category, "traya", _1));

    ros::Subscriber sub_trayb_any = n.subscribe<std_msgs::Int16>("/any_in_trayb_detector/count", 1, boost::bind(&cb_count_delivery, "trayb", _1));
    ros::Subscriber sub_trayb_cat = n.subscribe<std_msgs::Int16>("/foods_in_trayb_detector/count", 1, boost::bind(&cb_count_correct_category, "trayb", _1));

    ros::Subscriber sub_bina_any = n.subscribe<std_msgs::Int16>("/any_in_bina_detector/count", 1, boost::bind(&cb_count_delivery, "bina", _1));
    ros::Subscriber sub_bina_cat = n.subscribe<std_msgs::Int16>("/taskitems_in_bina_detector/count", 1, boost::bind(&cb_count_correct_category, "bina", _1));

    ros::Subscriber sub_binb_any = n.subscribe<std_msgs::Int16>("/any_in_binb_detector/count", 1, boost::bind(&cb_count_delivery, "binb", _1));
    ros::Subscriber sub_binb_cat = n.subscribe<std_msgs::Int16>("/taskitems_in_binb_detector/count", 1, boost::bind(&cb_count_correct_category, "binb", _1));

    ros::Subscriber sub = n.subscribe("/undesired_contact_detector/detect", 1, cb_detect);
    ros::Subscriber sub_hsrb_in_room2 = n.subscribe<std_msgs::Int16>("/hsrb_in_room2_detector/count", 1, cb_hsrb_in_room2);

    task1_delivery_score = 0.0;
    task1_category_score = 0.0;
    task2a_score = 0.0;

    double prev_score = 0.0;
    while (ros::ok()) {
        std_msgs::Float32 msg;
        double score = task1_delivery_score + task1_category_score + task2a_score;
        if (fabs(score - prev_score) > 0.1) {
            ROS_WARN("[WRS] Your score has been changed to %i (%+i).", (int)score, (int)(score - prev_score));
          prev_score = score;
        }
        msg.data = score;
        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
}
