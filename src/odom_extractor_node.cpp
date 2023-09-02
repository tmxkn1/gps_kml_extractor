/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Zhengyi Jiang
 *  Copyright (c) 2013, ISR University of Coimbra.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the ISR University of Coimbra nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Zhengyi Jiang 01/09/2023
 * Author: Joao Sousa on 18/04/2014
 *********************************************************************/
#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <fstream>
#include <nav_msgs/Odometry.h>

std::string filename;
std::ofstream fs;
bool odom_time_enabled;
ros::Subscriber odom_sub;

void callbackOdom(const nav_msgs::OdometryConstPtr &odom);

void setupFile(ros::NodeHandle node)
{
    fs.open(filename.c_str(), std::ios_base::out | std::ios_base::trunc);
    if (fs.fail())
    {
        ROS_ERROR_STREAM("Failed to open file \"" << filename << "\".");
        return;
    }
    odom_sub = node.subscribe("odom", 30, callbackOdom);

    ROS_INFO_STREAM("File \"" << filename << "\" opened.\n"
                              << "Waiting for topic \"" << odom_sub.getTopic() << "\"...");
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom_extractor_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("odom_file", filename, "odom.txt");
    private_nh.param<bool>("odom_extract_time", odom_time_enabled, true);
    if (!filename.empty())
    {
        setupFile(node);
    }

    ros::spin();

    return 0;
}

void callbackOdom(const nav_msgs::OdometryConstPtr &odom)
{
    ROS_INFO_ONCE("Odom received. Extracting data...");
    if (odom_time_enabled)
    {
        fs << std::setprecision(12) << odom->header.stamp.sec;
        fs << ".";
        fs << odom->header.stamp.nsec;
        fs << ",";
    }
    fs << std::setprecision(12) << odom->pose.pose.position.x;
    fs << ",";
    fs << odom->pose.pose.position.y;
    fs << ",";
    fs << odom->pose.pose.position.z;
    fs << std::endl;
}
