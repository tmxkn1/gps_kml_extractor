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
#include <ros/package.h>
#include <signal.h>
#include <string>
#include <fstream>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

std::string kml_filename, utm_filename;
std::ofstream utm_fs, kml_fs;
std::ifstream kml_footer;
bool utm_time_enabled;

ros::Subscriber fix_sub;
ros::Subscriber odom_sub;

void copyFileData(std::ifstream &ifs, std::ofstream &ofs);
void setupKML(ros::NodeHandle node, ros::NodeHandle private_nh);
void setupUTM(ros::NodeHandle node);
void callbackFix(const sensor_msgs::NavSatFixConstPtr &fix);
void callbackOdom(const nav_msgs::OdometryConstPtr &odom);
void sigintHandler(int a);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "kml_extractor_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    signal(SIGINT, sigintHandler);
    private_nh.param<std::string>("kml_file", kml_filename, "gps.kml");
    private_nh.param<std::string>("utm_file", utm_filename, "utm.txt");
    private_nh.param<bool>("utm_extract_time", utm_time_enabled, true);
    if (!kml_filename.empty())
    {
        setupKML(node, private_nh);
    }
    if (!utm_filename.empty())
    {
        setupUTM(node);
    }

    ros::spin();

    return 0;
}

void copyFileData(std::ifstream &ifs, std::ofstream &ofs)
{
    while (ifs.good())
    {
        char c = ifs.get();
        if (ifs.good())
            ofs << c;
    }
    ofs << std::endl;
}

void setupKML(ros::NodeHandle node, ros::NodeHandle private_nh)
{
    std::string pkg_loc = ros::package::getPath("gps_kml_extractor");
    std::string header_file_path = pkg_loc + "/src/include/kml_header";
    std::string footer_file_path = pkg_loc + "/src/include/kml_footer";

    std::ifstream kml_header(header_file_path);
    if (kml_header.fail())
    {
        ROS_ERROR_STREAM("Failed to open header file \"" << header_file_path << "\".");
        return;
    }
    kml_footer.open(footer_file_path, std::ios_base::in);
    if (kml_footer.fail())
    {
        ROS_ERROR_STREAM("Failed to open footer file \"" << footer_file_path << "\".");
        return;
    }

    kml_fs.open(kml_filename.c_str(), std::ios_base::out | std::ios_base::trunc);
    if (kml_fs.fail())
    {
        ROS_ERROR_STREAM("Failed to open KML file \"" << kml_filename << "\".");
        return;
    }
    copyFileData(kml_header, kml_fs);

    fix_sub = node.subscribe("fix", 30, callbackFix);
    ROS_INFO_STREAM("KML file \"" << kml_filename << "\" opened.\n"
                                  << "Waiting for topic \"" << fix_sub.getTopic() << "\"...");
}

void setupUTM(ros::NodeHandle node)
{
    utm_fs.open(utm_filename.c_str(), std::ios_base::out | std::ios_base::trunc);
    if (utm_fs.fail())
    {
        ROS_ERROR_STREAM("Failed to open UTM file \"" << utm_filename << "\".");
        return;
    }
    odom_sub = node.subscribe("odom", 30, callbackOdom);

    ROS_INFO_STREAM("UTM file \"" << utm_filename << "\" opened.\n"
                                  << "Waiting for topic \"" << odom_sub.getTopic() << "\"...");
}

void callbackFix(const sensor_msgs::NavSatFixConstPtr &fix)
{
    ROS_INFO_ONCE("Fix received. Extracting data...");
    kml_fs << std::setprecision(12) << fix->longitude << "," << fix->latitude << "," << fix->altitude << ",";
}

void callbackOdom(const nav_msgs::OdometryConstPtr &odom)
{
    ROS_INFO_ONCE("UTM odom received. Extracting data...");
    if (utm_time_enabled)
    {
        utm_fs << std::setprecision(12) << odom->header.stamp.sec;
        utm_fs << ".";
        utm_fs << odom->header.stamp.nsec;
        utm_fs << ",";
    }
    utm_fs << std::setprecision(12) << odom->pose.pose.position.x;
    utm_fs << ",";
    utm_fs << odom->pose.pose.position.y;
    utm_fs << ",";
    utm_fs << odom->pose.pose.position.z;
    utm_fs << std::endl;
}

void sigintHandler(int a)
{
    copyFileData(kml_footer, kml_fs);
    ros::requestShutdown();
}
