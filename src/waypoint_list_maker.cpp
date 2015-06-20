/*
 * Copyright (c) 2015, Scott K Logan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "waypoint_list_maker/waypoint_list_maker.hpp"

#include <iostream>
#include <fstream>

namespace waypoint_list_maker
{
	WaypointListMaker::WaypointListMaker(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
		: nh(nh),
		  nh_priv(nh_priv),
		  add_point_srv(nh_priv.advertiseService("add_point", &WaypointListMaker::addPointCallback, this)),
		  clear_points_srv(nh_priv.advertiseService("clear_points", &WaypointListMaker::clearPointsCallback, this)),
		  write_points_srv(nh_priv.advertiseService("write_points", &WaypointListMaker::writePointsCallback, this)),
		  waypoint_list_filename("waypoint_list.txt"),
		  frame_id("map"),
		  child_frame_id("base_footprint")
	{
		nh.param("waypoint_list_filename", waypoint_list_filename, waypoint_list_filename);
	}

	bool WaypointListMaker::writeList(const char *filename) const
	{
		ROS_INFO("Writing waypoint list to '%s'", filename);

		std::ofstream ofs(filename, std::ios::out|std::ios::binary);

		uint32_t serial_size = ros::serialization::serializationLength(waypoint_list);
		boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);

		ros::serialization::OStream ostream(obuffer.get(), serial_size);
		ros::serialization::serialize(ostream, waypoint_list);
		ofs.write((char*) obuffer.get(), serial_size);
		ofs.close();

		return true;
	}

	bool WaypointListMaker::addPointCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
	{
		tf::StampedTransform transform;
		geometry_msgs::PoseStamped pose;

		try
		{
			listener.lookupTransform(frame_id, child_frame_id, ros::Time(0), transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			return false;
		}

		tf::poseTFToMsg(transform, pose.pose);
		pose.header.frame_id = frame_id;

		ROS_INFO("Adding a new waypoint at (%lf, %lf)", pose.pose.position.x, pose.pose.position.y);

		waypoint_list.push_back(pose);

		return true;
	}

	bool WaypointListMaker::clearPointsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
	{
		ROS_INFO("Clearing the waypoint list...");

		waypoint_list.clear();

		return true;
	}

	bool WaypointListMaker::writePointsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
	{
		return writeList(waypoint_list_filename.c_str());
	}
}
