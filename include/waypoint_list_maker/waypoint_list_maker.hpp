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

#ifndef _waypoint_list_maker_hpp
#define _waypoint_list_maker_hpp

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

namespace waypoint_list_maker
{
	class WaypointListMaker
	{
	public:
		WaypointListMaker(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);

	private:
		bool writeList(const char *filename) const;
		bool addPointCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool clearPointsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool writePointsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

		ros::NodeHandle nh;
		ros::NodeHandle nh_priv;

		ros::ServiceServer add_point_srv;
		ros::ServiceServer clear_points_srv;
		ros::ServiceServer write_points_srv;

		tf::TransformListener listener;

		std::string waypoint_list_filename;
		std::string frame_id;
		std::string child_frame_id;
		std::vector<geometry_msgs::PoseStamped> waypoint_list;
	};
}

#endif /* _waypoint_list_maker_hpp */
