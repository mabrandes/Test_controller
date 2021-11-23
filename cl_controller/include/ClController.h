#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ModelStates.h>

#include <vector>
#include <string>


namespace cl_controller {

class ClController
{
public:

	ClController(ros::NodeHandle& nodehandle);

	virtual ~ClController();

private:

	void callback_poseimu(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message);
	void callback_wo(const gazebo_msgs::ModelStates::ConstPtr& message);
	bool readParameter();

	ros::NodeHandle& nodehandle_;
	ros::Subscriber sub_poseimu;
	ros::Subscriber sub_wo;
	ros::Publisher pub_error;
	std::string subscriberTopic_pose_;
	std::string subscriberTopic_wo_;
	std::vector<geometry_msgs::PoseWithCovarianceStamped::ConstPtr> pose;
	float error;


};
}
