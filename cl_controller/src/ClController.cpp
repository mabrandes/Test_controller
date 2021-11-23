#include <ClController.h>

namespace cl_controller {

ClController::ClController(ros::NodeHandle& nodehandle)
	: nodehandle_(nodehandle)
{
	if (!readParameter()){
		ROS_ERROR("Could not read parameter.");
		ros::requestShutdown();
	}

	sub_poseimu=nodehandle_.subscribe(subscriberTopic_pose_, 100 , &ClController::callback_poseimu, this);
	sub_wo=nodehandle_.subscribe(subscriberTopic_wo_, 100 , &ClController::callback_wo, this);

	pub_error=nodehandle_.advertise<std_msgs::Float64>("pose_error", 20);


	ROS_INFO("Successfully launched Node.");

}

ClController::~ClController()
{
}

void ClController::callback_poseimu(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message)
{
	//local variable
	double x;
	x = message->pose.pose.position.x;
	ROS_INFO("the position value from State estimation is %lf \n ",x);
    //pose.push_back(message);
}

void ClController::callback_wo(const gazebo_msgs::ModelStates::ConstPtr& message)
{
	double x;
	x = message->pose[1].position.x;
	ROS_INFO("the position form odometry value is %lf \n ",x);

}


bool ClController::readParameter()
{
	//even tough getParameter is called as a condition it actually got executed --> we dont need to call it again i the else statement
	if (!nodehandle_.getParam("pose_ref", subscriberTopic_pose_) || !nodehandle_.getParam("pose_meas", subscriberTopic_wo_)) return false;
	return true;

}

}
