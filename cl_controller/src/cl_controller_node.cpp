#include <ros/ros.h>
#include <ClController.h>

int main(int argc, char** argv)
{
	ros::init(argc,argv, "cl_contoller");
	ros::NodeHandle nodehandle("~");

	cl_controller::ClController clController(nodehandle);

	ros::spin();
return 0;

}
