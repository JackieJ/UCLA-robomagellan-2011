#include <ros/ros.h>

// Service includes
#include "glados/LLtoUTM.h"
#include "glados/UTMtoLL.h"

#include "conversions.h" // namespace UTM

#include <iostream>

bool LLtoUTM(glados::LLtoUTM::Request &request, glados::LLtoUTM::Response &response)
{
	std::string zone; // unused
	UTM::LLtoUTM(request.latitude, request.longitude, response.northing, response.easting, zone);
	std::cout << "LLtoUTM zone: " << zone << std::endl;
	return true;
}

// For zone: Use 11S for Los Angeles, 10S for San Mateo
bool UTMtoLL(glados::UTMtoLL::Request &request, glados::UTMtoLL::Response &response)
{
	UTM::UTMtoLL(request.northing, request.easting, request.zone, response.latitude, response.longitude);
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "utm_server");
	ros::NodeHandle n;

	ros::ServiceServer LLtoUTM_service = n.advertiseService("LLtoUTM", LLtoUTM);
	ros::ServiceServer UTMtoLL_service = n.advertiseService("UTMtoLL", UTMtoLL);

	ros::spin();
	return 0;
}
