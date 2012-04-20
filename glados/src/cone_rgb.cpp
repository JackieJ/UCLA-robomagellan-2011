#include <ros/ros.h>

// Image processing includes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

#include <iostream>

// Uncomment the /compressed line to use a compressed image
//#define COMPRESSED_IMAGE "/compressed"
#define COMPRESSED_IMAGE

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ConeFinder
{
public:
	ConeFinder();
	~ConeFinder() { cv::destroyWindow(WINDOW); }

	void receiveImage(const sensor_msgs::ImageConstPtr& msg);

private:
	ros::NodeHandle n;
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;
};

ConeFinder::ConeFinder() : it(n)
{
	image_sub = it.subscribe("/camera/image_raw" COMPRESSED_IMAGE, 1, &ConeFinder::receiveImage, this);
	cv::namedWindow(WINDOW);
}

void ConeFinder::receiveImage(const sensor_msgs::ImageConstPtr& msg)
{
	std::cout << "Received image (" << msg->encoding << ")" << std::endl;

	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Draw a cute little circle (but why?)
	if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
	{
		cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
	}

	cv::imshow(WINDOW, cv_ptr->image);
	cv::waitKey(3);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cone_rgba_node");
	ConeFinder coneFinder;
	ros::spin();
	return 0;
}
