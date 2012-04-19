// ROS core
#include <ros/ros.h>
//Image message
#include <sensor_msgs/Image.h>
//pcl::toROSMsg
#include <pcl/io/pcd_io.h>
//stl stuff
#include <string>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

typedef unsigned char byte;

using namespace std;

bool StatisticalNoiseFilterOn;
int NoiseFilteringNoOfNeighbours;
float StdDevMulThreshold;

bool AmplitudeFilterOn;
float AmplitudeThreshold;


class PointCloudToUDP
{
public:
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg)
  {
	  if ((msg->width * msg->height) == 0)
		  return; //return if the cloud is not dense!
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	  	pcl::fromROSMsg(*msg, *cloud);
	
//	  for (size_t i = 0; i < 10000; ++i)
//	  {
////	    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
////	    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
//	    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
//		ROS_ERROR_STREAM( "    " << cloud->points[i].x << " " 
//		                        << cloud->points[i].y << " " 
//		                        << cloud->points[i].z);
//		
//	  }


//	  		ROS_ERROR_STREAM( "    " << msg->width << " " <<msg->height );
		
//		ROS_ERROR_STREAM( "    " << cloud->points[0].x << " " 
//				                 << cloud->points[0].y << " " 
//			                     << cloud->points[0].z);
		float * dist = new float [cloud->points.size()];
		for (int i=0; i<cloud->points.size();i++){
			dist[i]=cloud->points[i].x;
		}
		
		sendUDP(dist);
		for (int i=0; i<cloud->points.size();i++){
			dist[i]=cloud->points[i].y;
		}
		sendUDP(dist);
		for (int i=0; i<cloud->points.size();i++){
			dist[i]=cloud->points[i].z;
		}
		sendUDP(dist);


  }
  void setupSocket(){
		// SOCKET CODE
		int portno, n;
		struct sockaddr_in serv_addr;
		struct hostent *server;
		char buffer[256];
		portno = 9999;
		sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (sockfd < 0)
			perror("ERROR opening socket");
		server = gethostbyname("192.168.1.141");
		if (server == NULL) {
			fprintf(stderr,"ERROR, no such host\n");
			exit(0);
		}
		bzero((char *) &serv_addr, sizeof(serv_addr));
		serv_addr.sin_family = AF_INET;
		bcopy((char *)server->h_addr,
			(char *)&serv_addr.sin_addr.s_addr,
			server->h_length);
			serv_addr.sin_port = htons(portno);
		if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
			perror("ERROR connecting");
//		printf("Please enter the message: ");
		bzero(buffer,256);
		n = write(sockfd,buffer,strlen(buffer));
		if (n < 0)
			perror("ERROR writing to socket");
		bzero(buffer,256);
  }
  
  void sendUDP(float dist[]){
	   int numColumns = 200;
	   int numRows = 200;
	   int xsize = 100;
	   int ysize = 100;
	   int mult  = 2;
	   int n;
	   
	  uint16_t dist_low[xsize][ysize];
	  // take in an array of floats
	  for (int x=0; x<xsize; x++){
	          for (int y=0; y<ysize; y++){
	                  dist_low[x][y] = static_cast<uint16_t> ((dist[y*mult * numColumns + x * mult  ] /10.) * 65536) ;
	          }
	  }
	  const byte *send_buff = reinterpret_cast<const byte *>(dist_low);
	  
	  int bufsize = xsize * ysize * 2 / 4;
	  n = write(sockfd, "start" , strlen("start") );
	  n = write(sockfd,send_buff, bufsize );
	  n = write(sockfd, &(send_buff[bufsize]), bufsize  );
	  n = write(sockfd, &(send_buff[bufsize*2]), bufsize );
	  n = write(sockfd, &(send_buff[bufsize*3]), bufsize );
	  ROS_ERROR_STREAM( "sent" << bufsize<< " bytes ");
  }
  PointCloudToUDP () : cloud_topic_("input"),image_topic_("output")
  {
    sub_ = nh_.subscribe (cloud_topic_, 30, &PointCloudToUDP::cloud_cb, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image> (image_topic_, 30);

    //print some info about the node
//    std::string r_ct = nh_.resolveName (cloud_topic_);
    std::string r_it = nh_.resolveName (image_topic_);
//    ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
    ROS_INFO_STREAM("Publishing image on topic " << r_it );
   	setupSocket();
	ROS_ERROR_STREAM( "dones setup");	
  }
  
private:
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  ros::NodeHandle nh_;
  sensor_msgs::Image image_; //cache the image message
  std::string cloud_topic_; //default input
  std::string image_topic_; //default output
  ros::Subscriber sub_; //cloud subscriber
  ros::Publisher image_pub_; //image message publisher
  int sockfd;
};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "convert_pointcloud_to_image");
  PointCloudToUDP pci; //this loads up the node
  ros::spin (); //where she stops nobody knows
  return 0;
}
