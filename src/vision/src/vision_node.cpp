#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float64.h>


using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
	ros::Subscriber point_sub;
	sensor_msgs::PointCloud2ConstPtr cloud;
	geometry_msgs::PoseArray pose_array;
	geometry_msgs::Pose p;
	ros::Publisher pose_pub;

	cv::Mat gray;
	cv::Mat copy;
	cv::Mat thres;
	cv::Mat canny;
	cv::Mat drawing;

	// x and y coordinater of camera frame wrt global frame
	float x_camera_frame = -0.1033;
	float y_camera_frame = -0.5660;
	float m_width = 1.12; // how many meters in the width of the image

public:
  ImageConverter(): it(nh)
  {
		this->cloud = 0;
    // Subscrive to input image and point cloud msg, and pose array in output
    image_sub = it.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageCb, this);
		//point_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, &ImageConverter::processCloud, this);
		pose_pub = nh.advertise<geometry_msgs::PoseArray>("/poses",1);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

	void processCloud(const sensor_msgs::PointCloud2ConstPtr& cloud)
	{
		// store the new pointcloud, we will use it later
		this->cloud = cloud;
	/*
		int u = 174;
		int v = 219;
		// get width and height of 2D point cloud data
    int width = this->cloud->width;
    int height = this->cloud->height;

		// Convert from u (column / width), v (row/height) to position in array
    // where X,Y,Z data starts
    int arrayPosition = v*this->cloud->row_step + u*this->cloud->point_step;

		// compute position in array where x,y,z data start
    int arrayPosX = arrayPosition + this->cloud->fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + this->cloud->fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + this->cloud->fields[2].offset; // Z has an offset of 8

		float X = 0.0; // Y in global frame
    float Y = 0.0;	// X (world) = X(world) kinect - sgn(Y)
    float Z = 0.0;

    memcpy(&X, &this->cloud->data[arrayPosX], sizeof(float));
    memcpy(&Y, &this->cloud->data[arrayPosY], sizeof(float));
    memcpy(&Z, &this->cloud->data[arrayPosZ], sizeof(float));

		cout << "X " << X << " Y " << Y << " Z " << Z << endl;
		*/
	}
	
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {	
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
			cout << "log : ERROR!" << endl;
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	
		//cv::Mat img = cv::imread("/home/vito/Moveit_ROS/src/vision/src/test.jpg", 0);
		cv::cvtColor(cv_ptr->image, gray, cv::COLOR_RGB2GRAY);
		findCoordinates(gray);
  }

	

	void findCoordinates(cv::Mat img)
	{
		// parameter for coordinates conversion
		float m_per_px = m_width/static_cast<float>(img.cols);
		float X_loc, Y_loc;

		// find shapes and centers
		cv::threshold(img, thres, 100, 255, cv::THRESH_BINARY_INV );
		cv::Canny(img, canny, 100, 200);
		vector<vector<cv::Point> > contours;
		findContours( canny, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE );
		
		for( size_t i = 0; i < contours.size(); i++ )
		{
				// find centroids
				cv::Moments mu = cv::moments(contours[i]);
				cv::Point centroid = cv::Point (mu.m10/mu.m00 , mu.m01/mu.m00);
				
				// find (x,y) in global frame
				p.position.x = (centroid.y*m_per_px)+x_camera_frame; // X in global frame
				p.position.y = (centroid.x*m_per_px)+y_camera_frame;	// Y in global frame
				p.position.z = 1.04;
				pose_array.poses.push_back(p);

				circle( thres, centroid, 2, cv::Scalar(0), 4 );
				cout << "X " << pose_array.poses[i].position.x << " Y " << pose_array.poses[i].position.y << endl;
		}

		// setup PoseArray message header
		cout << "TIME " << ros::Time::now() << endl;
		pose_array.header.stamp = ros::Time::now();
		pose_array.header.frame_id = "map";
		pose_pub.publish(pose_array);
		pose_array.poses.clear();

		/*
		cv::namedWindow(OPENCV_WINDOW);
		cv::imshow(OPENCV_WINDOW, thres);
		cv::waitKey(0);
*/
	
	}
};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();

  return 0;
}
