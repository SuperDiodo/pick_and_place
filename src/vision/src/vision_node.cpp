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
	float x_camera_frame = 1.087;
	float y_camera_frame = -1.241;
	float z_surface = 0.84;
	float m_width = 2.45; // how many meters in the width of the image
	float m_height = 1.75;

public:
  ImageConverter(): it(nh)
  {
		this->cloud = 0;
    // Subscrive to input image and point cloud msg, and pose array in output
		point_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, &ImageConverter::processCloud, this);
		image_sub = it.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageCb, this);
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
		int u = 250;
		int v = 250;
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

		cout << "XC " << X << " YC " << Y << " ZC " << Z << endl;
		*/
	}
	
	// find coordinates through pc
	void findCoordinatesPC(int u, int v)
	{
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
		//cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
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
		
		vector<vector<cv::Point> > contours_poly( contours.size() );
		vector<cv::Rect> boundRect( contours.size() );

		for( size_t i = 0; i < contours.size(); i++ )
		{
				// find centroids
				cv::Moments mu = cv::moments(contours[i]);
				cv::Point centroid = cv::Point (mu.m10/mu.m00 , mu.m01/mu.m00);
				
				// find bounding box
				approxPolyDP( contours[i], contours_poly[i], 3, true );
				boundRect[i] = boundingRect( contours_poly[i] );
				//rectangle( canny, boundRect[i].tl(), boundRect[i].br(), 255, 2 );
				

				// find (x,y,yaw) in global frame
				p.position.x = (centroid.y*m_per_px)+x_camera_frame; // X in global frame
				p.position.y = (centroid.x*m_per_px)+y_camera_frame;	// Y in global frame
				p.position.z = z_surface;
				p.orientation.z = findOrientation(canny, boundRect[i]);
				pose_array.poses.push_back(p);

				circle( thres, centroid, 2, cv::Scalar(0), 4 );
				cout << " ----------------- " << endl;
				cout << " X " << pose_array.poses[i].position.x << " Y " << pose_array.poses[i].position.y << " Yaw " << pose_array.poses[i].orientation.z << endl;
				//cout << " point cloud " << endl;
				//if(this->cloud != NULL)
				//	findCoordinatesPC(centroid.x, centroid.y);
		}

		// setup PoseArray message header
		pose_array.header.stamp = ros::Time::now();
		pose_array.header.frame_id = "map";
		pose_pub.publish(pose_array);
		pose_array.poses.clear();

		
		cv::namedWindow(OPENCV_WINDOW);
		cv::imshow(OPENCV_WINDOW, thres);
		cv::waitKey(0);
		
	
	}

	float findOrientation(cv::Mat img, cv::Rect roi)
	{
		/*
			NOTE: the angle returned isn't the effective yaw of the object, but the anghe
			at which end effector should grasp the object (long side of the object).
		*/
		cv::Mat background = cv::Mat::zeros(img.size(), CV_8UC1);
		cv::Mat mask = cv::Mat(roi.size(), CV_8UC1, 255);
		mask.copyTo(background(roi));

		// img containing only one object
		cv::Mat img_mask = img.mul(background);
				
		// find orientation
		float angle_step;
		if(mask.rows < 100 && mask.cols < 100)
		{
			angle_step = 3*(CV_PI/180);
		} else {
			angle_step = 2*(CV_PI/180);
		}
		cv::Mat lines;
		cv::HoughLines(img_mask, lines, 1, angle_step, round(max(mask.rows,mask.cols)/2), 0, 0 );
		
		// draw lines
		// theta is the angle (radians wrt x axis in camera frame) of the line perpendicular
		// to the longest lateral surface of the object
		float theta = 0.0;
		for( int i = 0; i < lines.rows; i++ )
    {
        float rho = lines.at<float>(i,0);
				theta = lines.at<float>(i,1);
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        cv::line( img_mask, pt1, pt2, 190, 1, cv::LINE_AA);
				
    }
		  
		// --------------
		
		// omega is the angle between the longest lateral surface of the object and x axis (camera frame)
		float omega = 0.0;
		if(theta <= CV_PI/2){
			omega = CV_PI - (CV_PI/2-theta);
		} else if (theta <= CV_PI){	
			omega = CV_PI/2 - (CV_PI-theta);
		}
		cout << "rad " << CV_PI-omega << " deg " << omega/(CV_PI/180) << endl;
		return omega;
		
/*
		cv::namedWindow(OPENCV_WINDOW);
		cv::imshow(OPENCV_WINDOW, img_mask);
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
