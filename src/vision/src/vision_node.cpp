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
#include "ImagePreProcessor.cpp"
#include <tf/transform_datatypes.h>

using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
	ImagePreProcessor pr;
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
	ros::Subscriber point_sub;
	sensor_msgs::PointCloud2ConstPtr cloud;
	geometry_msgs::PoseArray pose_array;
	ros::Publisher pose_pub;

	float x_camera = 1.5;
	cv::Mat gray;
	cv::Mat copy;
	cv::Mat thres;
	cv::Mat canny;
	cv::Mat drawing;

	// the z coordinate is fixed, we can also calculate from PC
	float z_surface = 0.84;

public:
  ImageConverter(): it(nh)
  {
		this->cloud = NULL;

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
	}

	// find coordinates through pc
	geometry_msgs::Pose findCoordinatesPC(int u, int v)
	{
		// get width and height of 2D point cloud data
    int width = this->cloud->width;
    int height = this->cloud->height;
		geometry_msgs::Pose p;

		// Convert from u (column / width), v (row/height) to position in array
    // where X,Y,Z data starts
    int arrayPosition = v*this->cloud->row_step + u*this->cloud->point_step;

		// compute position in array where x,y,z data start
    int arrayPosX = arrayPosition + this->cloud->fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + this->cloud->fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + this->cloud->fields[2].offset; // Z has an offset of 8

		float X = 0.0; // Y in global frame
    float Y = 0.0;
    float Z = 0.0;

    memcpy(&X, &this->cloud->data[arrayPosX], sizeof(float));
    memcpy(&Y, &this->cloud->data[arrayPosY], sizeof(float));
    memcpy(&Z, &this->cloud->data[arrayPosZ], sizeof(float));

		p.position.x = Y+this->x_camera; // X in global frame
		p.position.y = X;	// Y in global frame
		p.position.z = z_surface;

		return p;
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

		cv::cvtColor(cv_ptr->image, gray, cv::COLOR_RGB2GRAY);
		if (this->cloud != NULL)
			findCoordinates(gray);
  }

	void findCoordinates(cv::Mat img)
	{
		// find shapes and centers
		int threshold = pr.find_peak(gray, 0); // find the peak of histogram for thresholding
		cv::threshold(img, thres, threshold-10, 255, cv::THRESH_BINARY_INV );
		cv::Canny(img, canny, 100, 10);

		// find centers and bounding boxes of objects
		obj objects;
		objects = pr.findCentroids(canny);

		// find image with no lateral surfaces
		thres = pr.filter(gray, objects.boundRects, thres);
		cv::Canny(thres, canny, 100, 10);



		obj obj;
		obj = pr.findCentroids(canny);

		for( size_t i = 0; i < obj.centroids.size(); i++ )
		{
				cout << " ------- OBJ " << i << " ----------" << endl;
				// find (x,y,yaw) in global frame
				geometry_msgs::Pose p;
				p = findCoordinatesPC(obj.centroids[i].x, obj.centroids[i].y);

				tf::Quaternion orientation;
				orientation.setRPY(0, 0, findOrientation(canny, obj.boundRects[i]));

				quaternionTFToMsg(orientation, p.orientation);
				pose_array.poses.push_back(p);

				circle( thres, obj.centroids[i], 2, cv::Scalar(0), 4 );

				cout << " X " << pose_array.poses[i].position.x << " Y " << pose_array.poses[i].position.y << " Yaw " << pose_array.poses[i].orientation.z << endl;
				cout << "---------------" << endl;
		}

		// setup PoseArray message header
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
		float alpha = 0.0;
		for( int i = 0; i < lines.rows; i++ )
    {
        float rho = lines.at<float>(i,0);
				theta = lines.at<float>(i,1);
				alpha += theta;
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        cv::line( img_mask, pt1, pt2, 190, 1, cv::LINE_AA);

    }
		alpha = alpha/lines.rows;
		cout << lines << endl;
		cout << "alpha " << alpha << endl;
		// --------------

		// omega is the angle between the longest lateral surface of the object and x axis (camera frame)
		float omega = 0.0;
		if(theta <= CV_PI/2){
			omega = CV_PI - (CV_PI/2-alpha);
		} else if (theta <= CV_PI){
			omega = CV_PI/2 - (CV_PI-alpha);
		}
		//cout << "rad " << CV_PI-omega << " deg " << omega/(CV_PI/180) << " yaw " << omega <<endl;
/*
		cv::namedWindow(OPENCV_WINDOW);
		cv::imshow(OPENCV_WINDOW, img_mask);
		cv::waitKey(0);
*/
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
