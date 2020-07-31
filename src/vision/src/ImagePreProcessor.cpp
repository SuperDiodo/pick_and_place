#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
struct obj
{
		 vector<cv::Point> centroids;
		 vector<cv::Rect> boundRects;
};

class ImagePreProcessor{

public:
	int find_peak(cv::Mat img, int binary){
		// binary = 0 takes count of 0 intensity in histogram
		// binary = 1 reject 0 intensity in histogram (used for black background img)
	  int histSize = 256;
    float range[] = { 0, 256 }; //the upper boundary is exclusive
    const float* histRange = { range };
    bool uniform = true, accumulate = false;
    cv::Mat hist;
    cv::calcHist( &img, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );

		int max = 0;
		int j = 0;
		for(int i=binary; i<hist.rows; i++){
			if((int)hist.at<float>(i) > max){
				max = (int)hist.at<float>(i);
				j = i;
			}
		}

		return j;
	}

	cv::Mat filter(cv::Mat gray, vector<cv::Rect> roi, cv::Mat thres){

		cv::Mat out = cv::Mat::zeros(gray.size(), CV_8UC1);
		for(int i=0; i<roi.size(); i++){
			cv::Mat background = cv::Mat::zeros(gray.size(), CV_8UC1);
			cv::Mat mask = cv::Mat(roi[i].size(), CV_8UC1, 1);
			mask.copyTo(background(roi[i])); // put roi mask in background Mat

			cv::Mat binary(thres.size(), thres.type());
			cv::threshold(thres, binary, 250, 1, cv::THRESH_BINARY ); //background 0, objects 1 (filter img)
			cv::Mat box = binary.mul(background); // single box filter img

			cv::Mat gray_mask = gray.mul(box); // grayscale filtered img (only box and black background)

			int peak = find_peak(gray_mask, 1);

			cv::blur(gray_mask, gray_mask, cv::Size(3,3));
			cv::Mat surface(thres.size(), thres.type()); // only top surface of the object
			cv::threshold(gray_mask, surface, peak-15, 255, cv::THRESH_BINARY );

			cv::Mat canny;
			cv::Canny(surface, canny, 100, 0);

			out = out + surface;

		}
		return out;

	}

	vector<cv::Rect> boundRects(cv::Mat canny){

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
		}

		return boundRect;
	}

	obj findCentroids(cv::Mat canny){
		obj objects;
		vector<vector<cv::Point> > contours;
		findContours( canny, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE );

		vector<vector<cv::Point> > contours_poly( contours.size() );
		vector<cv::Rect> boundRect( contours.size() );
		vector<cv::Point> centroids( contours.size() );
		for( int i = 0; i < contours.size(); i++ )
		{
				// find centroids
				approxPolyDP( contours[i], contours_poly[i], 3, true );
				boundRect[i] = boundingRect( contours_poly[i] );
				centroids[i] = cv::Point( (int) boundRect[i].x+(boundRect[i].width/2), (int) boundRect[i].y+(boundRect[i].height/2) );

		}

		objects.centroids = centroids;
		objects.boundRects = boundRect;
		return objects;

	}

};
