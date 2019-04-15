#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>

#include <iostream>
using namespace cv;
using namespace std;

//https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html

int cnt=0;
ros::Publisher info_publisher;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{	
	cout<<"We have received image"<<endl;
	
	Mat img(cv_bridge::toCvShare(msg, "bgr8")->image);
	imshow( "Landmark Finder", img );  
	waitKey(0);
	
	
	int iLowH = 100; int iHighH = 140;
	int iLowS = 150; int iHighS = 255;
	int iLowV = 0; int iHighV = 255;
	
	
	Mat imgHSV;
	cvtColor(img, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
	
	Mat imgThresholded;
	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

	//morphological opening (removes small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

	//morphological closing (removes small holes from the foreground)
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

	//Calculate the moments of the thresholded image
	Moments oMoments = moments(imgThresholded);

	double dM01 = oMoments.m01;
	double dM10 = oMoments.m10;
	double dArea = oMoments.m00;
	
	std_msgs::String msg2;
	std::stringstream ss;
	// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
	if (dArea > 10000)
	{
		//calculate the position of the ball
		int posX = dM10 / dArea;
		int posY = dM01 / dArea;        
		
		ss<<"X = "<< posX << ", Y = " << posY <<endl;
		cout<<"X = "<< posX << ", Y = " << posY <<endl;
		//TODO: выводить тип кубика
		//TODO: выводить инфу по нескольким кубикам
	} 
	else 
	{
		cout<<"No object" <<endl;
		ss<<"No object" <<endl;
	}
	 
	//отсылаем строку с XY
	msg2.data = ss.str();
	info_publisher.publish(msg2);

	cout<<"We have processed image"<<endl;
	
	if(cnt++ == 0)
		imwrite( "/home/youbot/test.jpg", img );

}

int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "landmark_finder");
	
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub=it.subscribe(
	"camera/rgb/image_raw", 1, imageCallback);
	
	
	
	std::string topic_info="/landmark_xy";
	info_publisher = nh.advertise< std_msgs::String >(topic_info, 1000);	


	
	namedWindow( "Landmark Finder", WINDOW_AUTOSIZE );// Create a window for display.
	
	cout<<"Starting landmark finder node 2"<<endl;
	
	ros::spin();
	
}