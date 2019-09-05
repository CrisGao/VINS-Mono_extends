

#if 1
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
 #include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main( int argc, char **argv )
{
    int capture_width = 3280 ;
    int capture_height = 2464 ;
    int display_width = 1280;
    int display_height = 720;
    int framerate =20;
    int flip_method = 2;

    std::string pipeline = gstreamer_pipeline(capture_width,
	capture_height,
	display_width,
	display_height,
	framerate,
	flip_method);

  ros::init( argc, argv, "img_publisher" );
  ros::NodeHandle n;

  // Open camera with CAMERA_INDEX (webcam is typically #0).
  cv::VideoCapture capture( pipeline,cv::CAP_GSTREAMER); //摄像头视频的读操作
  if( not capture.isOpened() )
  {
    ROS_ERROR_STREAM(
      "Failed to open camera with index " << pipeline << "!"
    );
    ros::shutdown();
  }
  //1 捕获视频
  
  //2 创建ROS中图像的发布者
  image_transport::ImageTransport it( n ); 
  image_transport::Publisher pub_image = it.advertise( "camera/image", 1 );

  
  //cv_bridge功能包提供了ROS图像和OpenCV图像转换的接口，建立了一座桥梁
  		
  cv_bridge::CvImagePtr frame = boost::make_shared< cv_bridge::CvImage >();
 frame->encoding = sensor_msgs::image_encodings::BGR8;

  ros::Rate loop_rate(30);
  while( ros::ok() ) {
    capture >> frame->image; //流的转换
    if( frame->image.empty() )
    {
      ROS_ERROR_STREAM( "Failed to capture frame!" );
      ros::shutdown();
    }
	//打成ROS数据包
    frame->header.stamp = ros::Time::now();
    frame->header.frame_id = "camera_link";
    pub_image.publish( frame->toImageMsg() );

    //cv::waitKey( 3 );//opencv刷新图像 3ms
    ros::spinOnce();
    loop_rate.sleep();
  }

  capture.release();  //释放流
  return EXIT_SUCCESS;
}
#endif

