#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <exception>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * Returns an OGM for a defined image
 */
nav_msgs::OccupancyGrid
getImageAsOgm(const std::string mapImagePath, const float imageResolution_meterPerCell, const std::string frameId) {

  nav_msgs::OccupancyGrid ogm;

  // LOAD
  cv::Mat image(cv::imread( mapImagePath, 1 ));
  if( !image.data ) {
    throw std::runtime_error("Loading image failed");
  }

  // CONVERT TO GRAY
  if (image.channels() == 3 /*It's an BGR image*/) {
    cvtColor( image, image, CV_BGR2GRAY );
  }

  // Fill the OGM content
  ogm.header.frame_id = frameId;
  ogm.header.stamp = ros::Time::now();
  ogm.info.width = image.cols;
  ogm.info.height = image.rows;
  ogm.info.resolution = imageResolution_meterPerCell;
  // ogm.info.origin // we let this at (0,0,0,...) and define the pose by a static transform via the frame_id
  for(int idx=0;idx < image.cols * image.rows; ++idx) {
      ogm.data.push_back(image.at<int8_t>(idx));
  }

  return ogm;
}

 /**
 * This programm publishes a map as OGM
 */
int
main(int argc, char **argv) {

  std::string ogmTopicArg;
  std::string frameIdArg;
  float imageResolutionArg_meterPerCell;
  nav_msgs::OccupancyGrid map;
  int rateArg;
  std::string mapImagePathArg;

  ros::init(argc, argv, "map_publisher");
  ros::NodeHandle n("~");

  ROS_INFO("Started map_publisher");

  n.param<std::string>("frame_id", frameIdArg, "world");
  n.param<std::string>("map_image_path", mapImagePathArg, "/foo/bar.png");
  n.param<std::string>("map_topic", ogmTopicArg, "/map");
  n.param<int>("rate", rateArg, 1);
  n.param<float>("image_resolution", imageResolutionArg_meterPerCell, 1.0f /*m/cell*/);
  ros::Rate rate(rateArg); // The rate with which we send

  // Load the image
  try {
    map = getImageAsOgm(mapImagePathArg, imageResolutionArg_meterPerCell, frameIdArg);
  } catch (std::exception &e) {
    std::cerr << "Caught exception: " << e.what() << std::endl;
    return -1;
  }

  ros::Publisher publisher = n.advertise<nav_msgs::OccupancyGrid>(ogmTopicArg, 1);

  while (n.ok())
  {
    publisher.publish(map);
    rate.sleep();
  }
  return 0;
}
