#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/geometry/Rotation.pb.h>

#include <mutex>
#include <thread>


using namespace rsb;
using namespace rsb::converter;

ros::Publisher vizPub;
tf::TransformListener *listener;
std::vector<std::string> robotNames;
static std::string expectedScope;
static std::string expectedSubscope;
static std::string expectedContent;
static std::size_t markerId = 0;
std::mutex idMtx;
double makerRemainTime;
int markerClass = 0;
double markerScaleX;
double markerScaleY;
double markerScaleZ;
double markerColorA;
double markerColorR;
double markerColorG;
double markerColorB;

void vizIt(std::string robots, std::string state, std::string content) {

  // Source and destination names of the robots
  std::string srcRobotName, dstRobotName;

  // Check subscope (don't care if empty)
  if (!expectedSubscope.empty()) {
    if ( state.compare(expectedSubscope) != 0 ) {
      ROS_INFO("Subscope '%s' does not match '%s'/n", expectedSubscope.c_str(), state.c_str());
//      std::cerr << "Subscope " << expectedSubscope.c_str() <<  " does not match " <<  state.c_str() << std::endl;;
      return;
    }
  }

  // Check content (don't care if empty)
  if (!expectedContent.empty()) {
    if ( content.compare(expectedContent) != 0 ) {
      ROS_INFO("Content '%s' does not match '%s'/n", expectedContent.c_str(), content.c_str());
//      std::cerr << "Content " << expectedContent.c_str() <<  " does not match " <<  content.c_str() << std::endl;;
      return;
    }
  }

  // Get the origin
  for ( int idx = 0; idx < robotNames.size(); ++idx) {
    boost::regex srcRobotNameRegex(std::string("([^ ]*)") + robotNames.at(idx) + std::string("([0-9]*)"));
    boost::cmatch charMatches;
    if (boost::regex_match(robots.c_str(), charMatches, srcRobotNameRegex)) {
      // Copy the found string
      srcRobotName = charMatches[1].str();
      for (int idy = 0; idy < charMatches.size(); ++idy) {
        ROS_DEBUG("match: %s\n", charMatches[idy].str().c_str());
      }
    }
  }
  // The rest must be the the destination
  dstRobotName = std::string(&robots.c_str()[srcRobotName.size()]);

  ROS_INFO("srcRobotName robot: %s/n", srcRobotName.c_str());
  ROS_INFO("dstRobotName robot: %s/n", dstRobotName.c_str());

  tf::StampedTransform transform;
  try{
    listener->lookupTransform(dstRobotName + std::string("/base_link"), srcRobotName + std::string("/base_link"), ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    return;
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = srcRobotName + std::string("/base_link");
  marker.header.stamp = ros::Time();
  marker.ns = "";
  idMtx.lock();
    if (markerId < std::numeric_limits<std::size_t>::max()) {
      marker.id = ++markerId;
    } else {
      markerId = 0;
    }
  idMtx.unlock();

  // Configure the marker
  if (markerClass == visualization_msgs::Marker::ARROW) {
    marker.type = visualization_msgs::Marker::ARROW;
    geometry_msgs::Point child, parent /* Is alread (0,0,0)*/;
    child.x = -transform.getOrigin().getX();
    child.y = -transform.getOrigin().getY();
    child.z = -transform.getOrigin().getZ();
    marker.points.push_back(parent);
    marker.points.push_back(child);
  } else if (markerClass == visualization_msgs::Marker::SPHERE) {
    marker.type = visualization_msgs::Marker::SPHERE;
  } else {
    ROS_ERROR("No suitable marker class for id %d. See http://wiki.ros.org/rviz/DisplayTypes/Marker", markerClass);
    return;
  }

//  marker.pose.position.x = -transform.getOrigin().getX();
//  marker.pose.position.y = -transform.getOrigin().getY();
//  marker.pose.position.z = -transform.getOrigin().getZ();
//  marker.pose.orientation.x = transform.getRotation().getX();
//  marker.pose.orientation.y = transform.getRotation().getY();
//  marker.pose.orientation.z = transform.getRotation().getZ();
//  marker.pose.orientation.w = transform.getRotation().getW();
  marker.scale.x = markerScaleX;
  marker.scale.y = markerScaleY;
  marker.scale.z = markerScaleZ;
  marker.color.a = markerColorA;
  marker.color.r = markerColorR;
  marker.color.g = markerColorG;
  marker.color.b = markerColorB;
  marker.mesh_resource = "";

  // Create the marker
  marker.action = visualization_msgs::Marker::ADD;
  vizPub.publish(marker);

  if (makerRemainTime > 0.0) {
    sleep(makerRemainTime);
    // Delete the marker
    marker.action = visualization_msgs::Marker::DELETE;
    vizPub.publish(marker);
  }
}

void vizScope(rsb::EventPtr msg) {
  // Get the parent scope
  ROS_INFO("REC: %s/n", msg->getScope().getComponents().at(0).c_str());

  // Communication sceme: /robot[1]anotherrobot[3]/[state] -> [Content as string]
  std::string robots(msg->getScope().getComponents().at(0));
  std::string state(msg->getScope().getComponents().at(1));
  std::string content(*boost::static_pointer_cast<std::string>(msg->getData()));

  std::thread t(vizIt, robots, state, content);
  t.detach();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "com_viz");
  ros::NodeHandle n("~");

  // ROS STUFF
  n.param<std::string>("rsb_viz_scope", expectedScope, "/amiro1tobi");
  n.param<std::string>("rsb_viz_subscope", expectedSubscope, "");
  n.param<std::string>("rsb_viz_content", expectedContent, "");
  n.param<int>("marker_class", markerClass, 0);
  n.param<double>("marker_remain_time", makerRemainTime, 1.0);
  n.param<double>("marker_scale_x", markerScaleX, 0.03);
  n.param<double>("marker_scale_y", markerScaleY, 0.06);
  n.param<double>("marker_scale_z", markerScaleZ, 1.0);
  n.param<double>("marker_color_a", markerColorA, 1.0);
  n.param<double>("marker_color_r", markerColorR, 0.0);
  n.param<double>("marker_color_g", markerColorG, 1.0);
  n.param<double>("marker_color_b", markerColorB, 0.0);

  listener = new tf::TransformListener;

  // Define possible names which may appear during communication
  robotNames.push_back(std::string("amiro"));
  robotNames.push_back(std::string("tobi"));
  robotNames.push_back(std::string("meka"));

  // RSB STUFF
  // Get the RSB factory
  rsb::Factory& factory = rsb::getFactory();

  // Prepare RSB listener for scopes
  rsb::ListenerPtr listener = factory.createListener(expectedScope);
  listener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&vizScope)));

  // ROS publisher for the marker
  vizPub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  ros::spin();
  return 0;
}
