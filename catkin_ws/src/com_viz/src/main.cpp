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
#include <rsb/converter/Repository.h>
#include <rsb/converter/PredicateConverterList.h>
#include <rsb/converter/ByteArrayConverter.h>

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
static std::string expectedDestroyContent;
static std::size_t markerId = 0;
std::mutex idMtx;
std::mutex creationScopeMtx;
double makerRemainTime;
int markerClass = 0;
double markerScaleX;
double markerScaleY;
double markerScaleZ;
double markerColorA;
double markerColorR;
double markerColorG;
double markerColorB;

// These are the lists which keep track of the already created markers
std::vector<std::string> creationScopes;
std::vector<std::size_t> creationScopesIdx;

enum markerState { create, update, destroy};

void vizIt(std::string robots, std::string state, std::string content) {

  // Source and destination names of the robots
  std::string srcRobotName, dstRobotName;

  // The marker
  visualization_msgs::Marker marker;

  // Variables to check if marker for that scope was already created
  markerState currentMarkerState = create;
  int creationScopeIdx = 0;

  // Check subscope (don't care if empty)
  if (!expectedSubscope.empty()) {
    if ( state.compare(expectedSubscope) != 0 ) {
      ROS_WARN("Subscope '%s' does not match '%s'\n", expectedSubscope.c_str(), state.c_str());
      return;
    }
  }

  // Check content (don't care if empty)
  if (!expectedContent.empty() || !expectedDestroyContent.empty()) {
    if ( content.compare(expectedContent) != 0 && content.compare(expectedDestroyContent) != 0 ) {
      ROS_WARN("Content '%s' does not match '%s' for creation, or '%s' for destruction\n", expectedContent.c_str(), content.c_str(), expectedDestroyContent.c_str());
      return;
    }
  }

  ROS_INFO("robots %s, state %s, content %s", robots.c_str(), state.c_str(), content.c_str());

  // This condition is just to keep track of the marker id
  // if we want to be able to erase it via a scope, and not by timeout
  if ( makerRemainTime < 0.0 ) {
  creationScopeMtx.lock();
    for( ; creationScopeIdx < creationScopes.size(); ++creationScopeIdx ) {
      if (creationScopes.at(creationScopeIdx).compare(robots) == 0) {
        if ( content.compare(expectedContent) == 0 ) {  // Update marker
          currentMarkerState = update;
          break;
        } else if ( content.compare(expectedDestroyContent) == 0 ) {  // Remove marker
          currentMarkerState = destroy;
          break;
        }
      }
    }

    if (creationScopeIdx == creationScopes.size() && content.compare(expectedDestroyContent) == 0) {
      ROS_INFO("Nothing to destroy");
      creationScopeMtx.unlock();
      return;
    }

    ROS_INFO("currentMarkerState %d", currentMarkerState);

      // Set the marker id and scope name
      if ( currentMarkerState == create ) {
        creationScopes.push_back(robots);
        if (markerId < std::numeric_limits<std::size_t>::max()) {
          ++markerId;
        } else {
          markerId = 0;
        }
        creationScopesIdx.push_back(markerId);
        marker.id = markerId;
      } else { // update or destroy marker
        marker.id = creationScopesIdx.at(creationScopeIdx);
        if ( currentMarkerState == destroy ) {
          creationScopes.erase(creationScopes.begin() + creationScopeIdx);
          creationScopesIdx.erase(creationScopesIdx.begin() + creationScopeIdx);
        }
      }
    creationScopeMtx.unlock();
  } else {
    marker.id = ++markerId;
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
      // The rest must be the destination
      dstRobotName = std::string(&robots.c_str()[srcRobotName.size()]);

      ROS_INFO("srcRobotName robot: %s\n", srcRobotName.c_str());
      ROS_INFO("dstRobotName robot: %s\n", dstRobotName.c_str());

      if ( dstRobotName.empty() || srcRobotName.empty() ) {
        ROS_WARN("Source or destination is empty");
        return;
      }

      marker.header.stamp = ros::Time();
      marker.ns = "";
      marker.scale.x = markerScaleX;
      marker.scale.y = markerScaleY;
      marker.scale.z = markerScaleZ;
      marker.color.a = markerColorA;
      marker.color.r = markerColorR;
      marker.color.g = markerColorG;
      marker.color.b = markerColorB;
      marker.mesh_resource = "";

      // Configure the marker
      if (markerClass == visualization_msgs::Marker::ARROW) {
        tf::StampedTransform transform;
        geometry_msgs::Point child, parent;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.header.frame_id = "map";
        try {
          listener->lookupTransform(marker.header.frame_id, dstRobotName + std::string("/base_link"), ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          return;
        }
        child.x = transform.getOrigin().getX();
        child.y = transform.getOrigin().getY();
        child.z = transform.getOrigin().getZ();

        try {
          listener->lookupTransform(marker.header.frame_id, srcRobotName + std::string("/base_link"), ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          return;
        }
        parent.x = transform.getOrigin().getX();
        parent.y = transform.getOrigin().getY();
        parent.z = transform.getOrigin().getZ();

        marker.points.push_back(parent);
        marker.points.push_back(child);
      } else if (markerClass == visualization_msgs::Marker::SPHERE) {
        marker.header.frame_id = srcRobotName + std::string("/base_link");
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


      if ( currentMarkerState == destroy ) {
        // Delete the marker
        marker.action = visualization_msgs::Marker::DELETE;
      } else {
        marker.action = visualization_msgs::Marker::ADD;
      }

      if ( makerRemainTime > 0.0 ) {
        marker.lifetime = ros::Duration(makerRemainTime);
      }
  // Create/destroy the marker
  vizPub.publish(marker);

  // Wait and destroy the marker
//  if ( makerRemainTime > 0.0 ) {
//    // Delete the marker
//    sleep(makerRemainTime);
//    marker.action = visualization_msgs::Marker::DELETE;
//    vizPub.publish(marker);
//  }
}

void vizScope(rsb::EventPtr msg) {
  // Get the parent scope
  ROS_INFO("REC: %s/n", msg->getScope().getComponents().at(0).c_str());

  // Communication sceme: /robot[1]anotherrobot[3]/[state] -> [Content as string]
  std::string robots(msg->getScope().getComponents().at(0));
  std::string state = "";
  if (msg->getScope().getComponents().size() > 1) {
    state = msg->getScope().getComponents().at(1);
  }
  std::string content;
  try {
    content = *boost::static_pointer_cast<std::string>(msg->getData());
  } catch(...) {
    return;
  }

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
  n.param<std::string>("rsb_viz_destroy_content", expectedDestroyContent, "");
  n.param<int>("marker_class", markerClass, 0);
  n.param<double>("marker_remain_time", makerRemainTime, 1.0);
  n.param<double>("marker_scale_x", markerScaleX, 0.03);
  n.param<double>("marker_scale_y", markerScaleY, 0.06);
  n.param<double>("marker_scale_z", markerScaleZ, 1.0);
  n.param<double>("marker_color_a", markerColorA, 1.0);
  n.param<double>("marker_color_r", markerColorR, 0.0);
  n.param<double>("marker_color_g", markerColorG, 1.0);
  n.param<double>("marker_color_b", markerColorB, 0.0);

  // Sanity check
  if ( !expectedDestroyContent.empty() && makerRemainTime > 0.0 ) {
    ROS_ERROR("Deactivate marker_remain_time by setting it to < 0 or let the rsb_viz_destroy_content empty");
  }

  listener = new tf::TransformListener;

  // Define possible names which may appear during communication
  robotNames.push_back(std::string("amiro"));
  robotNames.push_back(std::string("tobi"));
  robotNames.push_back(std::string("meka"));

  // RSB STUFF
  // Get the RSB factory
  rsb::Factory& factory = rsb::getFactory();

  // Set the logger level to OFF, so that we don't see WARN messages by not having a given converter
  rsc::logging::LoggerFactory::getInstance().getLogger()->setLevel(rsc::logging::Logger::Level::LEVEL_OFF);

  // Prepare RSB listener for scopes
  rsb::ListenerPtr listener = factory.createListener(expectedScope);
  listener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&vizScope)));

  // ROS publisher for the marker
  vizPub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  ros::spin();
  return 0;
}
