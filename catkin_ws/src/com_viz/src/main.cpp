#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>


#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
//#include <rsc/misc/SignalWaiter.h>
//#include <rsc/threading/PeriodicTask.h>
//#include <rsc/threading/ThreadedTaskExecutor.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

// RST Proto types
#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/geometry/Rotation.pb.h>

using namespace rsb;
using namespace rsb::converter;

static std::string parentFrame;
static std::string childFrame;
static std::string rsbScope;

ros::Publisher vis_pub;

tf::TransformListener *listener;

void vizScope(rsb::EventPtr msg) {
  // Get the parent scope
  std::cerr << "REC: " << msg->getScope().getComponents().at(0) << std::endl << std::flush;

//  tf::TransformListener listener;
  tf::StampedTransform transform;
//  try{
//    listener.lookupTransform(parentFrame, childFrame, ros::Time(0), transform);
//    std::string* error_msg = NULL;
//    listener->waitForTransform(childFrame, parentFrame, ros::Time(0), ros::Duration(1.0 /*sec*/), ros::Duration(0.01), error_msg);
//    if (error_msg != NULL) {
//      ROS_ERROR("Tf Error: %s",error_msg->c_str());
//      std::cerr << "Tf Error: " << *error_msg << std::endl << std::flush;
//      ros::Duration(1.0).sleep();
//    }
//  }
//  catch (tf::TransformException &ex){
//    ROS_ERROR("%s",ex.what());
//    ros::Duration(1.0).sleep();
//  }

    try{
      listener->lookupTransform(childFrame, parentFrame,
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }

  visualization_msgs::Marker marker;
  marker.header.frame_id = parentFrame;
  marker.header.stamp = ros::Time();
  marker.ns = "";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point child, parent;


//  std::cerr << "x: " << transform.getOrigin().x() << std::endl;
//  std::cerr << "y: " << transform.getOrigin().y() << std::endl;
//  std::cerr << "z: " << transform.getOrigin().z() << std::endl;
//
//  std::cerr << "x: " << transform.getOrigin().getX() << std::endl;
//    std::cerr << "y: " << transform.getOrigin().getY() << std::endl;
//    std::cerr << "z: " << transform.getOrigin().getZ() << std::endl;

  parent.x = 0.0;
  parent.y = 0.0;
  parent.z = 0.0;
  child.x = -transform.getOrigin().getX();
  child.y = -transform.getOrigin().getY();
  child.z = -transform.getOrigin().getZ();

  marker.points.push_back(parent);
  marker.points.push_back(child);

//  marker.pose.position.x = -transform.getOrigin().getX();
//  marker.pose.position.y = -transform.getOrigin().getY();
//  marker.pose.position.z = -transform.getOrigin().getZ();
//  marker.pose.orientation.x = transform.getRotation().getX();
//  marker.pose.orientation.y = transform.getRotation().getY();
//  marker.pose.orientation.z = transform.getRotation().getZ();
//  marker.pose.orientation.w = transform.getRotation().getW();
  marker.scale.x = 0.03;
  marker.scale.y = 0.06;
  marker.scale.z = 1.0;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  //only if using a MESH_RESOURCE marker type:
  marker.mesh_resource = "";
  vis_pub.publish(marker);

}

 /**
 * This programm publishes a tf for every the last received pose
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "com_viz");
  ros::NodeHandle n("~");

  // ROS STUFF
  n.param<std::string>("rsb_viz_scope", rsbScope, "/amiro1tobi");
  n.param<std::string>("parent_frame", parentFrame, "world");
  n.param<std::string>("child_frame", childFrame, "foo");
  listener = new tf::TransformListener;
  // RSB STUFF
  // Get the RSB factory
  rsb::Factory& factory = rsb::getFactory();

  // Prepare RSB listener for scopes
  rsb::ListenerPtr listener = factory.createListener(rsbScope);
  listener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&vizScope)));

  // ROS publisher for the marker
  vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  ros::spin();
  return 0;
}
