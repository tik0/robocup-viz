#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseStamped.h>


#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST Proto types
#include <rst/geometry/Pose.pb.h>

using namespace rsb;
using namespace rsb::converter;

rsb::Informer< rst::geometry::Pose >::Ptr poseInformer;

void callback(const geometry_msgs::PoseStamped rosPose) {
  // Got ROS pose and convert to RSB pose
  boost::shared_ptr<rst::geometry::Pose> rsbPose(new rst::geometry::Pose);
  rsbPose->mutable_translation()->set_x(rosPose.pose.position.x);
  rsbPose->mutable_translation()->set_y(rosPose.pose.position.y);
  rsbPose->mutable_translation()->set_z(rosPose.pose.position.z);
  rsbPose->mutable_rotation()->set_qx(rosPose.pose.orientation.x);
  rsbPose->mutable_rotation()->set_qy(rosPose.pose.orientation.y);
  rsbPose->mutable_rotation()->set_qz(rosPose.pose.orientation.z);
  rsbPose->mutable_rotation()->set_qw(rosPose.pose.orientation.w);

  // Send RSB pose
  poseInformer->publish(rsbPose);
}

 /**
 * This programm publishes a tf for every the last received pose
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "navgoal_to_rsbpose_bridge");
  ros::NodeHandle n("~");

  // ROS STUFF
  std::string rsbScope;
  std::string rosTopic;

  n.param<std::string>("rsb_scope", rsbScope, "/rsbPose");
  n.param<std::string>("ros_topic", rosTopic, "/initialpose");

  // RSB STUFF
  // Get the RSB factory
  rsb::Factory& factory = rsb::getFactory();

  // Register converters
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Pose > > odomConverter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose >());
  rsb::converter::converterRepository<std::string>()->registerConverter(odomConverter);

  poseInformer = factory.createInformer<rst::geometry::Pose> (rsbScope);

  ros::Subscriber subIsm_data = n.subscribe<geometry_msgs::PoseStamped>(rosTopic, 2, callback);
  ros::spin();

  return 0;
}
