#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

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

tf::Transform getTfFromPose(rst::geometry::Pose &rsbPose){
  tf::Transform transform;
  tf::Quaternion q;
  tf::Vector3 translation;
  translation.setZero();
  translation.setX(rsbPose.translation().x());
  translation.setY(rsbPose.translation().y());
  q.setX(rsbPose.rotation().qx());
  q.setY(rsbPose.rotation().qy());
  q.setZ(rsbPose.rotation().qz());
  q.setW(rsbPose.rotation().qw());
  transform.setOrigin(translation);
  transform.setRotation(q);
  return transform;
}

 /**
 * This programm publishes a tf for every the last received pose
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_to_tf_bridge");
  ros::NodeHandle n("~");

  // ROS STUFF
  std::string parentFrame;
  std::string childFrame;
  std::string rsbScope;
  int rateArg;

  n.param<std::string>("frame_id", parentFrame, "world");
  n.param<std::string>("child_frame", childFrame, "foo");
  n.param<std::string>("rsb_scope", rsbScope, "/pose");
  n.param<int>("rate", rateArg, 1);

  tf::TransformBroadcaster br;
  ros::Rate rate(rateArg);

  // RSB STUFF
  // Get the RSB factory
  rsb::Factory& factory = rsb::getFactory();

  // Register converters
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Pose > > odomConverter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose >());
  rsb::converter::converterRepository<std::string>()->registerConverter(odomConverter);

  // Prepare RSB listener for incomming path request
  rsb::ListenerPtr poseListener = factory.createListener(rsbScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::geometry::Pose>>>poseQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::geometry::Pose>>(1));
  poseListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::geometry::Pose>(poseQueue)));

  rst::geometry::Pose rsbPose;
  rsbPose.mutable_translation()->set_x(0.0);
  rsbPose.mutable_translation()->set_y(0.0);
  rsbPose.mutable_translation()->set_z(0.0);
  rsbPose.mutable_rotation()->set_qx(0.0);
  rsbPose.mutable_rotation()->set_qy(0.0);
  rsbPose.mutable_rotation()->set_qz(0.0);
  rsbPose.mutable_rotation()->set_qw(1.0);

  // PROCESS THE POSE
  while (ros::ok()) {

    if (!poseQueue->empty()) {
      rsbPose = *poseQueue->pop();
    }

    br.sendTransform(tf::StampedTransform( getTfFromPose(rsbPose), ros::Time::now(), parentFrame, childFrame));
    rate.sleep();
  }

  return 0;
}
