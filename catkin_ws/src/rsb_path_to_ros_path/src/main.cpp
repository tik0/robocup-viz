#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <rsb/Factory.h>
#include <rsb/Listener.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rst/navigation/Path.pb.h>

ros::Publisher publisher;
std::string frameId;

//nav_msgs::Path convertRSBPathToROSPath(rst::navigation::Path &rsbPath, std::string frameId) {
//    nav_msgs::Path rosPath;
//    ros::Time stamp = ros::Time::now();
//    rosPath.header.frame_id = frameId;
//    rosPath.header.stamp = stamp;

//    for (rst::geometry::Pose rsbPose : rsbPath.poses()) {
//        geometry_msgs::PoseStamped rosPose;
//        rosPose.header.frame_id = frameId;
//        rosPose.header.stamp = stamp;

//        rosPose.pose.position.x = rsbPose.translation().x();
//        rosPose.pose.position.y = rsbPose.translation().y();
//        rosPose.pose.position.z = rsbPose.translation().z();
//        rosPose.pose.orientation.x = rsbPose.rotation().qx();
//        rosPose.pose.orientation.y = rsbPose.rotation().qy();
//        rosPose.pose.orientation.z = rsbPose.rotation().qz();
//        rosPose.pose.orientation.w = rsbPose.rotation().qw();

//        rosPath.poses.push_back(rosPose);
//    }

//    return rosPath;
//}

void publishRSBPathAsROSPath(boost::shared_ptr<rst::navigation::Path> rsbPath)
{
    nav_msgs::Path rosPath;
    ros::Time stamp = ros::Time::now();
    rosPath.header.frame_id = frameId;
    rosPath.header.stamp = stamp;

    for (rst::geometry::Pose rsbPose : rsbPath->poses()) {
        geometry_msgs::PoseStamped rosPose;
        rosPose.header.frame_id = frameId;
        rosPose.header.stamp = stamp;

        rosPose.pose.position.x = rsbPose.translation().x();
        rosPose.pose.position.y = rsbPose.translation().y();
        rosPose.pose.position.z = rsbPose.translation().z();
        rosPose.pose.orientation.x = rsbPose.rotation().qx();
        rosPose.pose.orientation.y = rsbPose.rotation().qy();
        rosPose.pose.orientation.z = rsbPose.rotation().qz();
        rosPose.pose.orientation.w = rsbPose.rotation().qw();

        rosPath.poses.push_back(rosPose);
    }

    publisher.publish(rosPath);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rsb_path_to_ros_path");
    ros::NodeHandle n("~");

    std::string rosTopic;
    std::string rsbScope;
    n.param<std::string>("ros_topic", rosTopic, "/rsb_path");
    n.param<std::string>("rsb_scope", rsbScope, "/path");
    n.param<std::string>("frame_id", frameId, "world");

    publisher = n.advertise<nav_msgs::Path>(rosTopic, 5);

    rsb::Factory& factory = rsb::getFactory();
    // register converter for navigation path
    boost::shared_ptr< rsb::converter::ProtocolBufferConverter< rst::navigation::Path > > pathConverter(new rsb::converter::ProtocolBufferConverter< rst::navigation::Path >());
    rsb::converter::converterRepository<std::string>()->registerConverter(pathConverter);

    rsb::ListenerPtr listener = factory.createListener(rsbScope);
    listener->addHandler(rsb::HandlerPtr(new rsb::DataFunctionHandler<rst::navigation::Path>(&publishRSBPathAsROSPath)));

    ros::spin();

    return 0;
}
