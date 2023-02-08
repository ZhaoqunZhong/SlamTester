#include "ros/ros.h"
#include <thread>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Imu.h"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/Bool.h"
#include "glog/logging.h"
#include "../algorithms/vins_mono/VinsAlgorithm.h"
#include "../socket_publisher/publisher.h"

DEFINE_string(algoConfig, "", "Path to algorithm config file.");
DEFINE_string(camConfig, "", "Path to internal camera config file.");
DEFINE_string(groundTruth, "", "Path to ground truth file.");
DEFINE_bool(rs_cam, false, "Choose rolling shutter camera data if available.");
DEFINE_bool(resizeAndUndistort, false, "Resize the rgb resolution and undistort before feeding to algorithm.");
DEFINE_bool(showOrigCamStream, false, "");
DEFINE_string(socketConfig, "", "Path to socket publisher config file.");

std::unique_ptr<SlamTester::AlgorithmInterface> algorithm_inter;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
  cv_bridge::CvImageConstPtr ptr;
/*  if (img_msg->encoding == "8UC1")
  {
    sensor_msgs::Image img;
    img.header = img_msg->header;
    img.height = img_msg->height;
    img.width = img_msg->width;
    img.is_bigendian = img_msg->is_bigendian;
    img.step = img_msg->step;
    img.data = img_msg->data;
    img.encoding = "mono8";
    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  }
  else*/
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
  cv::Mat mat_out = ptr->image;
  // LOG(INFO) << "img timestamp: " << img_msg->header.stamp.toNSec();
  algorithm_inter->feedMonoImg(img_msg->header.stamp.toSec(), mat_out);
}
void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
  // LOG(INFO) << "imu timestamp: " << imu_msg->header.stamp.toNSec();
  algorithm_inter->feedImu(imu_msg->header.stamp.toSec(),
                 Eigen::Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,
                    imu_msg->linear_acceleration.z),
                 Eigen::Vector3d(imu_msg->angular_velocity.x,
                    imu_msg->angular_velocity.y, imu_msg->angular_velocity.z));
}

int main(int argc, char **argv)
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    FLAGS_logtostdout = true;
    FLAGS_colorlogtostdout = true;
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "ros_wrap_vins");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub_img = n.subscribe("/cam0/image_raw", 1, img_callback);
  ros::Subscriber sub_imu = n.subscribe("/imu0", 2000, imu_callback, ros::TransportHints().tcpNoDelay());

  /// Set up algorithm
  algorithm_inter = std::make_unique<VinsAlgorithm>(FLAGS_algoConfig);
  std::shared_ptr<socket_publisher::publisher> socket_viewer;
  YAML::Node socket_config = YAML::LoadFile(FLAGS_socketConfig);
  socket_viewer = std::make_shared<socket_publisher::publisher>(socket_config);
  algorithm_inter->output_interfaces.push_back(socket_viewer);

  algorithm_inter->start();

  std::thread socket_thread([&]{socket_viewer->run();});

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}