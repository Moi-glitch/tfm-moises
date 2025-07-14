#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "moi_exp_lite/bt_behaviors.hpp"

using namespace std::chrono_literals;

class BTNode : public rclcpp::Node
{
public:
  BTNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("bt_node", options)
  {
    std::string default_bt =
      ament_index_cpp::get_package_share_directory("moi_exp_lite") + "/bt_xml/bt.xml";
    this->declare_parameter<std::string>("bt_xml", default_bt);
    if (!this->has_parameter("use_sim_time")) {
      this->declare_parameter<bool>("use_sim_time", false);
    }
    this->declare_parameter<bool>("enable_groot_monitoring", true);

    this->get_parameter("bt_xml", bt_xml_);
    this->get_parameter("enable_groot_monitoring", enable_groot_);
    this->get_parameter("use_sim_time", use_sim_time_);
    setenv("USE_SIM_TIME", use_sim_time_ ? "true" : "false", 1);

    moi_exp_lite::register_bt_nodes(factory_);
    tree_ = factory_.createTreeFromFile(bt_xml_);

    if (enable_groot_) {
      publisher_ = std::make_shared<BT::PublisherZMQ>(tree_);
    }

    timer_ = this->create_wall_timer(100ms, [this]() {
      auto status = tree_.tickRoot();
      if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
        RCLCPP_INFO(this->get_logger(),
                    "Behavior tree finished");
        timer_->cancel();
      }
    });
  }

  void init_blackboard()
  {
    tree_.rootBlackboard()->set("node", shared_from_this());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    this->declare_parameter<std::string>("robot_base_frame", "base_link");
    this->get_parameter("robot_base_frame", robot_base_frame_);

    geometry_msgs::msg::Pose pose;
    for (int i = 0; i < 50; ++i) {
      if (tf_buffer_->canTransform("map", robot_base_frame_, tf2::TimePointZero,
                                   tf2::durationFromSec(0.1))) {
        auto tf = tf_buffer_->lookupTransform("map", robot_base_frame_,
                                             tf2::TimePointZero);
        pose.position.x = tf.transform.translation.x;
        pose.position.y = tf.transform.translation.y;
        pose.orientation = tf.transform.rotation;
        tree_.rootBlackboard()->set("initial_pose", pose);
        break;
      }
      rclcpp::sleep_for(100ms);
    }
  }

private:
  std::string bt_xml_;
  bool enable_groot_;
  bool use_sim_time_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::shared_ptr<BT::PublisherZMQ> publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string robot_base_frame_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BTNode>();
  node->init_blackboard();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
