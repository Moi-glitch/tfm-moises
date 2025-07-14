#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <future>
#include <cstdlib>
#include "moi_exp_lite/bt_behaviors.hpp"

using namespace BT;


// Decorator that repeats its child until SUCCESS is returned
class RepeatUntilSuccess : public DecoratorNode
{
public:
  RepeatUntilSuccess(const std::string& name, const BT::NodeConfiguration& conf)
    : DecoratorNode(name, conf) {}

  static BT::PortsList providedPorts() { return {}; }

  NodeStatus tick() override
  {
    const NodeStatus child_status = child_node_->executeTick();

    switch (child_status)
    {
      case NodeStatus::SUCCESS:
        child_node_->halt();
        return NodeStatus::SUCCESS;

      case NodeStatus::FAILURE:
        child_node_->halt();
        return NodeStatus::RUNNING;

      case NodeStatus::RUNNING:
        return NodeStatus::RUNNING;

      default:
        return NodeStatus::FAILURE;
    }
  }
};

// Action to initialize camera calibrations
class StartDetectObject : public SyncActionNode {
public:
  StartDetectObject(const std::string& name, const BT::NodeConfiguration& conf)
      : SyncActionNode(name, conf) {}

  static BT::PortsList providedPorts() { return {}; }

  NodeStatus tick() override {
    RCLCPP_INFO(rclcpp::get_logger("bt_behaviors"),
                "Starting camera calibration nodes");
    const char* sim = std::getenv("USE_SIM_TIME");
    std::string use_sim = sim ? sim : "false";
    std::string cmd1 =
      "ros2 launch turtlebot3_autorace_camera intrinsic_camera_calibration.launch.py use_sim_time:=" + use_sim + " &";
    std::string cmd2 =
      "ros2 launch turtlebot3_autorace_camera extrinsic_camera_calibration.launch.py use_sim_time:=" + use_sim + " &";
    std::system(cmd1.c_str());
    std::system(cmd2.c_str());
    return NodeStatus::SUCCESS;
  }
};

// Action to start explore_controller and color detector
class StartExploreController : public SyncActionNode {
public:
  StartExploreController(const std::string& name, const BT::NodeConfiguration& conf)
      : SyncActionNode(name, conf) {}

  static BT::PortsList providedPorts() { return {}; }

  NodeStatus tick() override {
    RCLCPP_INFO(rclcpp::get_logger("bt_behaviors"),
                "Starting explore_controller and color detection nodes");

    const char* sim = std::getenv("USE_SIM_TIME");
    std::string use_sim = sim ? sim : "false";

    std::system(
      "ros2 run image_transport republish raw compressed --ros-args -r in:=/camera/image_raw -r out/compressed:=/camera/image_raw/compressed &");
    std::string detect_cmd =
      "ros2 launch moi_exp_lite detect_object.launch.py calibration_mode:=True use_sim_time:=" + use_sim + " &";
    std::system(detect_cmd.c_str());

    std::string params =
      ament_index_cpp::get_package_share_directory("moi_exp_lite") +
      "/param/explore_controller_params.yaml";
    std::string cmd =
      std::string("ros2 run moi_exp_lite explore_controller --ros-args --params-file ") +
      params + " --ros-args -p use_sim_time:=" + use_sim + " &";
    std::system(cmd.c_str());

    return NodeStatus::SUCCESS;
  }
};

// Action to launch explorer node
class StartExplorer : public SyncActionNode {
public:
  StartExplorer(const std::string& name, const BT::NodeConfiguration& conf)
      : SyncActionNode(name, conf) {}

  static BT::PortsList providedPorts() { return {}; }
  NodeStatus tick() override {
    RCLCPP_INFO(rclcpp::get_logger("bt_behaviors"), "Starting explorer node");

    const char* sim = std::getenv("USE_SIM_TIME");
    std::string use_sim = sim ? sim : "false";

    std::string pkg_share =
      ament_index_cpp::get_package_share_directory("moi_exp_lite");
    std::string params = pkg_share + "/param/explorer_params.yaml";
    std::string cmd = std::string(
      "ros2 launch moi_exp_lite explore.launch.py params_file:=") + params +
      " use_sim_time:=" + use_sim + " &";
    std::system(cmd.c_str());

    return NodeStatus::SUCCESS;
  }
};

// Condition node: check if three red detections exist
class HaveThreeDetections : public ConditionNode {
public:
  HaveThreeDetections(const std::string& name, const BT::NodeConfiguration& conf)
      : ConditionNode(name, conf), remaining_(3)
  {
    node_ = rclcpp::Node::make_shared("have_three_detections_bt");
    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/object_marked", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr) {
        if (remaining_ > 0) {
          --remaining_;
        }
      });
  }

  static BT::PortsList providedPorts() { return {}; }

  NodeStatus tick() override {
    rclcpp::spin_some(node_);
    if (remaining_ <= 0) {
      RCLCPP_INFO(node_->get_logger(), "Received three red object marks");
      return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
  }

private:
  int remaining_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
};

// Action to stop explorer
class StopExplorer : public SyncActionNode {
public:
  StopExplorer(const std::string& name, const BT::NodeConfiguration& conf)
      : SyncActionNode(name, conf) {}

  static BT::PortsList providedPorts() { return {}; }
  NodeStatus tick() override {
    RCLCPP_INFO(rclcpp::get_logger("bt_behaviors"), "Stopping explorer node");
    // Send SIGINT specifically to the explorer node process so that
    // other nodes like explore_controller keep running
    std::system("pkill -2 -f explore_node &");
    return NodeStatus::SUCCESS;
  }
};

// Action to command nav2 to return to initial pose
class ReturnToInitialPose : public StatefulActionNode {
public:
  ReturnToInitialPose(const std::string& name, const BT::NodeConfiguration& conf)
      : StatefulActionNode(name, conf) {}

  static BT::PortsList providedPorts() { return {}; }

  NodeStatus onStart() override {
    if (!config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_)) {
      RCLCPP_ERROR(rclcpp::get_logger("bt_behaviors"),
                   "Node pointer not found in blackboard");
      return NodeStatus::FAILURE;
    }

    if (!config().blackboard->get("initial_pose", initial_pose_)) {
      RCLCPP_ERROR(rclcpp::get_logger("bt_behaviors"),
                   "Initial pose not available on blackboard");
      return NodeStatus::FAILURE;
    }

    if (!nav2_client_) {
      nav2_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          node_, "navigate_to_pose");
    }

    RCLCPP_INFO(node_->get_logger(),
                "Returning to initial pose with nav2");

    goal_sent_ = false;
    return NodeStatus::RUNNING;
  }

  NodeStatus onRunning() override {
    if (!goal_sent_) {
      if (!nav2_client_->wait_for_action_server(std::chrono::seconds(0))) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Waiting for NavigateToPose action server");
        return NodeStatus::RUNNING;
      }

      nav2_msgs::action::NavigateToPose::Goal goal;
      goal.pose.header.frame_id = "map";
      goal.pose.header.stamp = node_->now();
      goal.pose.pose = initial_pose_;

      goal_handle_future_ = nav2_client_->async_send_goal(goal);
      goal_sent_ = true;
      return NodeStatus::RUNNING;
    }

    if (goal_handle_future_.valid() &&
        goal_handle_future_.wait_for(std::chrono::seconds(0)) ==
          std::future_status::ready &&
        !goal_handle_) {
      goal_handle_ = goal_handle_future_.get();
      if (!goal_handle_) {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected");
        return NodeStatus::FAILURE;
      }
      result_future_ = nav2_client_->async_get_result(goal_handle_);
    }

    if (result_future_.valid() &&
        result_future_.wait_for(std::chrono::seconds(0)) ==
          std::future_status::ready) {
      auto result = result_future_.get();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        return NodeStatus::SUCCESS;
      }

      RCLCPP_ERROR(
        node_->get_logger(),
        "Navigation to initial pose failed: action result %d",
        static_cast<int>(result.code));
      return NodeStatus::FAILURE;
    }

    return NodeStatus::RUNNING;
  }

  void onHalted() override {
    if (nav2_client_ && goal_handle_) {
      nav2_client_->async_cancel_goal(goal_handle_);
    }
    goal_sent_ = false;
  }

private:
  rclcpp::Node::SharedPtr node_;
  geometry_msgs::msg::Pose initial_pose_{};
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  std::shared_future<typename GoalHandleNav::SharedPtr> goal_handle_future_;
  std::shared_ptr<GoalHandleNav> goal_handle_;
  std::shared_future<typename GoalHandleNav::WrappedResult> result_future_;
  bool goal_sent_{false};
};

namespace moi_exp_lite
{

void register_bt_nodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<StartDetectObject>("StartDetectObject");
  factory.registerNodeType<StartExploreController>("StartExploreController");
  factory.registerNodeType<StartExplorer>("StartExplorer");
  factory.registerNodeType<HaveThreeDetections>("HaveThreeDetections");
  factory.registerNodeType<RepeatUntilSuccess>("RepeatUntilSuccess");
  factory.registerNodeType<StopExplorer>("StopExplorer");
  factory.registerNodeType<ReturnToInitialPose>("ReturnToInitialPose");
}

}  // namespace moi_exp_lite

