#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

// Subscribes to:
//   /falcon/position  (geometry_msgs/PointStamped)  - Falcon end-effector position
//   /falcon/button_raw (std_msgs/Int32)             - button bitmask (1=plus, 2=forward, 4=center, 8=minus)
//
// Publishes to:
//   /falcon/force     (geometry_msgs/Vector3Stamped)
//   /target_pose      (geometry_msgs/PoseStamped)
//   /gripper_controller/joint_trajectory (trajectory_msgs/JointTrajectory)
//
// Falcon workspace (metres):
//   x: -0.062 to  0.062
//   y: -0.058 to  0.058
//   z:  0.075 to  0.175
//
// Gripper control (Gazebo simulation):
//   Center button (bit 2, mask 4) toggles gripper open/closed on each press.
//   Publishes directly to gripper_controller/joint_trajectory topic.
//   Open:  finger_joint1 = 0.04 m  (max, 4 cm per finger)
//   Close: finger_joint1 = 0.00 m  (fully closed)

static constexpr int    CENTER_BTN_MASK  = 4;
static constexpr double FINGER_OPEN_POS  = 0.04;
static constexpr double FINGER_CLOSE_POS = 0.00;
static constexpr double GRIPPER_MOVE_SEC = 2.0;
// Minimum seconds between gripper commands (debounce)
static constexpr double GRIPPER_COOLDOWN_SEC = 2.5;

class FalconFrankaBridge : public rclcpp::Node
{
public:
  FalconFrankaBridge() : Node("falcon_franka_bridge")
  {
    // Parameters ------------------------------------------------------------------------------------------

    declare_parameter("franka_center_x", 0.5);
    declare_parameter("franka_center_y", 0.0);
    declare_parameter("franka_center_z", 0.4);
    declare_parameter("falcon_rest_z", 0.125);
    declare_parameter("scale_x", 8.0);
    declare_parameter("scale_y", 8.0);
    declare_parameter("scale_z", 8.0);
    declare_parameter("loop_rate_hz", 100.0);
    declare_parameter("position_deadband", 0.001);
    declare_parameter("ee_orientation_w", 1.0);
    declare_parameter("ee_orientation_x", 0.0);
    declare_parameter("ee_orientation_y", 0.0);
    declare_parameter("ee_orientation_z", 0.0);

    franka_center_x_ = get_parameter("franka_center_x").as_double();
    franka_center_y_ = get_parameter("franka_center_y").as_double();
    franka_center_z_ = get_parameter("franka_center_z").as_double();
    scale_x_ = get_parameter("scale_x").as_double();
    scale_y_ = get_parameter("scale_y").as_double();
    scale_z_ = get_parameter("scale_z").as_double();
    loop_rate_hz_ = get_parameter("loop_rate_hz").as_double();
    position_deadband_ = get_parameter("position_deadband").as_double();
    falcon_rest_z_ = get_parameter("falcon_rest_z").as_double();
    ee_orientation_w_ = get_parameter("ee_orientation_w").as_double();
    ee_orientation_x_ = get_parameter("ee_orientation_x").as_double();
    ee_orientation_y_ = get_parameter("ee_orientation_y").as_double();
    ee_orientation_z_ = get_parameter("ee_orientation_z").as_double();

    // Subscribers -----------------------------------------------------------------------------------------
    pos_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "falcon/position", 10,
      std::bind(&FalconFrankaBridge::falcon_pos_cb, this, std::placeholders::_1));

    btn_sub_ = create_subscription<std_msgs::msg::Int32>(
      "falcon/button_raw", 10,
      std::bind(&FalconFrankaBridge::falcon_btn_cb, this, std::placeholders::_1));

    // Publishers ------------------------------------------------------------------------------------------
    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
    force_pub_  = create_publisher<geometry_msgs::msg::Vector3Stamped>("falcon/force", 10);
    // Direct topic interface of JointTrajectoryController — no action overhead
    gripper_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "gripper_controller/joint_trajectory", 10);

    // One-shot timer: open gripper 5 s after startup (gives controllers time to activate)
    startup_timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&FalconFrankaBridge::open_gripper_on_startup, this));

    // Main control loop
    timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&FalconFrankaBridge::timer_callback, this));
  }

private:
  void open_gripper_on_startup() {
    startup_timer_->cancel();
    RCLCPP_INFO(get_logger(), "Startup: sending open command.");
    send_gripper_command(FINGER_OPEN_POS);
    gripper_open_ = true;
  }

  void falcon_pos_cb(geometry_msgs::msg::PointStamped::SharedPtr msg) {
    latest_falcon_pos_ = *msg;
  }

  void falcon_btn_cb(std_msgs::msg::Int32::SharedPtr msg) {
    bool center_pressed = (msg->data & CENTER_BTN_MASK) != 0;

    if (center_pressed && !prev_center_btn_) {
      // Debounce: ignore presses within cooldown window of the last command
      double now = get_clock()->now().seconds();
      if ((now - last_gripper_cmd_time_) < GRIPPER_COOLDOWN_SEC) {
        RCLCPP_INFO(get_logger(), "Center button pressed — cooldown active, ignoring.");
      } else {
        RCLCPP_INFO(get_logger(), "Center button pressed — toggling gripper.");
        toggle_gripper();
      }
    }

    prev_center_btn_ = center_pressed;
  }

  void toggle_gripper() {
    if (gripper_open_) {
      send_gripper_command(FINGER_CLOSE_POS);
      gripper_open_ = false;
      RCLCPP_INFO(get_logger(), "Gripper closing.");
    } else {
      send_gripper_command(FINGER_OPEN_POS);
      gripper_open_ = true;
      RCLCPP_INFO(get_logger(), "Gripper opening.");
    }
  }

  void send_gripper_command(double target_position) {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = rclcpp::Time(0);  // zero = execute immediately
    msg.joint_names = {"fr3_finger_joint1"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {target_position};
    point.time_from_start = rclcpp::Duration::from_seconds(GRIPPER_MOVE_SEC);
    msg.points = {point};

    gripper_pub_->publish(msg);
    last_gripper_cmd_time_ = get_clock()->now().seconds();
  }

  void timer_callback() {
    double falcon_x = latest_falcon_pos_.point.x;
    double falcon_y = latest_falcon_pos_.point.y;
    double falcon_z = latest_falcon_pos_.point.z - falcon_rest_z_;

    double franka_x = franka_center_x_ - (falcon_z * scale_z_);
    double franka_y = franka_center_y_ - (falcon_x * scale_x_);
    double franka_z = franka_center_z_ + (falcon_y * scale_y_);

    geometry_msgs::msg::PoseStamped target;
    target.header.stamp    = get_clock()->now();
    target.header.frame_id = "fr3_link0";
    target.pose.position.x = franka_x;
    target.pose.position.y = franka_y;
    target.pose.position.z = franka_z;
    target.pose.orientation.w = ee_orientation_w_;
    target.pose.orientation.x = ee_orientation_x_;
    target.pose.orientation.y = ee_orientation_y_;
    target.pose.orientation.z = ee_orientation_z_;

    target_pub_->publish(target);
  }

  // Handles -------------------------------------------------------------------
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr             btn_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr     target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr  force_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;

  // States --------------------------------------------------------------------
  geometry_msgs::msg::PointStamped latest_falcon_pos_;
  bool   prev_center_btn_{false};
  bool   gripper_open_{true};
  double last_gripper_cmd_time_{0.0};

  // Parameters ----------------------------------------------------------------
  double franka_center_x_, franka_center_y_, franka_center_z_;
  double scale_x_, scale_y_, scale_z_;
  double loop_rate_hz_;
  double position_deadband_;
  double falcon_rest_z_;
  double ee_orientation_w_, ee_orientation_x_, ee_orientation_y_, ee_orientation_z_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FalconFrankaBridge>());
  rclcpp::shutdown();
  return 0;
}
