#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/int32.hpp>

// TODO: implement FalconFrankaBridge
//
// Subscribes to:
//   /falcon/position  (geometry_msgs/PointStamped)  - Falcon end-effector position
//   /falcon/button_raw (std_msgs/Int32)             - button bitmask (1=right,2=up,4=middle,8=left)
//
// Publishes to:
//   /falcon/force     (geometry_msgs/Vector3Stamped) - haptic force feedback to Falcon
//   /target_pose      (geometry_msgs/PoseStamped)    - scaled Cartesian goal for Franka
//
// Falcon workspace (metres):
//   x: -0.062 to  0.062
//   y: -0.058 to  0.058
//   z:  0.075 to  0.175
//
// You will need to scale these into the Franka reachable workspace
// and choose a fixed orientation (or add orientation control later).

class FalconFrankaBridge : public rclcpp::Node
{
public:
  FalconFrankaBridge() : Node("falcon_franka_bridge")
  {
    // Parameters ------------------------------------------------------------------------------------------

    // Franka center point, when Novint is at rest position (0.0, 0.0, 1.0)
    declare_parameter("franka_center_x", 0.5);
    declare_parameter("franka_center_y", 0.5);
    declare_parameter("franka_center_z", 0.5);

    // Gains from Falcon to Franka space
    // Falcon range is +/- 10 cm from rest position
    declare_parameter("scale_x", 8.0);
    declare_parameter("scale_y", 8.0);
    declare_parameter("scale_z", 8.0);

    // Timer rate
    declare_parameter("loop_rate_hz", 100.0);

    // Deadband threshold
    declare_parameter("position_deadband", 0.001);

    // Fixed Orientation
    // Quaternion (points the grippper straight down)
    declare_parameter("ee_orientation_w", 1.0);
    declare_parameter("ee_orientation_x", 0.0);
    declare_parameter("ee_orientation_y", 0.0);
    declare_parameter("ee_orientation_z", 0.0);

    // Read parameters into member variables
    franka_center_x_ = get_parameter("franka_center_x").as_double();
    franka_center_y_ = get_parameter("franka_center_y").as_double();
    franka_center_z_ = get_parameter("franka_center_z").as_double();
    scale_x_ = get_parameter("scale_x").as_double();
    scale_y_ = get_parameter("scale_y").as_double();
    scale_z_ = get_parameter("scale_z").as_double();
    loop_rate_hz_ = get_parameter("loop_rate_hz").as_double();
    position_deadband_ = get_parameter("position_deadband").as_double();
    ee_orientation_w_ = get_parameter("ee_orientation_w").as_double();
    ee_orientation_x_ = get_parameter("ee_orientation_x").as_double();
    ee_orientation_y_ = get_parameter("ee_orientation_y").as_double();
    ee_orientation_z_ = get_parameter("ee_orientation_z").as_double();

    // Subscribers ---------------------------------------------------------------------------------------
    pos_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "falcon/position", 10, std::bind(&FalconFrankaBridge::falcon_pos_cb, this, std::placeholders::_1)
    );

    btn_sub_ = create_subscription<std_msgs::msg::Int32>(
      "falcon/button_raw", 10, std::bind(&FalconFrankaBridge::falcon_btn_cb, this, std::placeholders::_1)
    );

    // Publishers ------------------------------------------------------------------------------------------
    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
    force_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("falcon/force", 10);

    // Timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(10), // 100Hz
      std::bind(&FalconFrankaBridge::timer_callback, this)
    );
  }

private:
  void falcon_pos_cb(geometry_msgs::msg::PointStamped::SharedPtr msg){
    latest_falcon_pos_ = *msg;
  }

  void falcon_btn_cb(std_msgs::msg::Int32::SharedPtr msg){
    // use the middle button to actuate the gripper
    gripper_state_ = msg->data & 4; // state = 0 -> open, state = 1 -> closed
  }

  void timer_callback(){
    // Falcon resting position is (0, 0, 0.125)
    double falcon_x = latest_falcon_pos_.point.x;
    double falcon_y = latest_falcon_pos_.point.y;
    double falcon_z = latest_falcon_pos_.point.z - 0.125;

    // scale to franka workspace
    double franka_x = franka_center_x_ + (falcon_x * scale_x_);
    double franka_y = franka_center_y_ + (falcon_y * scale_y_);
    double franka_z = franka_center_z_ + (falcon_z * scale_z_);

    // build pose msg
    geometry_msgs::msg::PoseStamped target;

    target.header.stamp = get_clock()->now();
    target.header.frame_id = "fr3_link0"; // Franka base frame

    target.pose.position.x = franka_x;                    
    target.pose.position.y = franka_y;                                        
    target.pose.position.z = franka_z;                    

    target.pose.orientation.w = ee_orientation_w_;                            
    target.pose.orientation.x = ee_orientation_x_;
    target.pose.orientation.y = ee_orientation_y_;                            
    target.pose.orientation.z = ee_orientation_z_;

    // publish pose msg
    target_pub_->publish(target);
  }

  // Handles ------------------------------------------------------------------------                                                                                
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr btn_sub_;   
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr force_pub_; 
  rclcpp::TimerBase::SharedPtr timer_;

  // States ---------------------------------------------------------------------------
  geometry_msgs::msg::PointStamped latest_falcon_pos_;
  int gripper_state_{0};

  // Parameters -------------------------------------------------------------------
  double franka_center_x_, franka_center_y_, franka_center_z_;
  double scale_x_, scale_y_, scale_z_;
  double loop_rate_hz_;
  double position_deadband_;
  double ee_orientation_w_, ee_orientation_x_, ee_orientation_y_, ee_orientation_z_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FalconFrankaBridge>());
  rclcpp::shutdown();
  return 0;
}
