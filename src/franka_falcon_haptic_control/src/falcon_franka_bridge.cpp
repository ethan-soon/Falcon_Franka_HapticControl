#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <algorithm>
#include <cmath>
#include <mutex>

// Subscribes to:
//   /falcon/position   (geometry_msgs/PointStamped)  - Falcon end-effector position
//   /falcon/button_raw (std_msgs/Int32)              - button bitmask (1=plus, 2=forward, 4=center, 8=minus)
//   /barrier_contact   (ros_gz_interfaces/Contacts)  - Gz physics contact sensor on the barrier
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
static constexpr double GRIPPER_COOLDOWN_SEC = 2.5;

// Haptic feedback via Gz contact sensor.
// Force = HAPTIC_STIFFNESS * contact_depth, in the contact normal direction,
// summed over all contact points. Clamped to HAPTIC_MAX_FORCE per axis.
// If no contact message arrives within CONTACT_TIMEOUT_SEC, force is zeroed.
static constexpr double HAPTIC_STIFFNESS    = 50000.0; // N/m — ODE contact depths are sub-mm
static constexpr double HAPTIC_MAX_FORCE    =    3.0;  // N per axis (Falcon max ~8.9 N total)
static constexpr double CONTACT_TIMEOUT_SEC =    0.05; // zero force if contact goes stale

// Cube geometry — must match spawn position/size in the launch file
static constexpr double CUBE_CX   = 0.6;   // centre x (Franka world frame)
static constexpr double CUBE_CY   = 0.0;   // centre y
static constexpr double CUBE_CZ   = 0.4;   // centre z
static constexpr double CUBE_H    = 0.15;  // half-side (cube is 0.3 m)
// A contact point is "on" a face when its offset from centre is within FACE_TOL
// of the half-size. This lets edge/corner contacts trigger forces on multiple axes.
static constexpr double FACE_TOL  = 0.02;  // m

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
    declare_parameter("scale_y", 15.0);
    declare_parameter("scale_z", 10.0);
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

    contact_sub_ = create_subscription<ros_gz_interfaces::msg::Contacts>(
      "barrier_contact", 10,
      std::bind(&FalconFrankaBridge::contact_cb, this, std::placeholders::_1));

    // Publishers ------------------------------------------------------------------------------------------
    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
    force_pub_  = create_publisher<geometry_msgs::msg::Vector3Stamped>("falcon/force", 10);
    gripper_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "gripper_controller/joint_trajectory", 10);

    startup_timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&FalconFrankaBridge::open_gripper_on_startup, this));

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

  void contact_cb(ros_gz_interfaces::msg::Contacts::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(contact_mutex_);
    latest_contacts_ = *msg;
    last_contact_time_ = get_clock()->now().seconds();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "contact_cb: %zu contacts in message", msg->contacts.size());
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
    msg.header.stamp = rclcpp::Time(0);
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

    // Haptic feedback from Gz contact sensor -------------------------------------------
    // The contact sensor on the barrier publishes contact normals and penetration depths
    // straight from the physics engine. Sum depth * normal over all contact points to
    // get a net force vector in Franka world frame, then map to Falcon force frame.
    geometry_msgs::msg::Vector3Stamped force_msg;
    force_msg.header.stamp    = get_clock()->now();
    force_msg.header.frame_id = "falcon";

    double now = get_clock()->now().seconds();
    bool contact_fresh;
    ros_gz_interfaces::msg::Contacts contacts_copy;
    {
      std::lock_guard<std::mutex> lock(contact_mutex_);
      contact_fresh = (now - last_contact_time_) < CONTACT_TIMEOUT_SEC;
      if (contact_fresh) {
        contacts_copy = latest_contacts_;
      }
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "timer: fresh=%d contacts=%zu", contact_fresh, contacts_copy.contacts.size());

    if (contact_fresh && !contacts_copy.contacts.empty()) {
      // Accumulate average contact normal across all contact points.
      // We deliberately avoid relying on contact.depths — Gz Sim's ODE bridge
      // does not reliably populate that field. Instead we use the average normal
      // direction and scale to HAPTIC_MAX_FORCE, which is the right behaviour for
      // a rigid wall anyway.
      double nx_sum = 0.0, ny_sum = 0.0, nz_sum = 0.0;
      int count = 0;
      for (const auto & contact : contacts_copy.contacts) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "contact: positions:%zu normals:%zu depths:%zu",
          contact.positions.size(), contact.normals.size(), contact.depths.size());
        for (const auto & n : contact.normals) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "  normal: (%.3f, %.3f, %.3f)", n.x, n.y, n.z);
          nx_sum += n.x;
          ny_sum += n.y;
          nz_sum += n.z;
          ++count;
        }
      }

      // Map Franka world normal → Falcon force frame, scale to HAPTIC_MAX_FORCE:
      //   franka_x ↔ falcon_z (negated)  →  F_falcon_z = +nx * MAX
      //   franka_y ↔ falcon_x (negated)  →  F_falcon_x = +ny * MAX
      //   franka_z ↔ falcon_y            →  F_falcon_y = -nz * MAX
      auto clamp = [](double v, double limit) {
        return std::max(-limit, std::min(limit, v));
      };

      if (count > 0) {
        double nx = nx_sum / count;
        double ny = ny_sum / count;
        double nz = nz_sum / count;

        double mag = std::sqrt(nx*nx + ny*ny + nz*nz);
        if (mag > 1e-6) { nx /= mag; ny /= mag; nz /= mag; }

        force_msg.vector.x = clamp( ny * HAPTIC_MAX_FORCE, HAPTIC_MAX_FORCE);
        force_msg.vector.y = clamp(-nz * HAPTIC_MAX_FORCE, HAPTIC_MAX_FORCE);
        force_msg.vector.z = clamp( nx * HAPTIC_MAX_FORCE, HAPTIC_MAX_FORCE);
      } else {
        // Normals not populated — use contact positions to determine which cube face was hit.
        // The contact point lies on the face surface, so its offset from the cube centre
        // tells us which face: the face whose outward normal points from the cube centre
        // toward the contact point.
        //   normal ≈ -sign(pos - centre) per axis (outward = away from centre toward robot)
        // Then map to Falcon frame:
        //   franka_x face → F_falcon_z = +normal_x * MAX
        //   franka_y face → F_falcon_x = +normal_y * MAX   (signs match working x-axis case)
        RCLCPP_WARN_ONCE(get_logger(), "No normals — using contact positions for direction");

        // Per-axis accumulators and counts kept separate so each axis is
        // normalised independently — a flat-face contact won't dilute other axes.
        double fx_sum = 0.0, fy_sum = 0.0, fz_sum = 0.0;
        int fx_n = 0, fy_n = 0, fz_n = 0;

        for (const auto & contact : contacts_copy.contacts) {
          for (const auto & pos : contact.positions) {
            double px = pos.x - CUBE_CX;
            double py = pos.y - CUBE_CY;
            double pz = pos.z - CUBE_CZ;

            // Apply force on an axis when the contact point is within FACE_TOL of
            // that face — allows edge/corner contacts to produce simultaneous
            // forces on multiple axes.
            // Signs:  franka_x ↔ -falcon_z,  franka_y ↔ -falcon_x,  franka_z ↔ +falcon_y
            if (std::abs(px) >= CUBE_H - FACE_TOL) {
              fx_sum += (px < 0) ? 1.0 : -1.0;
              ++fx_n;
            }
            if (std::abs(py) >= CUBE_H - FACE_TOL) {
              fy_sum += (py < 0) ? 1.0 : -1.0;
              ++fy_n;
            }
            if (std::abs(pz) >= CUBE_H - FACE_TOL) {
              fz_sum += (pz > 0) ? 1.0 : -1.0;
              ++fz_n;
            }
          }
        }

        auto clamp = [](double v, double limit) {
          return std::max(-limit, std::min(limit, v));
        };
        if (fx_n > 0)
          force_msg.vector.z = clamp((fx_sum / fx_n) * HAPTIC_MAX_FORCE, HAPTIC_MAX_FORCE);
        if (fy_n > 0)
          force_msg.vector.x = clamp((fy_sum / fy_n) * HAPTIC_MAX_FORCE, HAPTIC_MAX_FORCE);
        if (fz_n > 0)
          force_msg.vector.y = clamp((fz_sum / fz_n) * HAPTIC_MAX_FORCE, HAPTIC_MAX_FORCE);
      }
    }

    force_pub_->publish(force_msg);
  }

  // Handles -------------------------------------------------------------------
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr             btn_sub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contact_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr     target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr  force_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;

  // States --------------------------------------------------------------------
  geometry_msgs::msg::PointStamped    latest_falcon_pos_;
  ros_gz_interfaces::msg::Contacts    latest_contacts_;
  std::mutex                          contact_mutex_;
  double                              last_contact_time_{0.0};
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
