// falcon_node.cpp
// ROS 2 node for the Novint Falcon haptic device.
//
// 
//   - On startup : opens device, loads firmware (same retry logic as haptics.cpp)
//   - Each cycle : setForce → runIOLoop → getPosition, plus LED homing indicator
//   - On shutdown: zeroes force and closes device
//
// Topics
//   Published   : falcon/position  (geometry_msgs/PointStamped)   – 3-D end-effector position (metres)
//   Published   : falcon/button_raw (std_msgs/Int32)              – raw bitmask (1=plus,2=forward,4=center,8=minus)
//   Subscribed  : falcon/force     (geometry_msgs/Vector3Stamped)  – force command [N]
//
// Parameters (all have defaults, override with --ros-args -p name:=value)
//   firmware_path  (string) – absolute path to nvent_firmware.bin
//   device_index   (int)    – USB device index (0 = first Falcon found)
//   loop_rate_hz   (double) – control loop rate in Hz (default 1000)

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/int32.hpp>

#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/comm/FalconCommLibUSB.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/grip/FalconGripFourButton.h"  // ← added

#include <array>
#include <mutex>
#include <stdexcept>
#include <string>

class FalconNode : public rclcpp::Node
{
public:
    FalconNode() : Node("falcon_node"), force_{0.0, 0.0, 0.0}
    {
        declare_parameter("firmware_path",
            std::string("/home/ansoon/NovintFalcon/nvent_firmware.bin"));
        declare_parameter("device_index", 0);
        declare_parameter("loop_rate_hz", 1000.0);

        const std::string fw_path = get_parameter("firmware_path").as_string();
        const int         dev_idx = get_parameter("device_index").as_int();
        const double      rate_hz = get_parameter("loop_rate_hz").as_double();

        // ── Device initialisation ─────────────────────────────────────────────
        device_.setFalconComm<libnifalcon::FalconCommLibUSB>();
        device_.setFalconFirmware<libnifalcon::FalconFirmwareNovintSDK>();
        device_.setFalconKinematic<libnifalcon::FalconKinematicStamper>();
        device_.setFalconGrip<libnifalcon::FalconGripFourButton>();  // ← added

        if (!device_.open(static_cast<unsigned int>(dev_idx))) {
            throw std::runtime_error(
                "Cannot open Falcon (device " + std::to_string(dev_idx) +
                ") – is it plugged in?");
        }

        if (!device_.isFirmwareLoaded()) {
            if (!device_.setFirmwareFile(fw_path)) {
                throw std::runtime_error("Cannot find firmware file: " + fw_path);
            }
            if (!device_.loadFirmware(10u)) {
                throw std::runtime_error("Firmware upload failed after 10 retries");
            }
        }

        // Pump IO until firmware is confirmed ready (same 1000-attempt loop)
        int attempts = 0;
        while (!device_.isFirmwareLoaded() && attempts < 1000) {
            device_.runIOLoop(libnifalcon::FalconDevice::FALCON_LOOP_FIRMWARE);
            ++attempts;
        }
        if (!device_.isFirmwareLoaded()) {
            throw std::runtime_error("Falcon firmware not responding after upload");
        }

        RCLCPP_INFO(get_logger(),
            "Falcon ready on device %d @ %.0f Hz. "
            "Pull the grip out then push in to home (LED turns green).",
            dev_idx, rate_hz);
        RCLCPP_INFO(get_logger(),
            "Position will read (0,0,0) until homing is complete.");

        // ── Publishers ────────────────────────────────────────────────────────
        pos_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
            "falcon/position", 10);
        btn_raw_pub_ = create_publisher<std_msgs::msg::Int32>(
            "falcon/button_raw", 10);

        // ── Subscriber ────────────────────────────────────────────────────────
        force_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "falcon/force", 10,
            [this](geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(force_mutex_);
                force_ = {msg->vector.x, msg->vector.y, msg->vector.z};
            });

        // ── Control loop timer ────────────────────────────────────────────────
        const auto period = std::chrono::duration<double>(1.0 / rate_hz);
        timer_ = create_wall_timer(period, std::bind(&FalconNode::loop, this));
    }

    ~FalconNode()
    {
        // Zero force before closing
        device_.setForce({0.0, 0.0, 0.0});
        device_.runIOLoop(libnifalcon::FalconDevice::FALCON_LOOP_FIRMWARE |
                          libnifalcon::FalconDevice::FALCON_LOOP_KINEMATIC);
        device_.close();
        RCLCPP_INFO(get_logger(), "Falcon closed.");
    }

private:
    void loop()
    {
        // 1. Apply latest force command
        {
            std::lock_guard<std::mutex> lock(force_mutex_);
            device_.setForce(force_);
        }

        // 2. Run firmware + kinematic + grip IO.
        //    runIOLoop can return false when the USB buffer has no new data this
        //    tick – this is normal non-blocking behaviour.  We still publish the
        //    last-good cached position/button values rather than skipping, so
        //    downstream nodes always receive a value.
        //    FALCON_LOOP_GRIP must be included so runGripLoop() is called and
        //    button states are populated in m_digitalInputs.
        device_.runIOLoop(
            libnifalcon::FalconDevice::FALCON_LOOP_FIRMWARE |
            libnifalcon::FalconDevice::FALCON_LOOP_KINEMATIC |
            libnifalcon::FalconDevice::FALCON_LOOP_GRIP);

        // 3. Update LED: red = not homed, green = homed
        auto fw = device_.getFalconFirmware();
        const bool homed = fw->isHomed();
        fw->setLEDStatus(homed
            ? libnifalcon::FalconFirmware::GREEN_LED
            : libnifalcon::FalconFirmware::RED_LED);

        // 4. Publish position
        const auto pos = device_.getPosition();
        const auto now = get_clock()->now();

        geometry_msgs::msg::PointStamped pos_msg;
        pos_msg.header.stamp    = now;
        pos_msg.header.frame_id = "falcon";
        pos_msg.point.x = pos[0];
        pos_msg.point.y = pos[1];
        pos_msg.point.z = pos[2];
        pos_pub_->publish(pos_msg);

        // 5. Publish button bitmask (1=plus, 2=forward, 4=center, 8=minus).
        auto grip = device_.getFalconGrip();

        std_msgs::msg::Int32 raw_msg;
        raw_msg.data = static_cast<int32_t>(grip->getDigitalInputs());
        btn_raw_pub_->publish(raw_msg);
    }

    libnifalcon::FalconDevice device_;

    std::array<double, 3> force_;
    std::mutex            force_mutex_;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr    pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr                btn_raw_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr force_sub_;
    rclcpp::TimerBase::SharedPtr                                      timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<FalconNode>());
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("falcon_node"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}