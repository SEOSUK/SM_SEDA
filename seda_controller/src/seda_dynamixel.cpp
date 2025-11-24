// seda_dynamixel.cpp
//
// ROS2 node for Dynamixel XM540-W270-T (ID=1, 1Mbps)
// - Subscribes: "theta_cmd" (std_msgs::msg::Float64), angle command [rad]
// - Publishes:  "theta_meas" (std_msgs::msg::Float64), measured angle [rad]
// - 200 Hz timer: read present position + write goal position
//
// NOTE:
//  - Uses ROBOTIS Dynamixel SDK (dynamixel_sdk).
//  - Default device: /dev/ttyUSB0, protocol 2.0, position in [0, 2π) rad.
//  - 필요하면 device_name, dxl_id, baudrate를 파라미터로 바꿔서 사용.

#include <chrono>
#include <cmath>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std::chrono_literals;

class SEDADynamixelNode : public rclcpp::Node
{
public:
  SEDADynamixelNode()
  : Node("seda_dynamixel"),
    theta_cmd_(0.0),
    theta_meas_(0.0),
    portHandler_(nullptr),
    packetHandler_(nullptr),
    port_opened_(false)
  {
    // Parameters
    device_name_ = this->declare_parameter<std::string>("device_name", "/dev/ttyUSB1");
    dxl_id_      = this->declare_parameter<int>("dxl_id", 1);
    baudrate_    = this->declare_parameter<int>("baudrate", 1000000);

    RCLCPP_INFO(this->get_logger(),
                "Initializing Dynamixel on %s, ID=%d, baud=%d",
                device_name_.c_str(), dxl_id_, baudrate_);

    // Dynamixel SDK 초기화
    portHandler_   = dynamixel::PortHandler::getPortHandler(device_name_.c_str());
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!openPortAndSetBaudrate()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to open port or set baudrate. Node will run but Dynamixel I/O will fail.");
    } else {
      enableTorque(true);
    }

    // Publisher: theta_meas
    theta_meas_pub_ = this->create_publisher<std_msgs::msg::Float64>("theta_meas", 10);

    // Subscriber: theta_cmd
    theta_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "theta_cmd",
      10,
      std::bind(&SEDADynamixelNode::thetaCmdCallback, this, std::placeholders::_1));

    // 200 Hz timer
    timer_ = this->create_wall_timer(
      5ms,
      std::bind(&SEDADynamixelNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "SEDA Dynamixel node initialized.");
  }

  ~SEDADynamixelNode() override
  {
    enableTorque(false);
    if (portHandler_ && port_opened_) {
      portHandler_->closePort();
      port_opened_ = false;
    }
  }

private:
  // XM540-W270-T (protocol 2.0) control table 정보
  static constexpr double   PROTOCOL_VERSION   = 2.0;
  static constexpr uint16_t ADDR_TORQUE_ENABLE = 64;
  static constexpr uint16_t ADDR_GOAL_POSITION = 116;
  static constexpr uint16_t ADDR_PRESENT_POS   = 132;
  static constexpr uint8_t  TORQUE_ENABLE      = 1;
  static constexpr uint8_t  TORQUE_DISABLE     = 0;
  static constexpr int      DXL_MIN_POS_VAL    = 0;      // 0 tick
  static constexpr int      DXL_MAX_POS_VAL    = 4095;   // 4095 tick (0~360deg)

  // 200 Hz 타이머 콜백
  void timerCallback()
  {
    if (!portHandler_ || !port_opened_) {
      return;
    }

    // 1) 현재 각도 읽기 → theta_meas_ 업데이트 & publish
    readTheta();
    publishThetaMeas();

    // 2) theta_cmd_ → goal position 쓰기
    writeThetaCmd();
  }

  // theta_cmd 구독 콜백
  void thetaCmdCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    theta_cmd_ = msg->data;  // rad
  }

  // Dynamixel에서 present position 읽고 theta_meas_ 업데이트
  void readTheta()
  {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    uint32_t present_pos = 0;

    dxl_comm_result = packetHandler_->read4ByteTxRx(
      portHandler_, dxl_id_, ADDR_PRESENT_POS, &present_pos, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Failed to read present position: %s",
        packetHandler_->getTxRxResult(dxl_comm_result));
      return;
    } else if (dxl_error != 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Dynamixel error while reading present position: %s",
        packetHandler_->getRxPacketError(dxl_error));
      return;
    }

    // ticks [0..4095] -> rad [0..2π)
    double theta_raw = (static_cast<double>(present_pos) / 4095.0) * 2.0 * M_PI;

    // convert [0, 2π) → [−π, +π)
    double theta_rad = theta_raw - M_PI;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      theta_meas_ = theta_rad;
    }
  }

  // theta_meas_를 theta_meas 토픽으로 publish
  void publishThetaMeas()
  {
    std_msgs::msg::Float64 msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msg.data = theta_meas_;
    }
    theta_meas_pub_->publish(msg);
  }

  // theta_cmd_를 ticks로 변환해서 Goal Position으로 쓰기
void writeThetaCmd()
{
  double theta_cmd_local;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    theta_cmd_local = theta_cmd_;
  }

  // ===== NEW: limit angle to [-45°, +45°] =====
  constexpr double ANGLE_MIN = -M_PI / 4.0;   // -45 deg
  constexpr double ANGLE_MAX =  M_PI / 4.0;   // +45 deg

  if (theta_cmd_local < ANGLE_MIN) theta_cmd_local = ANGLE_MIN;
  if (theta_cmd_local > ANGLE_MAX) theta_cmd_local = ANGLE_MAX;
  // ============================================

  // rad → ticks (Dynamixel expects [0, 2π) so re-map range)
  constexpr double TWO_PI = 2.0 * M_PI;

  // shift [-45°,45°] into [0,2π)
  theta_cmd_local += M_PI;   // shift by 180° to avoid negative
  if (theta_cmd_local >= TWO_PI) theta_cmd_local -= TWO_PI;

  // rad → ticks [0..4095]
  int goal_pos = static_cast<int>((theta_cmd_local / TWO_PI) * 4095.0);
  if (goal_pos < DXL_MIN_POS_VAL) goal_pos = DXL_MIN_POS_VAL;
  if (goal_pos > DXL_MAX_POS_VAL) goal_pos = DXL_MAX_POS_VAL;

  uint8_t dxl_error = 0;
  int dxl_comm_result = packetHandler_->write4ByteTxRx(
    portHandler_, dxl_id_, ADDR_GOAL_POSITION,
    static_cast<uint32_t>(goal_pos), &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Failed to write goal position: %s",
      packetHandler_->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Dynamixel error while writing goal position: %s",
      packetHandler_->getRxPacketError(dxl_error));
  }
}

  // 포트 열고 보드레이트 세팅
  bool openPortAndSetBaudrate()
  {
    if (!portHandler_) {
      RCLCPP_ERROR(this->get_logger(), "PortHandler is null");
      return false;
    }

    if (!portHandler_->openPort()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open port %s", device_name_.c_str());
      port_opened_ = false;
      return false;
    }

    if (!portHandler_->setBaudRate(baudrate_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate %d", baudrate_);
      portHandler_->closePort();
      port_opened_ = false;
      return false;
    }

    port_opened_ = true;
    return true;
  }

  // 토크 온/오프
  void enableTorque(bool enable)
  {
    if (!portHandler_ || !port_opened_) {
      return;
    }

    uint8_t value = enable ? TORQUE_ENABLE : TORQUE_DISABLE;
    uint8_t dxl_error = 0;

    int dxl_comm_result = packetHandler_->write1ByteTxRx(
      portHandler_, dxl_id_, ADDR_TORQUE_ENABLE, value, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to %s torque: %s",
                   enable ? "enable" : "disable",
                   packetHandler_->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Dynamixel error while %s torque: %s",
                   enable ? "enabling" : "disabling",
                   packetHandler_->getRxPacketError(dxl_error));
    } else {
      RCLCPP_INFO(this->get_logger(), "Torque %s",
                  enable ? "enabled" : "disabled");
    }
  }

private:
  // ROS
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr theta_meas_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr theta_cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Dynamixel
  dynamixel::PortHandler   *portHandler_;
  dynamixel::PacketHandler *packetHandler_;
  std::string device_name_;
  int dxl_id_;
  int baudrate_;
  bool port_opened_;

  // 상태 변수
  std::mutex mutex_;
  double theta_cmd_;   // 명령 각도 [rad]
  double theta_meas_;  // 측정 각도 [rad]
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SEDADynamixelNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
