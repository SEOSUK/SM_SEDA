// seda_controller.cpp
// GPT TO DO: 
// High-level controller node
// - Subscribes:
//     /encoder_angle (std_msgs::msg::Float32) → phi_meas_ [rad]
//     /theta_meas    (std_msgs::msg::Float64) → theta_meas_ [rad]
// - Publishes:
//     /theta_cmd     (std_msgs::msg::Float64) → theta_cmd_ [rad]
//     /q_meas        (std_msgs::msg::Float64) → q_meas_    [rad]
//     /q_cmd         (std_msgs::msg::Float64) → q_cmd_     [rad]
//     /q_dot_meas    (std_msgs::msg::Float64) → q_dot_meas_ [rad/s]
// - Timer (기본 200 Hz):
//     commandGeneration()          → q_cmd_
//     computeAnalyticCalculation() → q_meas_, q_dot_meas_
//     computeMCG()                 → M_, G_
//     computeThetaCommand()        → theta_cmd_
//     이후 위 값들을 publish
//
// Dynamics:
//   M(q) q_ddot + G(q) = tau
//   M(q) = I + m l_c^2
//   G(q) = m g l_c cos(q)  (또는 use_gravity=false면 0)
//
// Controller:
//   e      = q_cmd - q
//   v      = Kp e + Ki ∫e dt - Kd q_dot
//   tau_des = M(q) v + G(q)
//   theta_cmd = q + tau_des / K   (K = spring stiffness)
//
// 모든 파라미터는 config.yaml에서 override 가능하도록 할 것.

#include <chrono>
#include <mutex>
#include <cmath>
#include <algorithm>  // std::clamp  // <<< NEW

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

#include <fstream>
#include <filesystem>
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

class SEDAController : public rclcpp::Node
{
public:
  SEDAController()
  : Node("seda_controller"),
    phi_meas_(0.0),
    theta_meas_(0.0),
    theta_cmd_(0.0),
    q_meas_(0.0),
    q_meas_prev_(0.0),
    q_cmd_(0.0),
    q_dot_meas_(0.0),
    real_time_(0.0),
    q_error_int_(0.0)      // <<< NEW: integral state init
  {
    // ===== Parameters =====
    // 샘플링 시간 (기본 200 Hz = 0.005 s)
    dt_ = this->declare_parameter<double>("dt", 0.005);

    // 물리 파라미터
    I_           = this->declare_parameter<double>("I",   0.01);   // kg m^2 등
    m_           = this->declare_parameter<double>("m",   0.1);
    lc_          = this->declare_parameter<double>("lc",  0.05);   // m
    g_           = this->declare_parameter<double>("g",   9.81);
    use_gravity_ = this->declare_parameter<bool>("use_gravity", true);

    // 제어 파라미터
    Kp_        = this->declare_parameter<double>("Kp", 5.0);
    Kd_        = this->declare_parameter<double>("Kd", 0.5);
    K_spring_  = this->declare_parameter<double>("K_spring", 1.0);  // spring stiffness
    q_cmd_default_ = this->declare_parameter<double>("q_cmd_default", 0.0);

    // <<< NEW: Integral gain & saturation
    Ki_        = this->declare_parameter<double>("Ki", 0.0);         // default 0 → 기존 PD와 동일
    I_max_     = this->declare_parameter<double>("I_max", 1.0);      // integral state saturation [rad·s]

    // <<< NEW: outer integral gain on final command
    outer_Ki_  = this->declare_parameter<double>("outer_Ki", 0.0);   // outer I gain on theta_cmd

    // Step reference parameters
    step_period_ = this->declare_parameter<double>("step_period", 3.0);
    step_angle_  = this->declare_parameter<double>("step_angle", M_PI/2);

    // q_dot LPF cutoff [Hz] 파라미터 (기본 10 Hz)
    double cutoff_hz = this->declare_parameter<double>("qdot_lpf_cutoff_hz", 10.0);
    // 1차 저역통과 필터 계수 alpha 계산
    // RC = 1 / (2*pi*fc), alpha = RC / (RC + dt)
    if (cutoff_hz <= 0.0) {
      lpf_alpha_ = 1.0;  // effectively frozen (no update)
    } else {
      double RC = 1.0 / (2.0 * M_PI * cutoff_hz);
      lpf_alpha_ = RC / (RC + dt_);
    }


    RCLCPP_INFO(this->get_logger(),
                "SEDA Controller init: dt = %.4f, cutoff = %.2f Hz, alpha = %.4f",
                dt_, cutoff_hz, lpf_alpha_);
    RCLCPP_INFO(this->get_logger(),
                "Params: I=%.4f, m=%.4f, lc=%.4f, g=%.4f, use_gravity=%d, "
                "Kp=%.4f, Ki=%.4f, Kd=%.4f, K_spring=%.4f, I_max=%.4f, outer_Ki=%.4f",
                I_, m_, lc_, g_, use_gravity_, Kp_, Ki_, Kd_, K_spring_, I_max_, outer_Ki_);


    try {
      namespace fs = std::filesystem;

      // CMake에서 넘겨준 패키지 소스 디렉토리 (예: ~/ros2_ws/src/seda_controller)
      fs::path src_dir(SEDA_CONTROLLER_SRC_DIR);

      // src/seda_controller/bag 디렉토리
      fs::path bag_dir = src_dir / "bag";
      fs::create_directories(bag_dir);

      // CSV 파일 경로: src/seda_controller/bag/seda_log.csv
      csv_path_ = (bag_dir / "seda_log.csv").string();

      csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
      if (csv_file_.is_open()) {
        csv_file_ << "time,q_cmd,q_meas,theta_cmd,theta_meas\n";
        csv_file_.flush();
        csv_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "CSV logging to: %s", csv_path_.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to open CSV log file at %s", csv_path_.c_str());
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(),
                  "Exception initializing CSV logging: %s", e.what());
    }




    // ===== Subscribers =====
    // /encoder_angle (Float32) → phi_meas_
    encoder_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/encoder_angle",
      10,
      std::bind(&SEDAController::encoderCallback, this, std::placeholders::_1));

    // /theta_meas (Float64) → theta_meas_
    theta_meas_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/theta_meas",
      10,
      std::bind(&SEDAController::thetaMeasCallback, this, std::placeholders::_1));

    // ===== Publishers =====
    theta_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/theta_cmd", 10);
    q_meas_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/q_meas", 10);
    q_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/q_cmd", 10);
    q_dot_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/q_dot_meas", 10);

    // ===== Timer (control loop) =====
    auto period = std::chrono::duration<double>(dt_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&SEDAController::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "SEDA Controller node initialized.");
  }

private:
  // ===== 콜백들 =====

  // /encoder_angle → phi_meas_
  void encoderCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    phi_meas_ = static_cast<double>(msg->data);
  }

  // /theta_meas → theta_meas_
  void thetaMeasCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    theta_meas_ = msg->data;
  }

  // =====  메인 제어 타이머 (dt 간격) =====
  void timerCallback()
  {
    real_time_ += dt_;

    // 1) 명령 생성 (참조 q_cmd)
    commandGeneration();

    // 2) q, q_dot 계산
    computeAnalyticCalculation();

    // 3) M, G 계산
    computeMCG();

    // 4) 제어 law로 theta_cmd 계산
    computeThetaCommand();

    // 5) publish (theta_cmd, q_meas, q_cmd, q_dot)
    std_msgs::msg::Float64 cmd_msg;
    std_msgs::msg::Float64 q_meas_msg;
    std_msgs::msg::Float64 q_cmd_msg;
    std_msgs::msg::Float64 q_dot_msg;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmd_msg.data    = theta_cmd_;
      q_meas_msg.data = q_meas_;
      q_cmd_msg.data  = q_cmd_;
      q_dot_msg.data  = q_dot_meas_;
    }

    theta_cmd_pub_->publish(cmd_msg);
    q_meas_pub_->publish(q_meas_msg);
    q_cmd_pub_->publish(q_cmd_msg);
    q_dot_pub_->publish(q_dot_msg);


    // 6) csv logging
    for_csv_logging();
  }

  // ===== Reference / Command Generation =====
  void commandGeneration()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // real_time_ [sec], step_period_ [sec], step_angle_ [rad]

    // 현재 얼마나 많은 period가 지난 상태인가?
    int k = static_cast<int>( std::floor(real_time_ / step_period_) );

    // 짝수 / 홀수에 따라 step 출력
    if (k % 2 == 0) {
      q_cmd_ = 0.0;             // even interval → baseline
    } else {
      q_cmd_ = step_angle_;     // odd interval  → step
    }
  }

  // ===== q_meas, q_dot_meas 계산 + 1차 LPF =====
  void computeAnalyticCalculation()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // q = phi + theta (조인트 합 각)
    q_meas_ = phi_meas_ + theta_meas_;

    // 미분으로 속도 추정 (raw)
    double q_dot_raw = (q_meas_ - q_meas_prev_) / dt_;

    // 1차 low-pass filter:
    // q_dot_filtered[k] = alpha * q_dot_filtered[k-1] + (1 - alpha) * q_dot_raw[k]
    q_dot_meas_ = lpf_alpha_ * q_dot_meas_ + (1.0 - lpf_alpha_) * q_dot_raw;

    // 다음 스텝을 위한 이전값 업데이트
    q_meas_prev_ = q_meas_;
  }

  // ===== M(q), G(q) 계산 =====
  void computeMCG()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // M(q) = I + m l_c^2 (q에 의존하지 않는 상수)
    M_ = I_ + m_ * lc_ * lc_;

    // G(q) = m g l_c cos(q) 또는 0
    if (use_gravity_) {
      G_ = m_ * g_ * lc_ * std::cos(q_meas_);
    } else {
      G_ = 0.0;
    }
  }

  // ===== 제어 law: PID(사실상 P+I+D) + Feedback Linearization + Control Allocation =====
void computeThetaCommand()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Joint error
  double q_error = q_cmd_ - q_meas_;

  // ∫e dt (공통 적분 상태, anti-windup)
  q_error_int_ += q_error * dt_;
  q_error_int_ = std::clamp(q_error_int_, -I_max_, I_max_);

  // === Feedback Linearization 쪽 가상 입력 v ===
  // v = Kp e + Ki ∫e dt - Kd q_dot
  double v = Kp_ * q_error + Ki_ * q_error_int_ - Kd_ * q_dot_meas_;

  // Feedback linearization: tau_des = M(q) v + G(q)
  double tau_des = M_ * v + G_;

  // === 스프링을 통한 theta 명령 + outer I term ===
  // 기본 feedforward: theta_ff = q_cmd + tau_des / K_spring
  if (K_spring_ > 1e-6) {
    double theta_ff = q_cmd_ + tau_des / K_spring_;

    // 최종 명령단 I 게인: outer_Ki * ∫e dt
    theta_cmd_ = theta_ff + outer_Ki_ * q_error_int_;
  } else {
    // spring 모델 잘못되면 그냥 q_cmd 추종 + outer I만 적용
    theta_cmd_ = q_cmd_ + outer_Ki_ * q_error_int_;
  }
}


  void for_csv_logging()
  {
    // 1st: real_time
    // 2nd: q_cmd
    // 3rd: q_meas
    // 4th: theta_cmd
    // 5th: theta_meas

    if (!csv_initialized_ || !csv_file_.is_open()) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    csv_file_
      << real_time_   << ","
      << q_cmd_       << ","
      << q_meas_      << ","
      << theta_cmd_   << ","
      << theta_meas_  << "\n";

    // 자주 flush 해서 노드가 비정상 종료돼도 최대한 데이터 남도록
    csv_file_.flush();
  }




private:
  // ROS 인터페이스
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr encoder_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr theta_meas_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr theta_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr q_meas_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr q_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr q_dot_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 상태 변수들
  std::mutex mutex_;
  double phi_meas_;      // from /encoder_angle
  double theta_meas_;    // from /theta_meas
  double theta_cmd_;     // to /theta_cmd

  double q_meas_;        // phi + theta (combined angle)
  double q_meas_prev_;   // prev q_meas for derivative
  double q_cmd_;         // desired q (reference)
  double q_dot_meas_;    // filtered dq/dt

  double real_time_;     // [s]
  double dt_;            // [s]

  // LPF
  double lpf_alpha_;     // 1st order LPF coefficient for q_dot

  // 물리 파라미터
  double I_;
  double m_;
  double lc_;
  double g_;
  bool   use_gravity_;

  // 제어 파라미터
  double Kp_;
  double Ki_;        // FL용 I gain
  double Kd_;
  double K_spring_;
  double q_cmd_default_;

  double I_max_;        // integral saturation bound
  double q_error_int_;  // ∫(q_cmd - q_meas) dt
  double outer_Ki_;     // 최종 theta_cmd에 붙는 I gain

  // 동역학 항 (매 step 업데이트)
  double M_;             // inertia scalar
  double G_;             // gravity term

  double step_period_;   // [sec]
  double step_angle_;    // [rad]  

  // CSV logging
  std::ofstream csv_file_;
  bool csv_initialized_ = false;
  std::string csv_path_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SEDAController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
