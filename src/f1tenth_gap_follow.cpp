#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <cstdio>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <numbers>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/header.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace ackermann_msgs::msg;
using namespace std_msgs::msg;
using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;
using namespace std::numbers;

// Distance from rear wheels
static const double HOKUYO_LASER_OFFSET_M = 0.29;

// Distance between front and rear wheels
static const double ROBOT_WHEEL_BASE_M = 0.325;

class F1TenthGapFollow : public rclcpp::Node {
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_scan_sub;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      m_drive_pub;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_lookahead_pub;

 public:
  F1TenthGapFollow() : Node("f1tenth_gap_follow") {
    m_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&F1TenthGapFollow::scan_callback, this,
                  std::placeholders::_1));
    m_drive_pub =
        this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            /* "drive", 10); */
            "gap_follow_ackermann_cmd", 10);  // drive

    m_lookahead_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "gap_follow_lookahead", 10);  // lookahead point
  }

 private:
  struct ScanProperties {
    const LaserScan* msg = nullptr;
    int min_index;
    int max_index;

    // number of consecutive beams that can be ignored
    int gap_error_tolerance;
  };

  struct Gap {
    int start_index = 0;
    int size = 0;
  };

  Gap find_largest_gap(const ScanProperties& scan, double range_threshold) {
    Gap max_gap;
    Gap current_gap;

    int num_consecutive_errors = 0;
    bool in_gap = false;

    for (int i = scan.min_index; i <= scan.max_index; ++i) {
      const bool is_gap_present = scan.msg->ranges[i] > range_threshold;

      if (is_gap_present) {
        if (in_gap) {  // Extend current gap
          current_gap.size++;
          if (num_consecutive_errors) {  // Ending a small error
            current_gap.size += num_consecutive_errors;
          }
          num_consecutive_errors = 0;
          if (current_gap.size > max_gap.size) {
            max_gap.size = current_gap.size;
            max_gap.start_index = current_gap.start_index;
          }
        } else {  // Start new gap
          in_gap = true;
          current_gap.start_index = i;
          current_gap.size = 1;
          num_consecutive_errors = 0;
        }
      } else {
        if (in_gap) {
          if (num_consecutive_errors < scan.gap_error_tolerance) {
            num_consecutive_errors++;
          } else {  // End current gap
            in_gap = false;
          }
        }
      }
    }

    return max_gap;
  }

  void scan_callback(const LaserScan& msg) {
    const ScanProperties scan_props{
        .msg = &msg,
        .min_index = convert_angle_to_index(-M_PI / 2.0, msg),
        .max_index = convert_angle_to_index(+M_PI / 2.0, msg),
        .gap_error_tolerance =
            static_cast<int>(0.035 / msg.angle_increment),  // 2 deg
    };

    AckermannDrive drive_msg;
    drive_msg.speed = 0.0;
    drive_msg.steering_angle = 0.0;

    double lookahead = 1.5;
    Gap gap = find_largest_gap(scan_props, lookahead);
    double gap_angle = gap.size * msg.angle_increment;

    if (gap_angle > 0.1) {
      double steering_angle_rad = calculate_steering_angle(msg, gap, lookahead);
      double abs_steering_angle_rad = std::abs(steering_angle_rad);

      // Set output

      drive_msg.speed = 2.5;

      if (abs_steering_angle_rad > 0.17) {
        drive_msg.speed = 0.5;
      } else if (abs_steering_angle_rad > 0.087) {
        drive_msg.speed = 1.0;
      } else if (abs_steering_angle_rad > 0.035) {
        drive_msg.speed = 2.0;
      }

      drive_msg.steering_angle = steering_angle_rad;
    } else {
      // Try a smaller lookahead distance and drive slowly.

      lookahead = 0.75;
      gap = find_largest_gap(scan_props, lookahead);
      gap_angle = gap.size * msg.angle_increment;

      if (gap_angle > 0.1) {
        double steering_angle_rad =
            calculate_steering_angle(msg, gap, lookahead);

        drive_msg.speed = 0.5;
        drive_msg.steering_angle = steering_angle_rad;
      }
    }

    AckermannDriveStamped drive_stamped_msg;
    drive_stamped_msg.header.stamp = this->now();
    drive_stamped_msg.drive = drive_msg;
    m_drive_pub->publish(drive_stamped_msg);
  }

  int convert_angle_to_index(double angle_rad, const LaserScan& scan) {
    int index =
        static_cast<int>((angle_rad - scan.angle_min) / scan.angle_increment);
    if (index < 0 || index >= (int)scan.ranges.size()) return 0;
    return index;
  }

  double convert_index_to_angle(int index, const LaserScan& scan) {
    if (index < 0) return scan.angle_min;
    if (index >= (int)scan.ranges.size()) return scan.angle_max;

    return scan.angle_min + index * scan.angle_increment;
  }

  Point get_relative_goal_point(double angle_rad, double distance) {
    double y = std::sin(angle_rad) * distance;
    double x = std::cos(angle_rad) * distance;

    Point goal_point;
    goal_point.x = x + HOKUYO_LASER_OFFSET_M;
    goal_point.y = y;

    return goal_point;
  }

  double calculate_steering_angle(const LaserScan& msg, const Gap& gap,
                                  double lookahead_distance) {
    int gap_center_index = gap.start_index + gap.size / 2;
    double gap_center_angle = convert_index_to_angle(gap_center_index, msg);

    Point goal = get_relative_goal_point(gap_center_angle, lookahead_distance);
    double goal_distance = std::hypot(goal.x, goal.y);

    // Publish the lookahead point for the simulation
    PoseStamped lookahead_msg;
    lookahead_msg.header.stamp = this->now();
    lookahead_msg.header.frame_id = "ego_racecar/base_link";
    lookahead_msg.pose.position.x = goal.x;
    lookahead_msg.pose.position.y = goal.y;
    lookahead_msg.pose.position.z = 0.0;
    lookahead_msg.pose.orientation.w = 1.0;  // No rotation
    m_lookahead_pub->publish(lookahead_msg);

    double turn_radius = std::pow(goal_distance, 2) / (2 * std::abs(goal.y));

    double steering_angle_rad = 0.0;
    if (turn_radius > 0.01) {
      steering_angle_rad = std::atan2(ROBOT_WHEEL_BASE_M, turn_radius);
    }
    steering_angle_rad = std::min(steering_angle_rad, 0.26);

    return std::copysign(steering_angle_rad, goal.y);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<F1TenthGapFollow>());
  rclcpp::shutdown();
  return 0;
}
