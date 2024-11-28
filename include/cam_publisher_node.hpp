#ifndef CAM_PUBLISHER_NODE_HPP
#define CAM_PUBLISHER_NODE_HPP

#include "rclcpp/rclcpp.hpp"

// Include nav_msgs for Odometry
#include "nav_msgs/msg/odometry.hpp"

// Autoware Messages
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_system_msgs/msg/autoware_state.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_report.hpp"

// ETSI ITS CAM Messages
#include "etsi_its_cam_msgs/msg/cam.hpp"

// Include vehicle info utility
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

// TF2 for transformations
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// GeographicLib for coordinate conversions
#include <GeographicLib/UTMUPS.hpp>

// Include Acceleration message
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"

// Include Trajectory message
#include "autoware_planning_msgs/msg/trajectory.hpp"

// Include PathHistory messages
#include "etsi_its_cam_msgs/msg/path_history.hpp"
#include "etsi_its_cam_msgs/msg/path_point.hpp"

// Standard Libraries
#include <deque>

namespace cam_publisher
{

class CamPublisherNode : public rclcpp::Node
{
public:
  explicit CamPublisherNode(const rclcpp::NodeOptions & options);

private:
  // Subscriber callbacks
  void kinematicStateCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlModeReportCallback(
    const autoware_vehicle_msgs::msg::ControlModeReport::SharedPtr msg);
  void steeringReportCallback(
    const autoware_vehicle_msgs::msg::SteeringReport::SharedPtr msg);
  void autowareStateCallback(
    const autoware_system_msgs::msg::AutowareState::SharedPtr msg);
  void hazardLightsReportCallback(
    const autoware_vehicle_msgs::msg::HazardLightsReport::SharedPtr msg);
  void turnIndicatorsReportCallback(
    const autoware_vehicle_msgs::msg::TurnIndicatorsReport::SharedPtr msg);
  void accelerationCallback(
    const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg);
  void trajectoryCallback(
    const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);

  // Timer callback to publish CAM messages
  void publishCamMessage();

  // Helper functions to populate CAM message fields
  void populateBasicContainer(etsi_its_cam_msgs::msg::BasicContainer & basic_container);
  void populateHighFrequencyContainer(
    etsi_its_cam_msgs::msg::HighFrequencyContainer & high_frequency_container);
  void populateLowFrequencyContainer(
    etsi_its_cam_msgs::msg::LowFrequencyContainer & low_frequency_container);

  // Path History Management
  void updatePathHistory();

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kinematic_state_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_report_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_report_sub_;
  rclcpp::Subscription<autoware_system_msgs::msg::AutowareState>::SharedPtr autoware_state_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr hazard_lights_report_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_indicators_report_sub_;
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr acceleration_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;

  // Publisher
  rclcpp::Publisher<etsi_its_cam_msgs::msg::CAM>::SharedPtr cam_publisher_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Stored data
  nav_msgs::msg::Odometry::SharedPtr kinematic_state_;
  autoware_vehicle_msgs::msg::ControlModeReport::SharedPtr control_mode_report_;
  autoware_vehicle_msgs::msg::SteeringReport::SharedPtr steering_report_;
  autoware_system_msgs::msg::AutowareState::SharedPtr autoware_state_;
  autoware_vehicle_msgs::msg::HazardLightsReport::SharedPtr hazard_lights_report_;
  autoware_vehicle_msgs::msg::TurnIndicatorsReport::SharedPtr turn_indicators_report_;
  geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr acceleration_;
  autoware_planning_msgs::msg::Trajectory::SharedPtr trajectory_;

  // Vehicle dimensions retrieved from vehicle_info_utils
  autoware::vehicle_info_utils::VehicleInfoUtils vehicle_info_utils_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  // Coordinate conversion
  double ref_lat_, ref_lon_, ref_alt_;
  int utm_zone_;
  bool utm_northp_;
  double ref_easting_, ref_northing_;

  // Path History
  std::deque<etsi_its_cam_msgs::msg::PathPoint> path_history_;
  const size_t max_path_points_ = 10;  // ETSI standard limits

  // Current position in latitude and longitude (degrees)
  double current_latitude_deg_;
  double current_longitude_deg_;
  double current_altitude_m_;

  // Unit conversion constants
  // Reference: [ETSI TS 102 894-2 V1.3.1] (2018-08), Intelligent Transport Systems (ITS);
  // Users and applications requirements; Part 2: Applications and facilities layer common data dictionary.
  // Unit definitions can be found throughout the document, e.g., DE_SpeedValue unit in A.74, page 57.

  static constexpr double DE_SpeedValue_Factor = 100.0;  // m/s to cm/s, DE_SpeedValue, A.74, page 57
  static constexpr double DE_LongitudinalAccelerationValue_Factor = 10.0;  // m/s² to 0.1 m/s², DE_LongitudinalAccelerationValue, A.45, page 43
  static constexpr double DE_YawRateValue_Factor = 10000.0;  // rad/s to 0.0001 rad/s, DE_YawRateValue, A.101, page 71
  static constexpr double DE_HeadingValue_Factor = 10.0;  // Degrees to 0.1 degrees, DE_HeadingValue, A.35, page 38
  static constexpr double DE_Latitude_Factor = 1e7;  // degrees to 10^-7 degrees, DE_Latitude, A.41, page 41
  static constexpr double DE_Longitude_Factor = 1e7;  // degrees to 10^-7 degrees, DE_Longitude, A.44, page 43
  static constexpr double DE_AltitudeValue_Factor = 100.0;  // meters to centimeters, DE_AltitudeValue, A.9, page 22
  static constexpr double DE_VehicleLengthValue_Factor = 100.0;  // meters to centimeters, DE_VehicleLengthValue, A.92, page 66
  static constexpr double DE_VehicleWidth_Factor = 100.0;  // meters to centimeters, DE_VehicleWidth, A.95, page 67
  static constexpr double DE_SteeringWheelAngleValue_Factor = 10000.0;  // rad to 0.0001 rad, DE_SteeringWheelAngleValue, A.80, page 61
  static constexpr double DE_CurvatureValue_Factor = 10000.0;  // 1/m to 0.0001 1/m, DE_CurvatureValue, A.15, page 27
  static constexpr double Sec_Mili = 1000.0;  // Seconds to milliseconds
  static constexpr double Rad_Deg = (180.0 / M_PI)
};

}  // namespace cam_publisher

#endif  // CAM_PUBLISHER_NODE_HPP
