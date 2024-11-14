#include "cam_publisher_node.hpp"

// TF2 for transformations
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

// Include UTMUPS
#include <GeographicLib/UTMUPS.hpp>

// Standard Libraries
#include <cmath>

namespace cam_publisher
{

CamPublisherNode::CamPublisherNode(const rclcpp::NodeOptions & options)
: Node("cam_publisher_node", options),
  ref_lat_(this->declare_parameter<double>("reference_latitude", 0.0)),
  ref_lon_(this->declare_parameter<double>("reference_longitude", 0.0)),
  ref_alt_(this->declare_parameter<double>("reference_altitude", 0.0)),
  vehicle_info_utils_(*this)  // Pass *this to match the expected constructor
{
  RCLCPP_INFO(this->get_logger(), "Initializing CAM Publisher Node...");

  int station_id = this->declare_parameter<int>("station_id", 1);

  // Retrieve vehicle information
  vehicle_info_ = vehicle_info_utils_.getVehicleInfo();

  // Convert reference geographic coordinates to UTM
  GeographicLib::UTMUPS::Forward(ref_lat_, ref_lon_, utm_zone_, utm_northp_, ref_easting_, ref_northing_);

  // Initialize subscribers
  kinematic_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", 10,
    std::bind(&CamPublisherNode::kinematicStateCallback, this, std::placeholders::_1));

  control_mode_report_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode", 10,
    std::bind(&CamPublisherNode::controlModeReportCallback, this, std::placeholders::_1));

  steering_report_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", 10,
    std::bind(&CamPublisherNode::steeringReportCallback, this, std::placeholders::_1));

  autoware_state_sub_ = this->create_subscription<autoware_system_msgs::msg::AutowareState>(
    "/autoware/state", 10,
    std::bind(&CamPublisherNode::autowareStateCallback, this, std::placeholders::_1));

  hazard_lights_report_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::HazardLightsReport>(
    "/vehicle/status/hazard_lights_status", 10,
    std::bind(&CamPublisherNode::hazardLightsReportCallback, this, std::placeholders::_1));

  turn_indicators_report_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
    "/vehicle/status/turn_indicators_status", 10,
    std::bind(&CamPublisherNode::turnIndicatorsReportCallback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/vehicle/status/imu", 10,
    std::bind(&CamPublisherNode::imuCallback, this, std::placeholders::_1));

  acceleration_sub_ = this->create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
    "/localization/acceleration", 10,
    std::bind(&CamPublisherNode::accelerationCallback, this, std::placeholders::_1));

  trajectory_sub_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "/planning/trajectory", 10,
    std::bind(&CamPublisherNode::trajectoryCallback, this, std::placeholders::_1));

  // Initialize publisher
  cam_publisher_ = this->create_publisher<etsi_its_cam_msgs::msg::CAM>("/etsi_its_conversion/cam/in", 10);

  // Initialize timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&CamPublisherNode::publishCamMessage, this));

  RCLCPP_INFO(this->get_logger(), "CAM Publisher Node Initialized.");
}

// Callback implementations
void CamPublisherNode::kinematicStateCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  kinematic_state_ = msg;
}

void CamPublisherNode::controlModeReportCallback(
  const autoware_vehicle_msgs::msg::ControlModeReport::SharedPtr msg)
{
  control_mode_report_ = msg;
}

void CamPublisherNode::steeringReportCallback(
  const autoware_vehicle_msgs::msg::SteeringReport::SharedPtr msg)
{
  steering_report_ = msg;
}

void CamPublisherNode::autowareStateCallback(
  const autoware_system_msgs::msg::AutowareState::SharedPtr msg)
{
  autoware_state_ = msg;
}

void CamPublisherNode::hazardLightsReportCallback(
  const autoware_vehicle_msgs::msg::HazardLightsReport::SharedPtr msg)
{
  hazard_lights_report_ = msg;
}

void CamPublisherNode::turnIndicatorsReportCallback(
  const autoware_vehicle_msgs::msg::TurnIndicatorsReport::SharedPtr msg)
{
  turn_indicators_report_ = msg;
}

void CamPublisherNode::imuCallback(
  const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_data_ = msg;
}

void CamPublisherNode::accelerationCallback(
  const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg)
{
  acceleration_ = msg;
}

void CamPublisherNode::trajectoryCallback(
  const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  trajectory_ = msg;
}

void CamPublisherNode::publishCamMessage()
{
  if (!kinematic_state_)
  {
    RCLCPP_WARN(this->get_logger(), "Waiting for necessary data to publish CAM message");
    return;
  }

  // Compute current position in geographic coordinates
  double x = kinematic_state_->pose.pose.position.x;
  double y = kinematic_state_->pose.pose.position.y;
  double z = kinematic_state_->pose.pose.position.z;

  // Calculate absolute UTM coordinates
  double abs_easting = ref_easting_ + x;
  double abs_northing = ref_northing_ + y;

  // Convert UTM coordinates back to geographic coordinates
  GeographicLib::UTMUPS::Reverse(utm_zone_, utm_northp_, abs_easting, abs_northing, current_latitude_deg_, current_longitude_deg_);

  current_altitude_m_ = ref_alt_ + z;

  // Update Path History
  updatePathHistory();

  etsi_its_cam_msgs::msg::CAM cam_msg;

  // ItsPduHeader
  cam_msg.header.protocol_version = 2; // Default value
  cam_msg.header.message_id = etsi_its_cam_msgs::msg::ItsPduHeader::MESSAGE_ID_CAM; // message ID
  cam_msg.header.station_id.value = static_cast<uint32_t>(this->get_parameter("station_id").as_int());

  // GenerationDeltaTime
  cam_msg.cam.generation_delta_time.value = static_cast<uint16_t>(
    (this->now().nanoseconds() / 1000000) % 65536);  // Time in milliseconds modulo 2^16

  // CamParameters
  populateBasicContainer(cam_msg.cam.cam_parameters.basic_container);
  populateHighFrequencyContainer(cam_msg.cam.cam_parameters.high_frequency_container);
  populateLowFrequencyContainer(cam_msg.cam.cam_parameters.low_frequency_container);

  cam_publisher_->publish(cam_msg);

  RCLCPP_DEBUG(this->get_logger(), "Published CAM message.");
}

void CamPublisherNode::populateBasicContainer(etsi_its_cam_msgs::msg::BasicContainer & basic_container)
{
  RCLCPP_DEBUG(this->get_logger(), "Populating Basic Container.");

  // StationType
  basic_container.station_type.value = etsi_its_cam_msgs::msg::StationType::PASSENGER_CAR;

  // ReferencePosition
  basic_container.reference_position.latitude.value = static_cast<int32_t>(
    current_latitude_deg_ * DE_Latitude_Factor);  // Degrees to 10^-7 degrees
  basic_container.reference_position.longitude.value = static_cast<int32_t>(
    current_longitude_deg_ * DE_Longitude_Factor);

  // Altitude
  basic_container.reference_position.altitude.altitude_value.value = static_cast<int32_t>(
    current_altitude_m_ * DE_AltitudeValue_Factor);  // Meters to centimeters
  basic_container.reference_position.altitude.altitude_confidence.value =
    etsi_its_cam_msgs::msg::AltitudeConfidence::ALT_000_01;

  RCLCPP_DEBUG(this->get_logger(), "Populated Basic Container.");
}

void CamPublisherNode::populateHighFrequencyContainer(
  etsi_its_cam_msgs::msg::HighFrequencyContainer & high_frequency_container)
{
  RCLCPP_DEBUG(this->get_logger(), "Populating High Frequency Container.");

  high_frequency_container.choice = high_frequency_container.CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY;
  auto & container = high_frequency_container.basic_vehicle_container_high_frequency;

  // Heading
  double heading_rad = tf2::getYaw(kinematic_state_->pose.pose.orientation);
  double heading_deg = heading_rad * 180.0 / M_PI;
  if (heading_deg < 0)
    heading_deg += 360.0;
  container.heading.heading_value.value = static_cast<uint16_t>(
    heading_deg * DE_HeadingValue_Factor);  // Degrees to 0.1 degrees
  container.heading.heading_confidence.value = etsi_its_cam_msgs::msg::HeadingConfidence::EQUAL_OR_WITHIN_ZERO_POINT_ONE_DEGREE;  // Best confidence

  // Speed
  double speed_mps = kinematic_state_->twist.twist.linear.x;  // Only use linear.x
  container.speed.speed_value.value = static_cast<uint16_t>(
    std::abs(speed_mps) * DE_SpeedValue_Factor);  // m/s to cm/s
  container.speed.speed_confidence.value = etsi_its_cam_msgs::msg::SpeedConfidence::EQUAL_OR_WITHIN_ONE_CENTIMETER_PER_SEC;  // Best confidence

  // Drive Direction
  if (speed_mps >= 0)
  {
    container.drive_direction.value = etsi_its_cam_msgs::msg::DriveDirection::FORWARD;
  }
  else
  {
    container.drive_direction.value = etsi_its_cam_msgs::msg::DriveDirection::BACKWARD;
  }

  // Vehicle Length
  container.vehicle_length.vehicle_length_value.value = static_cast<uint16_t>(
    vehicle_info_.vehicle_length_m * DE_VehicleLengthValue_Factor);  // m to cm
  container.vehicle_length.vehicle_length_confidence_indication.value =
    etsi_its_cam_msgs::msg::VehicleLengthConfidenceIndication::NO_TRAILER_PRESENT;

  // Vehicle Width
  container.vehicle_width.value = static_cast<uint8_t>(
    vehicle_info_.vehicle_width_m * DE_VehicleWidth_Factor);  // m to cm

  // Longitudinal Acceleration
  if (acceleration_)
  {
    double longitudinal_acc_mps2 = acceleration_->accel.accel.linear.x;
    container.longitudinal_acceleration.longitudinal_acceleration_value.value = static_cast<int16_t>(
      longitudinal_acc_mps2 * DE_LongitudinalAccelerationValue_Factor);  // m/s^2 to 0.1 m/s^2
    container.longitudinal_acceleration.longitudinal_acceleration_confidence.value =
      etsi_its_cam_msgs::msg::AccelerationConfidence::POINT_ONE_METER_PER_SEC_SQUARED;  // Best confidence
  }
  else
  {
    container.longitudinal_acceleration.longitudinal_acceleration_value.value = 0;
    container.longitudinal_acceleration.longitudinal_acceleration_confidence.value =
      etsi_its_cam_msgs::msg::AccelerationConfidence::UNAVAILABLE;
  }

  // Yaw Rate
  double yaw_rate_rps = kinematic_state_->twist.twist.angular.z;
  container.yaw_rate.yaw_rate_value.value = static_cast<int16_t>(
    yaw_rate_rps * DE_YawRateValue_Factor);  // rad/s to 0.0001 rad/s
  container.yaw_rate.yaw_rate_confidence.value = etsi_its_cam_msgs::msg::YawRateConfidence::DEG_SEC_000_01;  // Best confidence

  // Steering Wheel Angle
  if (steering_report_)
  {
    container.steering_wheel_angle_is_present = true;
    container.steering_wheel_angle.steering_wheel_angle_value.value = static_cast<int16_t>(
      steering_report_->steering_tire_angle * DE_SteeringWheelAngleValue_Factor);  // rad to 0.0001 rad
    container.steering_wheel_angle.steering_wheel_angle_confidence.value =
      etsi_its_cam_msgs::msg::SteeringWheelAngleConfidence::EQUAL_OR_WITHIN_ONE_POINT_FIVE_DEGREE;  // Best confidence
  }
  else
  {
    container.steering_wheel_angle_is_present = false;
  }

  // Lateral and Vertical Acceleration
  if (imu_data_)
  {
    double lateral_acc_mps2 = imu_data_->linear_acceleration.y;
    container.lateral_acceleration_is_present = true;
    container.lateral_acceleration.lateral_acceleration_value.value = static_cast<int16_t>(
      lateral_acc_mps2 * DE_LongitudinalAccelerationValue_Factor);  // m/s^2 to 0.1 m/s^2
    container.lateral_acceleration.lateral_acceleration_confidence.value =
      etsi_its_cam_msgs::msg::AccelerationConfidence::POINT_ONE_METER_PER_SEC_SQUARED;  // Best confidence

    double vertical_acc_mps2 = imu_data_->linear_acceleration.z;
    container.vertical_acceleration_is_present = true;
    container.vertical_acceleration.vertical_acceleration_value.value = static_cast<int16_t>(
      vertical_acc_mps2 * DE_LongitudinalAccelerationValue_Factor);  // m/s^2 to 0.1 m/s^2
    container.vertical_acceleration.vertical_acceleration_confidence.value =
      etsi_its_cam_msgs::msg::AccelerationConfidence::POINT_ONE_METER_PER_SEC_SQUARED;  // Best confidence
  }
  else
  {
    container.lateral_acceleration_is_present = false;
    container.vertical_acceleration_is_present = false;
  }

  // Curvature
  bool used_yaw_rate = false;
  if (speed_mps != 0 && yaw_rate_rps != 0)
  {
    // Compute curvature from yaw rate and speed
    // Curvature (1/m) = yaw_rate (rad/s) / speed (m/s)
    double curvature = yaw_rate_rps / speed_mps;

    container.curvature.curvature_value.value = static_cast<int16_t>(
      curvature * DE_CurvatureValue_Factor);  // 1/m to 0.0001 1/m

    // Use appropriate confidence value
    container.curvature.curvature_confidence.value = etsi_its_cam_msgs::msg::CurvatureConfidence::ONE_PER_METER_0_0001;  // Adjust as needed

    used_yaw_rate = true;
  }
  else if (trajectory_)
  {
    // Use trajectory data to compute curvature if yaw rate is not available
    if (trajectory_->points.size() >= 3)
    {
      auto & p0 = trajectory_->points[0];
      auto & p1 = trajectory_->points[1];
      auto & p2 = trajectory_->points[2];

      // Coordinates
      double x0 = p0.pose.position.x;
      double y0 = p0.pose.position.y;
      double x1 = p1.pose.position.x;
      double y1 = p1.pose.position.y;
      double x2 = p2.pose.position.x;
      double y2 = p2.pose.position.y;

      // Compute curvature
      double numerator = std::abs((x1 - x0)*(y2 - y0) - (x2 - x0)*(y1 - y0));
      double denominator = std::pow(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2), 1.5);

      double curvature = 0.0;
      if (denominator != 0)
      {
        curvature = numerator / denominator;
      }

      container.curvature.curvature_value.value = static_cast<int16_t>(
        curvature * DE_CurvatureValue_Factor);  // 1/m to 0.0001 1/m

      // Use appropriate confidence value
      container.curvature.curvature_confidence.value = etsi_its_cam_msgs::msg::CurvatureConfidence::ONE_PER_METER_0_0001; 

      used_yaw_rate = false;
    }
    else
    {
      // Set curvature to unavailable
      container.curvature.curvature_value.value = etsi_its_cam_msgs::msg::CurvatureValue::UNAVAILABLE;
      container.curvature.curvature_confidence.value = etsi_its_cam_msgs::msg::CurvatureConfidence::UNAVAILABLE;
      used_yaw_rate = false;
    }
  }
  else
  {
    // Set curvature to unavailable
    container.curvature.curvature_value.value = etsi_its_cam_msgs::msg::CurvatureValue::UNAVAILABLE;
    container.curvature.curvature_confidence.value = etsi_its_cam_msgs::msg::CurvatureConfidence::UNAVAILABLE;
    used_yaw_rate = false;
  }

  // CurvatureCalculationMode
  if (used_yaw_rate)
  {
    container.curvature_calculation_mode.value = etsi_its_cam_msgs::msg::CurvatureCalculationMode::YAW_RATE_USED;
  }
  else
  {
    container.curvature_calculation_mode.value = etsi_its_cam_msgs::msg::CurvatureCalculationMode::YAW_RATE_NOT_USED;
  }

  RCLCPP_DEBUG(this->get_logger(), "Populated High Frequency Container.");
}

void CamPublisherNode::populateLowFrequencyContainer(
  etsi_its_cam_msgs::msg::LowFrequencyContainer & low_frequency_container)
{
  RCLCPP_DEBUG(this->get_logger(), "Populating Low Frequency Container.");

  low_frequency_container.choice = low_frequency_container.CHOICE_BASIC_VEHICLE_CONTAINER_LOW_FREQUENCY;

  auto & container = low_frequency_container.basic_vehicle_container_low_frequency;

  // VehicleRole
  container.vehicle_role.value = etsi_its_cam_msgs::msg::VehicleRole::PUBLIC_TRANSPORT;

  // ExteriorLights
  container.exterior_lights.value.resize(1);
  container.exterior_lights.bits_unused = 0;
  container.exterior_lights.value[0] = 0;

  if (hazard_lights_report_)
  {
    if (hazard_lights_report_->report == autoware_vehicle_msgs::msg::HazardLightsReport::ENABLE)
    {
      // Since hazard lights are not explicitly defined, set both turn signals
      container.exterior_lights.value[0] |= (1 << etsi_its_cam_msgs::msg::ExteriorLights::BIT_INDEX_LEFT_TURN_SIGNAL_ON);
      container.exterior_lights.value[0] |= (1 << etsi_its_cam_msgs::msg::ExteriorLights::BIT_INDEX_RIGHT_TURN_SIGNAL_ON);
    }
  }

  if (turn_indicators_report_)
  {
    if (turn_indicators_report_->report == autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT)
    {
      container.exterior_lights.value[0] |= (1 << etsi_its_cam_msgs::msg::ExteriorLights::BIT_INDEX_LEFT_TURN_SIGNAL_ON);
    }
    if (turn_indicators_report_->report == autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT)
    {
      container.exterior_lights.value[0] |= (1 << etsi_its_cam_msgs::msg::ExteriorLights::BIT_INDEX_RIGHT_TURN_SIGNAL_ON);
    }
  }

  // PathHistory
  container.path_history.array.clear();

  for (const auto & path_point : path_history_)
  {
    container.path_history.array.push_back(path_point);
  }

  RCLCPP_DEBUG(this->get_logger(), "Populated Low Frequency Container.");
}

void CamPublisherNode::updatePathHistory()
{
  // Update the path history based on the current position
  if (!kinematic_state_)
    return;

  double current_time = this->now().seconds();

  if (path_history_.empty())
  {
    // Initialize the path history with the current position
    etsi_its_cam_msgs::msg::PathPoint path_point;
    path_point.path_position.delta_latitude.value = 0;
    path_point.path_position.delta_longitude.value = 0;
    path_point.path_position.delta_altitude.value = 0;
    path_point.path_delta_time_is_present = false;
    path_history_.push_back(path_point);
  }
  else
  {
    // Compute delta latitude and longitude
    double prev_latitude_deg = (path_history_.back().path_position.delta_latitude.value) / DE_Latitude_Factor;
    double prev_longitude_deg = (path_history_.back().path_position.delta_longitude.value) / DE_Longitude_Factor;
    double delta_latitude_deg = current_latitude_deg_ - prev_latitude_deg;
    double delta_longitude_deg = current_longitude_deg_ - prev_longitude_deg;

    // Convert to DE_DeltaLatitude and DE_DeltaLongitude units (1e-7 degrees)
    int32_t delta_latitude = static_cast<int32_t>(delta_latitude_deg * DE_Latitude_Factor);
    int32_t delta_longitude = static_cast<int32_t>(delta_longitude_deg * DE_Longitude_Factor);

    // Compute delta time
    double prev_time = path_history_.back().path_delta_time.value / 1000.0;  // milliseconds to seconds
    double delta_time = current_time - prev_time;

    etsi_its_cam_msgs::msg::PathPoint path_point;
    path_point.path_position.delta_latitude.value = delta_latitude;
    path_point.path_position.delta_longitude.value = delta_longitude;
    path_point.path_position.delta_altitude.value = 0;  // Assuming no change in altitude
    path_point.path_delta_time.value = static_cast<uint16_t>(delta_time *Sec_Mili);  // seconds to milliseconds
    path_point.path_delta_time_is_present = true;

    // Add the new point to the path history
    path_history_.push_back(path_point);

    // Ensure the path history does not exceed the maximum size
    if (path_history_.size() > max_path_points_)
    {
      path_history_.pop_front();
    }
  }
}

}  // namespace cam_publisher

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cam_publisher::CamPublisherNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
