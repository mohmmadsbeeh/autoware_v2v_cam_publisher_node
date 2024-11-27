#ifndef CAM_PUBLISHER_NODE_WITH_MQTT_HPP
#define CAM_PUBLISHER_NODE_WITH_MQTT_HPP

#include "rclcpp/rclcpp.hpp"

// Include nav_msgs for Odometry
#include "nav_msgs/msg/odometry.hpp"

// Include AccelWithCovarianceStamped message
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"

// Include vehicle info utility
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

// MQTT
#include <mosquitto.h>

// JSON library for message formatting
#include <nlohmann/json.hpp>

// Standard Libraries
#include <string>

namespace cam_publisher_with_mqtt
{

class CamPublisherNodeWithMQTT : public rclcpp::Node
{
public:
  explicit CamPublisherNodeWithMQTT(const rclcpp::NodeOptions & options);
  ~CamPublisherNodeWithMQTT();

private:
  // Subscriber callbacks
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void accelerationCallback(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg);

  // Timer callback to publish data
  void publishData();

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr acceleration_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Stored data
  nav_msgs::msg::Odometry::SharedPtr odometry_;
  geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr acceleration_data_;

  // Vehicle info
  autoware::vehicle_info_utils::VehicleInfoUtils vehicle_info_utils_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  // MQTT client
  mosquitto *mosq_;
  std::string mqtt_host_;
  int mqtt_port_;
  std::string mqtt_topic_;
  std::string mqtt_client_id_;

  // Initialize and cleanup MQTT client
  void init_mqtt();
  void cleanup_mqtt();

  // Additional members
  int cab_id_;

  // Coordinate conversion
  double ref_lat_, ref_lon_, ref_alt_;
  int utm_zone_;
  bool utm_northp_;
  double ref_easting_, ref_northing_;

  // Current position in latitude and longitude (degrees)
  double current_latitude_deg_;
  double current_longitude_deg_;

  // Dummy variables for parameters not available from Autoware
  int component_temperatures_;          // Component temperatures, integer, °C
  float battery_soc_;                   // Battery state of charge (SoC), float, percent
  float battery_voltage_;               // Battery voltage, float, V
  int battery_discharge_current_;       // Battery discharge current, integer, A
  float battery_charge_current_;        // Battery charge current, float, A
  float motor_current_;                 // Motor current, float, A
  float current_discharge_power_;       // Current discharge power, float, W
  float current_charge_power_;          // Current charge power, float, W
  int odometer_;                        // Odometer reading, integer, km
  float cumulative_charge_energy_;      // Cumulative charge energy, float, kWh
  int current_load_;                    // Current load (number of passengers), integer
  int ad_general_status_;               // Autonomous driving general status, integer
  int ad_coupling_status_;              // Autonomous driving coupling status, integer
};

}  // namespace cam_publisher_with_mqtt

#endif  // CAM_PUBLISHER_NODE_WITH_MQTT_HPP
