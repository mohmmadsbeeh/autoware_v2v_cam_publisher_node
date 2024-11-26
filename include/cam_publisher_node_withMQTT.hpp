#ifndef CAM_PUBLISHER_NODE_WITH_MQTT_HPP
#define CAM_PUBLISHER_NODE_WITH_MQTT_HPP

#include "rclcpp/rclcpp.hpp"

// Include nav_msgs for Odometry
#include "nav_msgs/msg/odometry.hpp"

// Include IMU message
#include "sensor_msgs/msg/imu.hpp"

// Include vehicle info utility (updated include and namespace)
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
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // Timer callback to publish data
  void publishData();

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Stored data
  nav_msgs::msg::Odometry::SharedPtr odometry_;
  sensor_msgs::msg::Imu::SharedPtr imu_data_;

  // Vehicle info (updated namespace)
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
  int component_temperatures_;          // Komponententemperaturen, integer, °C
  float battery_soc_;                   // Batteriestand (SoC), float/double, Prozent
  float battery_voltage_;               // Batteriespannung, float/double, V
  int battery_discharge_current_;       // Batterie-Entladestrom, integer, A
  float battery_charge_current_;        // Batterie-Ladestrom, float/double, A
  float motor_current_;                 // Motorstrom, float/double, A
  float current_discharge_power_;       // Aktuelle Entladeleistung, float/double, W
  float current_charge_power_;          // aktuelle Ladeleistung, float/double, W
  int odometer_;                        // Kilometerstand, integer, km
  float cumulative_charge_energy_;      // Energie Ladung kummulativ, float/double, kWh
  int current_load_;                    // aktuelle Beladung bzw. Anzahl Personen, integer, Anzahl Personen
  int ad_general_status_;               // AD Status Allgemein, integer, Stati
  int ad_coupling_status_;              // AD Koppelzustand/Vorgang, integer, Stati
};

}  // namespace cam_publisher_with_mqtt

#endif  // CAM_PUBLISHER_NODE_WITH_MQTT_HPP
