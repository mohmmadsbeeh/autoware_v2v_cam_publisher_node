#include "cam_publisher_node_withMQTT.hpp"

// Include necessary headers
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include <GeographicLib/UTMUPS.hpp>

#include <cmath>
#include <chrono>
#include <string>

namespace cam_publisher_with_mqtt
{

CamPublisherNodeWithMQTT::CamPublisherNodeWithMQTT(const rclcpp::NodeOptions & options)
: Node("cam_publisher_node_withMQTT", options),
  vehicle_info_utils_(*this),
  mqtt_host_(this->declare_parameter<std::string>("mqtt_host", "localhost")),
  mqtt_port_(this->declare_parameter<int>("mqtt_port", 1883)),
  mqtt_topic_(this->declare_parameter<std::string>("mqtt_topic", "/nemo/cab1/telemetry")),
  mqtt_client_id_(this->declare_parameter<std::string>("mqtt_client_id", "cam_publisher_mqtt")),
  cab_id_(this->declare_parameter<int>("cab_id", 1)),
  ref_lat_(this->declare_parameter<double>("reference_latitude", 0.0)),
  ref_lon_(this->declare_parameter<double>("reference_longitude", 0.0)),
  ref_alt_(this->declare_parameter<double>("reference_altitude", 0.0))
{
  RCLCPP_INFO(this->get_logger(), "Initializing CAM Publisher Node with MQTT...");

  // Retrieve vehicle information
  vehicle_info_ = vehicle_info_utils_.getVehicleInfo();

  // Convert reference geographic coordinates to UTM
  GeographicLib::UTMUPS::Forward(ref_lat_, ref_lon_, utm_zone_, utm_northp_, ref_easting_, ref_northing_);

  // Initialize subscribers
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", 10,
    std::bind(&CamPublisherNodeWithMQTT::odometryCallback, this, std::placeholders::_1));

  acceleration_sub_ = this->create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
    "/localization/acceleration", 10,
    std::bind(&CamPublisherNodeWithMQTT::accelerationCallback, this, std::placeholders::_1));

  // Initialize timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&CamPublisherNodeWithMQTT::publishData, this));

  // Initialize MQTT client
  init_mqtt();

  // Initialize dummy variables for parameters not available from Autoware
  component_temperatures_ = 25;        // Dummy value in °C
  battery_soc_ = 80.0f;                // Dummy value in percent
  battery_voltage_ = 48.0f;            // Dummy value in volts
  battery_discharge_current_ = 10;     // Dummy value in A
  battery_charge_current_ = 5.0f;      // Dummy value in A
  motor_current_ = 15.0f;              // Dummy value in A
  current_discharge_power_ = 1000.0f;  // Dummy value in W
  current_charge_power_ = 500.0f;      // Dummy value in W
  odometer_ = 12345;                   // Dummy value in km
  cumulative_charge_energy_ = 50.0f;   // Dummy value in kWh
  current_load_ = 2;                   // Dummy value in number of passengers
  ad_general_status_ = 0;              // Dummy status
  ad_coupling_status_ = 0;             // Dummy status

  RCLCPP_INFO(this->get_logger(), "CAM Publisher Node with MQTT Initialized.");
}

CamPublisherNodeWithMQTT::~CamPublisherNodeWithMQTT()
{
  cleanup_mqtt();
}

// Initialize MQTT client
void CamPublisherNodeWithMQTT::init_mqtt()
{
  mosquitto_lib_init();

  mosq_ = mosquitto_new(mqtt_client_id_.c_str(), true, nullptr);
  if (!mosq_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create Mosquitto client");
    return;
  }

  int ret = mosquitto_connect(mosq_, mqtt_host_.c_str(), mqtt_port_, 60);
  if (ret != MOSQ_ERR_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to MQTT broker: %s", mosquitto_strerror(ret));
    mosquitto_destroy(mosq_);
    mosq_ = nullptr;
  } else {
    RCLCPP_INFO(this->get_logger(), "Connected to MQTT broker at %s:%d", mqtt_host_.c_str(), mqtt_port_);

    // Start the Mosquitto network handling loop
    int loop_ret = mosquitto_loop_start(mosq_);
    if (loop_ret != MOSQ_ERR_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start Mosquitto loop: %s", mosquitto_strerror(loop_ret));
      mosquitto_disconnect(mosq_);
      mosquitto_destroy(mosq_);
      mosq_ = nullptr;
    }
  }
}

void CamPublisherNodeWithMQTT::cleanup_mqtt()
{
  if (mosq_) {
    // Stop the Mosquitto network handling loop
    mosquitto_loop_stop(mosq_, true);
    mosquitto_disconnect(mosq_);
    mosquitto_destroy(mosq_);
    mosq_ = nullptr;
  }
  mosquitto_lib_cleanup();
}

// Callback implementations
void CamPublisherNodeWithMQTT::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odometry_ = msg;
}

void CamPublisherNodeWithMQTT::accelerationCallback(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg)
{
  acceleration_data_ = msg;
}

void CamPublisherNodeWithMQTT::publishData()
{
  if (!odometry_) {
    RCLCPP_WARN(this->get_logger(), "Waiting for odometry data to publish");
    return;
  }

  if (!acceleration_data_) {
    RCLCPP_WARN(this->get_logger(), "Waiting for acceleration data to publish");
    return;
  }

  // Position: Latitude and Longitude in decimal degrees
  double x = odometry_->pose.pose.position.x;
  double y = odometry_->pose.pose.position.y;

  // Compute absolute UTM coordinates
  double abs_easting = ref_easting_ + x;
  double abs_northing = ref_northing_ + y;

  // Convert UTM coordinates back to geographic coordinates
  double latitude, longitude;
  GeographicLib::UTMUPS::Reverse(utm_zone_, utm_northp_, abs_easting, abs_northing, latitude, longitude);

  // Heading: in degrees deviation from North
  double heading_rad = tf2::getYaw(odometry_->pose.pose.orientation);
  double heading_deg = heading_rad * 180.0 / M_PI;
  if (heading_deg < 0)
    heading_deg += 360.0;

  // Speed: in km/h
  double speed_mps = odometry_->twist.twist.linear.x;
  double speed_kmh = speed_mps * 3.6;

  // Acceleration: x and y from acceleration data
  float acceleration_x = acceleration_data_->accel.accel.linear.x;
  float acceleration_y = acceleration_data_->accel.accel.linear.y;

  // Create JSON object
  nlohmann::json data_json;

  data_json["position"] = {
    {"latitude", latitude},
    {"longitude", longitude}
  };

  data_json["heading"] = heading_deg;

  data_json["speed"] = speed_kmh;

  data_json["cab_id"] = cab_id_;

  data_json["acceleration"] = {
    {"x", acceleration_x},
    {"y", acceleration_y}
  };

  data_json["component_temperatures"] = component_temperatures_;

  data_json["battery_soc"] = battery_soc_;

  data_json["battery_voltage"] = battery_voltage_;

  data_json["battery_discharge_current"] = battery_discharge_current_;

  data_json["battery_charge_current"] = battery_charge_current_;

  data_json["motor_current"] = motor_current_;

  data_json["current_discharge_power"] = current_discharge_power_;

  data_json["current_charge_power"] = current_charge_power_;

  data_json["odometer"] = odometer_;

  data_json["cumulative_charge_energy"] = cumulative_charge_energy_;

  data_json["current_load"] = current_load_;

  data_json["ad_general_status"] = ad_general_status_;

  data_json["ad_coupling_status"] = ad_coupling_status_;

  // Serialize JSON to string
  std::string payload = data_json.dump();

  // Publish JSON payload to the MQTT topic
  if (mosq_) {
    int ret = mosquitto_publish(mosq_, nullptr, mqtt_topic_.c_str(), payload.size(), payload.c_str(), 0, false);
    if (ret != MOSQ_ERR_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to publish MQTT message: %s", mosquitto_strerror(ret));
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Published data over MQTT.");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "MQTT client not initialized.");
  }
}

}  // namespace cam_publisher_with_mqtt

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cam_publisher_with_mqtt::CamPublisherNodeWithMQTT>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
