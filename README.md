# Introduction to the Autoware V2X CAM Publisher Node

This repository contains the Autoware V2X CAM Publisher Node, a ROS 2 package designed to enable vehicle-to-everything (V2X) communication using Cooperative Awareness Messages (CAM) as specified by the ETSI ITS standards. The package integrates with the Autoware autonomous driving stack and facilitates the broadcasting of vehicle status information over UDP, adhering to the ETSI ITS message format.

## Key Features

- **ETSI ITS Message Support**: Implements the ETSI ITS CAM message structures for interoperability with other ITS stations.
- **UDP Communication**: Utilizes the udp_driver package to send CAM messages over UDP to specified IP addresses and ports.
- **ROS 2 Integration**: Designed to work within the ROS 2 ecosystem, leveraging standard ROS 2 messages and communication patterns.
- **Custom Modifications**: Includes custom modifications to the udp_driver package to meet specific communication requirements.

## Repository Contents

- **Source Code**: The main implementation of the CAM publisher node, which reads vehicle status data and constructs CAM messages.
- **Modified UDP Driver**: A customized version of the udp_driver package from the transport_drivers repository, adapted to support the specific needs of the V2X communication.
- **Launch Files**: Provides launch files to easily start the CAM publisher node along with the modified UDP driver.
- **Configuration Files**: Includes parameter files for configuring IP addresses, ports, and other communication settings.

## Getting Started

To build and run the Autoware V2X CAM Publisher Node:

1. **Clone the Repository**: Clone this repository into your ROS 2 workspace.
2. **Clone Dependencies**:
   - Clone the `etsi_its_messages` repository to provide the ETSI ITS message definitions.
   - Clone the `transport_drivers` repository to include the `udp_driver` package.
3. **Replace UDP Driver Files**: Replace specific files in the `udp_driver` package with the custom versions provided in this repository to ensure compatibility with the CAM publisher node.
4. **Build the Workspace**: Use `colcon build` to build the workspace, ensuring that all dependencies are correctly resolved.
5. **Run the Nodes**: Utilize the provided launch files to start the CAM publisher node and the UDP driver, establishing communication with other ITS stations.

### Via Dockerfile:

To build the image from the Dockerfile:

```bash
docker build -t autoware_v2x_with_etsi:latest .
```

Run the Container from the Docker Compose File, To start the container, use the following command:

```bash
docker-compose up
```


## Usage

The CAM publisher node collects vehicle data from the Autoware stack, constructs ETSI ITS CAM messages, and sends them over UDP to configured destinations. This enables other ITS-compliant systems to receive and process the vehicle's status information, facilitating cooperative awareness in traffic environments.

## Contributing

Contributions are welcome! If you have suggestions for improvements or encounter any issues, please open an issue or submit a pull request.
