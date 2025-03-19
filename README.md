# UM982 Serial Driver for ROS 2

## Introduction

This project provides a serial driver for the UM982 GNSS receiver, designed to work with ROS 2. The driver decodes GNSS messages and publishes relevant data to ROS 2 topics.

## Prerequisites

- ROS 2 (Humble or later)
- Python 3.10 or later
- `numpy==1.21.5`, `transforms3d==0.4.1`, and `pyproj==3.2.1` Python packages

## Installation

1. Install ROS 2 by following the instructions on the [ROS 2 installation page](https://docs.ros.org/en/humble/Installation.html).
2. Clone this repository:
   ```sh
   git clone https://github.com/lostDeers/UM982Driver-ros2.git
   cd UM982Driver-ros2
   ```
3. Install the required Python packages:
   ```sh
   pip install -r requirements.txt
   ```
4. Build the package:
   ```sh
   colcon build
   ```

## Topics

The following topics are used in the `um982_serial_driver/node.py` file:

- `odometry_topic`: The topic name for publishing odometry data (default: "odom").
- `nav_sat_fix_topic`: The topic name for publishing NavSatFix data (default: "nav_sat_fix").
- `gngga_topic`: The topic name for publishing GNGGA data (default: "gngga").
- `rtcm_topic`: The topic name for subscribing to RTCM data (default: "rtcm").

## License

This project is licensed under the GNU General Public License v3.0. See the [LICENSE](um982_serial_driver/LICENSE) file for details.

## Contributing

Contributions are welcome! Please follow these steps to contribute:

1. Fork the repository.
2. Create a new branch for your feature or bugfix.
3. Commit your changes and push the branch to your fork.
4. Create a pull request with a description of your changes.

## Reporting Issues

If you encounter any issues or have feature requests, please open an issue on the [GitHub repository](https://github.com/lostDeers/UM982Driver-ros2/issues).


## Acknowledgments

The serial driver code is based on [sunshineharry/UM982Driver](https://github.com/sunshineharry/UM982Driver).

README is written by Copilot.