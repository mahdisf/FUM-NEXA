# NEXA-Datalogging

CAN_GathersV2 is a C++ program that gathers sensor data from various sources using the Controller Area Network (CAN) protocol. It retrieves data from motor encoders, an MPU9250 IMU sensor, and other sources and decodes the data for further processing. The program is designed to run on a Linux system.

## Prerequisites

Before running the program, make sure you have the following prerequisites installed on your system:

- Linux operating system
- GCC (GNU Compiler Collection)
- CAN utilities (SocketCAN)

## Setup

1. Clone the repository:

   ```
   git clone <repository_url>
   ```

2. Compile the code:

   ```
   g++ -o CAN_GathersV2 CAN_GathersV2.cpp -std=c++11 -lpthread
   ```

## Usage

1. Connect the CAN interface to your Linux system.

2. Configure the CAN interface using the `ip` command. For example:

   ```
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set up can0
   ```

   Adjust the bitrate according to your specific setup.

3. Run the program:

   ```
   sudo ./CAN_GathersV2
   ```

   Note: Running the program requires root privileges to access the CAN interface.

4. The program will start gathering sensor data from the connected devices. The decoded data will be printed to the console.

5. Press Ctrl+C to stop the program.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments
- [SocketCAN](https://wiki.wireshark.org/SocketCAN) - CAN utilities for Linux
- MPU9250 register addresses and decoding logic adapted from [MPU-9250 Register Map and Descriptions](https://www.invensense.com/wp-content/uploads/2015/02/MPU-9250-Register-Map.pdf)
