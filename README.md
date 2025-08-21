# BlueROV2-EKF-Localization

<div align="center">

<!-- ![BlueROV2](https://bluerobotics.com/wp-content/uploads/2016/06/BlueROV2-4-lumen-1.png)
![DVL Sensor](https://hotrobotics.co.uk/wp-content/uploads/2021/07/Waterlinked-DVL-003-800x600.jpg) -->

<div align="center">
       <img src="https://bluerobotics.com/wp-content/uploads/2016/06/BlueROV2-4-lumen-1.png" width="50%">
       <img src="https://hotrobotics.co.uk/wp-content/uploads/2021/07/Waterlinked-DVL-003-800x600.jpg" width="40%">
</div><br>

**Extended Kalman Filter (EKF) Localization for BlueROV2 Underwater Robot**

[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg)](http://wiki.ros.org/noetic)
[![License](https://img.shields.io/badge/License-TODO-yellow.svg)](LICENSE)

</div>

## 📋 Overview

This repository implements an **Extended Kalman Filter (EKF)** localization system for the BlueROV2 underwater robot. The system processes sensor data from a **Doppler Velocity Log (DVL)** to estimate the robot's position and velocity in real-time.

### Key Features
- **DVL Sensor Integration**: Subscribes to [DVL sensor values](https://waterlinked.github.io/docs/dvl/dvl-protocol/)
- **EKF Implementation**: Advanced state estimation using Extended Kalman Filter
- **ROS Integration**: Publishes navigation data as [nav_msgs/Odometry](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html)
- **Real-time Processing**: Continuous position and velocity estimation

## 🏗️ Project Structure

```
BlueROV2-EKF-Localization/
├── src/
│   └── bluerov2_ekf_localization/     # ROS Package
│       ├── launch/                     # Launch files
│       ├── msg/                        # Custom message definitions
│       ├── scripts/                    # Python nodes
│       ├── CMakeLists.txt              # Build configuration
│       └── package.xml                 # Package metadata
├── img/                               # Project images
└── README.md                          # This file
```

## 🚀 Quick Start

### Prerequisites
- **ROS Noetic** on Ubuntu 20.04
- Python 3.x
- Required ROS packages: `roscpp`, `rospy`, `std_msgs`, `geometry_msgs`

### Installation

1. **Install ROS Noetic** (if not already installed):
   ```bash
   # Follow the official installation guide
   http://wiki.ros.org/noetic/Installation/Ubuntu
   ```

2. **Create ROS Workspace**:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/
   catkin_make
   source devel/setup.bash
   ```

3. **Clone and Build**:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/your-username/BlueROV2-EKF-Localization.git
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

4. **Launch the System**:
   ```bash
   roslaunch bluerov2_ekf_localization bluerov_navigation.launch
   ```

## 📚 Documentation

### Navigation Theory
- [Introduction to Subsea Navigation](https://www.nortekgroup.com/knowledge-center/wiki/new-to-subsea-navigation)

### Kalman Filter Resources
- **Theory**: [Kalman Filter Tutorial](https://www.kalmanfilter.net/default.aspx)
- **Implementation**: [FilterPy Documentation](https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html)

### ROS Resources
- [ROS Noetic Installation](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [ROS Workspace Tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

## 🔧 Usage

### Running the Localization System
```bash
# Terminal 1: Launch the main system
roslaunch bluerov2_ekf_localization bluerov_navigation.launch

# Terminal 2: Monitor DVL data
rostopic echo /dvl_data

# Terminal 3: Monitor estimated position
rostopic echo /odometry/filtered
```

### Available Topics
- **Input**: `/dvl_data` - DVL sensor measurements
- **Output**: `/odometry/filtered` - Estimated position and velocity

## 🎓 Academic Information

**Course**: CA-RIS-801 | Marine Robotics (Spring 2021)  
**Institution**: Jacobs University Bremen (JUB)  
**Instructors**: 
- Dr. Evelina Dineva
- Prof. Dr. Francesco Maurelli

**Contributors**:
- Dongwook Lee (do.lee@jacobs-university.de)
- Katrin von Seggern (K.vonSeggern@jacobs-university.de)

## 📄 License

This project is licensed under the TODO License - see the [LICENSE](LICENSE) file for details.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📞 Contact

- **Maintainer**: dongwooklee (dongwooklee1201@gmail.com)
- **Project Link**: [https://github.com/your-username/BlueROV2-EKF-Localization](https://github.com/your-username/BlueROV2-EKF-Localization)