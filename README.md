# 🛬 px4_vision_landing_staticTag - Precise UAV Landing with Vision

[![Download Latest Release](https://img.shields.io/badge/Download-Latest%20Release-blue?style=for-the-badge)](https://github.com/kapil3sharma/px4_vision_landing_staticTag/releases)

---

## 📘 About this Application

px4_vision_landing_staticTag helps you land drones accurately using a camera and special markers called AprilTags. This program works with PX4 drones and uses ROS 2 software to control them. It helps the drone detect a landing spot visually and land precisely by analyzing the drone’s position and speed. This tool also measures delays to improve landing accuracy.

You don’t need to be a tech expert to use this. This guide will walk you through getting the app on your computer and running it comfortably.

---

## 💻 System Requirements

To run px4_vision_landing_staticTag, make sure your computer meets the following:

- Operating System: Windows 10 or later / macOS 10.15 or later / Ubuntu 20.04 or later  
- Processor: 64-bit Intel or AMD processor, 2 GHz or faster  
- RAM: At least 8 GB  
- Storage: Minimum 1 GB free space  
- Graphics: Compatible with OpenGL 3.3 or newer  
- Additional: Webcam or USB camera connected for vision input  
- Software: ROS 2 installed (instructions included below)  

You will need a PX4 drone-compatible setup to use the full features, including a drone that supports offboard control.

---

## 🚀 Getting Started

Follow these steps carefully. You don’t need any coding skills.

### 1. Prepare Your Computer

- Check that your computer meets the requirements above.  
- Connect a compatible camera (USB webcam works fine).  
- Have your PX4 drone ready with the required firmware installed.

### 2. Install ROS 2

px4_vision_landing_staticTag uses ROS 2 for communication and control. Installing ROS 2 is straightforward:

- Visit [ROS 2 Installation Guide](https://docs.ros.org/en/foxy/Installation.html) (choose version Foxy or later).  
- Select your operating system and follow the detailed steps.  
- Verify ROS 2 is installed by opening a terminal (Command Prompt on Windows, Terminal app on macOS or Linux) and typing:
  
  ```
  ros2 --help
  ```

You should see a help list. If not, recheck the installation steps.

---

## 📦 Download & Install

You need to visit the release page to get the app files.

[![Download Latest Release](https://img.shields.io/badge/Download-Latest%20Release-blue?style=for-the-badge)](https://github.com/kapil3sharma/px4_vision_landing_staticTag/releases)

### How to Download

1. Click the link above. It takes you to the “Releases” page on GitHub.  
2. Under the latest release, find files that match your operating system (look for Windows `.exe`, Mac `.dmg`, or Linux `.tar.gz`).  
3. Click on the file name to download it to your computer.  

### How to Install

- **Windows:** Double-click the `.exe` file and follow the on-screen installer steps.  
- **Mac:** Open the `.dmg` file, then drag the app to the Applications folder.  
- **Linux:** Extract the `.tar.gz` file. Open a terminal in the extracted folder and read the included README for running instructions.  

---

## 🎯 Running the Application

Once installed, follow these steps:

1. Open the application from your Start menu, Applications folder, or terminal.  
2. Connect your camera if not already connected.  
3. Make sure your PX4 drone is powered on and ready.  
4. Follow on-screen prompts to authenticate connection with the drone.  
5. Use the interface to start vision tracking. The app detects AprilTags in view and guides the drone to land precisely.  
6. The app will display the drone’s current position, estimated time to land, and pose analysis data.  

The interface is designed to be clear and simple. You can close the app anytime to stop the process.

---

## 🔍 Understanding Key Features

- **AprilTag Detection:** The app finds special markers on the landing platform using your camera.  
- **Precision Landing:** Guides the drone smoothly and accurately onto the marker.  
- **Pose Analysis:** Measures and reports how well the drone estimates its position during landing.  
- **Kalman Filter Delay Evaluation:** Checks timing accuracy to improve future landings.  
- **ROS 2 Integration:** Uses ROS 2 messaging for real-time control and feedback.  

---

## 🛠 Troubleshooting

If you face issues, try these common fixes:

- **Camera not detected:** Check your camera is connected and drivers are installed. Restart the app after reconnecting.  
- **Drone connection failed:** Make sure your PX4 drone is powered on and in the correct mode (offboard). Check USB or telemetry links.  
- **App won’t start:** Verify ROS 2 is installed properly. Restart your computer then try again.  
- **AprilTags not recognized:** Ensure the camera is pointing clearly at the landing marker with good lighting.  

If problems persist, check the GitHub “Issues” page or contact your drone support team.

---

## ⚙ Configuring Advanced Settings

You can tweak settings to fit your needs:

- Adjust camera input resolution for better detection speed vs. quality.  
- Change threshold values for AprilTag detection to improve reliability in different environments.  
- Set Kalman filter parameters for estimating drone position more accurately.  
- Configure communication ports for drone and camera connections.  

These settings are available in the preferences menu inside the app.

---

## 📖 Learn More

The project uses open tools like PX4, ROS 2, and AprilTag detection. You can visit each official site to learn about these components:

- PX4 Autopilot: https://px4.io  
- ROS 2 Robotics Platform: https://ros.org  
- AprilTag Detection: https://april.eecs.umich.edu/software/apriltag  

These resources help you understand how the app works under the hood.

---

## 🧰 Supported Devices

- PX4-based drones supporting offboard mode  
- USB webcams or similar camera devices  
- Computers running Windows, macOS, or Linux with ROS 2 installed  

Make sure your drone’s firmware is up-to-date for best compatibility.

---

## ✉ Contact & Support

For help or questions, please use the GitHub repository’s discussion or issues page:

https://github.com/kapil3sharma/px4_vision_landing_staticTag/issues

Provide details about your setup and problem to get faster support.

---

[![Download Latest Release](https://img.shields.io/badge/Download-Latest%20Release-blue?style=for-the-badge)](https://github.com/kapil3sharma/px4_vision_landing_staticTag/releases)