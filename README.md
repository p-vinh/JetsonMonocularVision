# JetsonMonocularVision

JetsonMonocularVision is a Python-based computer vision project designed for NVIDIA Jetson platforms. It utilizes a single Sony camera to detect objects and compute their distance and GPS location. The system employs triangulation techniques to determine the distance from the drone to the object and then calculates the object's GPS coordinates using the drone's GPS data.

## Features

* **Object Detection**: Identifies objects using a pre-trained model.
* **Distance Estimation**: Calculates the distance to detected objects using triangulation.
* **GPS Localization**: Determines the GPS coordinates of objects based on drone telemetry.
* **Real-Time Processing**: Optimized for real-time performance on Jetson devices.

## Requirements

* NVIDIA Jetson device (e.g., Jetson Nano, TX2, Xavier)
* Python 3.6+
* OpenCV
* NumPy
* PyMAVLink
* Other dependencies as specified in `requirements.txt`([GitHub][1], [GitHub][2], [ResearchGate][3], [GitHub][4])

## Installation

1. **Clone the Repository**:

   ```bash
   git clone https://github.com/p-vinh/JetsonMonocularVision.git
   cd JetsonMonocularVision
   ```



2. **Set Up Virtual Environment**:

   ```bash
   python3 -m venv jetsonvenv
   source jetsonvenv/bin/activate
   ```



3. **Install Dependencies**:

   ```bash
   pip install -r requirements.txt
   ```



## Usage

1. **Run the Main Script**:

   ```bash
   python UAV_Code.py Configs/[CONFIG].conf
   ```



This script initializes the cameras, performs object detection, computes distances using triangulation, and calculates GPS coordinates.

## Project Structure

* `Detector.py`: Handles object detection logic.
* `DetectorThreadRunner.py`: Performs model inference and image slicing logic.
* `Triangulation.py`: Performs distance calculations using triangulation.
* `MavLink.py`: Manages communication with the drone to retrieve GPS data.
* `Network.py`: Communicates GPS coordinates with the IF750 through sockets
* `UAV_Code.py`: Main script that orchestrates the detection and localization process.
* `Configs/`: Contains configuration files.
* `Images/`: Stores sample images and outputs. Just Used for Testing
* `model/`: Includes the pre-trained object detection model.
* `UAV-UGV/HuskyScripts/`: Contains the files to run on the Husky
* `utils/`: Utility functions and helper scripts.([Amazon][5], [NVIDIA Developer Forums][6])

## Acknowledgments

This project is a fork of [JetsonStereoVision](https://github.com/dury2379/JetsonStereoVision) and has been adapted for monocular vision applications.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
