# Autonomous Patrol Robot with Real-Time Rule Enforcement

An intelligent autonomous robot system that patrols multiple rooms, detects rule violations using computer vision, and enforces behavioral policies through voice commands. Built with ROS, SMACH state machines, and YOLOv4 object detection.

## Overview

This system implements an autonomous patrol robot that monitors designated areas (Rooms A, B, and D) for policy violations. The robot navigates between rooms, continuously analyzes video feeds for prohibited scenarios, and responds with appropriate warnings when rules are broken.

**Detected Rule Violations:**
- **Rule 1**: Cat and dog in the same room (prevents conflicts)
- **Rule 2**: Unauthorized person in restricted area (Room D)

## Key Features

### Autonomous Navigation
- **Move Base Integration**: Uses ROS navigation stack with AMCL localization
- **Multi-Room Patrol**: Coordinates movement between three distinct patrol zones
- **Precise Positioning**: Quaternion-based orientation control for accurate room entry

### Computer Vision Pipeline
- **YOLOv4 Object Detection**: Real-time detection of cats, dogs, and persons
- **Live Video Processing**: Continuous frame analysis during patrol missions
- **Confidence Scoring**: Validates detections before triggering violations

### State Machine Architecture
- **SMACH Framework**: Hierarchical state machines for complex behavior coordination
- **Concurrent States**: Simultaneous navigation and detection using Concurrence containers
- **Action Server**: Implements ROS actionlib for mission-based operations with feedback

### Real-Time Feedback
- **Violation Tracking**: Maintains persistent count of each rule violation
- **Live Position Updates**: Publishes robot location with each detected violation
- **Text-to-Speech**: Issues verbal warnings through integrated speech synthesis
- **Action Feedback**: Streams mission progress to monitoring clients

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     Main Node                           │
│  (Action Client - Initiates patrol missions)            │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              Checkroom Action Server                     │
│  (Coordinates state machine execution)                   │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│            SMACH State Machine                           │
│  ┌───────────────────────────────────────────────┐      │
│  │  movetoA → CONA (Navigate + Detect)          │      │
│  │     ↓                                          │      │
│  │  movetoB → CONB (Navigate + Detect)          │      │
│  │     ↓                                          │      │
│  │  movetoD → COND (Navigate + Detect)          │      │
│  │     ↓                                          │      │
│  │  Loop until completion                         │      │
│  └───────────────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────┘
         │                              │
         ▼                              ▼
┌──────────────────┐         ┌──────────────────────┐
│  Navigation      │         │  YOLO Detection      │
│  - Move Base     │         │  - YOLOv4 CNN        │
│  - AMCL          │         │  - Frame Service     │
│  - Velocity Ctrl │         │  - Object Classes    │
└──────────────────┘         └──────────────────────┘
```

## Technical Components

### Navigation States
- **movetoA/B/D.py**: Service clients that request navigation to specific coordinates
- **navroomA/B/D.py**: Control robot movement within rooms using velocity commands
- **movetoroom_node.py**: Service server handling navigation requests via Move Base

### Detection System
- **yolo_ros.py**: ROS wrapper for YOLOv4 inference engine
- **yolo_state.py**: SMACH state integrating detection with patrol logic
- **YOLOlastframe.srv**: Service definition for retrieving detection results

### Coordination Layer
- **checkroom_node.py**: Main state machine orchestrating patrol behavior
- **main_node.py**: Entry point launching patrol missions with configurable parameters

## Installation

### Prerequisites
```bash
# ROS Noetic (Ubuntu 20.04) or ROS Melodic (Ubuntu 18.04)
sudo apt install ros-noetic-desktop-full
sudo apt install ros-noetic-navigation
sudo apt install ros-noetic-smach ros-noetic-smach-ros
sudo apt install ros-noetic-actionlib ros-noetic-actionlib-msgs
```

### YOLOv4 Setup
```bash
# Clone and build Darknet with GPU support
git clone https://github.com/AlexeyAB/darknet
cd darknet
# Edit Makefile: GPU=1, CUDNN=1, OPENCV=1
make

# Download YOLOv4 weights
wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights
```

### Project Setup
```bash
# Clone into your catkin workspace
cd ~/catkin_ws/src
git clone <repository-url> second_coursework

# Build the package
cd ~/catkin_ws
catkin_make

# Source the workspace
source devel/setup.bash
```

## Usage

### Launch Complete System
```bash
roslaunch second_coursework itr_cw.launch nchecks:=2
```

**Launch Parameters:**
- `nchecks`: Number of times to patrol room D before completion (default: 1)
- `vid_folder`: Directory containing video files for simulation

### What Happens

1. **Initialization**: Robot spawns in simulation environment
2. **Navigation**: Moves to Room A using path planning
3. **Patrol**: Executes 30-second circular patrol while analyzing video
4. **Detection**: YOLOv4 processes frames, identifies objects
5. **Violation Response**: If rules broken, publishes feedback and issues voice warning
6. **Iteration**: Continues to Room B, then Room D, cycling as configured
7. **Completion**: Returns violation statistics via action result

### Running Individual Nodes

```bash
# Launch navigation and move service
rosrun second_coursework movetoroom_node.py

# Start YOLO detection service
rosrun second_coursework yolo_ros.py

# Launch patrol state machine
rosrun second_coursework checkroom_node.py

# Initiate patrol mission
rosrun second_coursework main_node.py
```

## ROS Interface

### Services
- `/set_room` (movetoroom): Navigate to specified room by name
- `/detect_frame` (YOLOlastframe): Request object detections from latest camera frame

### Actions
- `checkroom`: Execute full patrol mission with real-time feedback
  - **Goal**: Number of patrol cycles
  - **Feedback**: Robot position + rule violation type
  - **Result**: Total count of each violation type

### Topics
- `/camera/image` (sensor_msgs/Image): Camera feed input
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for robot motion
- `/amcl_pose` (PoseWithCovarianceStamped): Current robot localization
- `/speech` (std_msgs/String): Text-to-speech output

## Configuration

### Room Coordinates (movetoroom_node.py)
```python
Room A: (1.7, 8.0)   # Top-left area
Room B: (6.0, 8.0)   # Top-right area
Room D: (1.7, 3.1)   # Bottom-left area
```

### Detection Parameters (yolo_state.py)
```python
PATROL_DURATION = 30  # seconds per room
CONFIDENCE_THRESHOLD = Default (YOLOv4 threshold)
```

### Patrol Behavior
- Each room receives 30 seconds of surveillance
- Concurrent execution: navigation + detection
- Violations tracked cumulatively across mission

## Project Structure

```
second_coursework/
├── action/
│   └── checkroom.action          # Action definition for patrol missions
├── srv/
│   ├── movetoroom.srv            # Room navigation service
│   └── YOLOlastframe.srv         # Detection request service
├── msg/
│   └── YOLODetection.msg         # Bounding box + class info
├── cfg/
│   └── coco.data                 # YOLOv4 configuration
├── scripts/
│   ├── main_node.py              # Mission launcher
│   ├── checkroom_node.py         # State machine controller
│   ├── movetoroom_node.py        # Navigation service server
│   ├── yolo_ros.py               # Detection service server
│   ├── yolo_state.py             # Detection state logic
│   ├── movetoA/B/D.py            # Room transition states
│   ├── navroomA/B/D.py           # In-room patrol states
│   └── set_node.py               # Test client example
├── launch/
│   └── itr_cw.launch             # Master launch file
├── CMakeLists.txt
└── package.xml
```

## Technical Highlights

- **Hierarchical State Machines**: SMACH's Concurrence containers enable parallel execution of navigation and detection tasks
- **Service-Oriented Architecture**: Modular design with separate navigation and detection services
- **Real-Time Processing**: Frame-by-frame analysis during continuous patrol motion
- **Robust Navigation**: Integration with ROS navigation stack ensures reliable path planning and obstacle avoidance
- **Actionlib Integration**: Mission-based operation with preemptable goals and detailed feedback

## Dependencies

```xml
<depend>rospy</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>actionlib</depend>
<depend>actionlib_msgs</depend>
<depend>move_base_msgs</depend>
<depend>cv_bridge</depend>
<depend>smach</depend>
<depend>smach_ros</depend>
```

**Python Packages:**
- `opencv-python`: Image processing
- `yolov4`: Darknet wrapper for object detection

## Output

The system provides:
- Real-time console logging of navigation and detection events
- RViz visualization of robot pose and navigation goals
- Spoken warnings when violations detected
- Final report with violation counts per rule type

## Future Enhancements

- Multi-robot coordination for larger patrol areas
- Integration with cloud logging for violation history
- Adaptive patrol routes based on violation hotspots
- Additional rule types and custom detection classes
- Web dashboard for remote monitoring

## License

MIT License - Free to use and modify.

