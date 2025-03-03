# CMPUT 412 Exercise 3: Computer Vision and Control

This repository contains the implementation for CMPUT 412 Exercise 3, which focuses on computer vision, controls, and lane following for Duckiebots.

## Overview

The exercise is divided into three main parts:

1. **Computer Vision**: Implementation of camera undistortion, color detection, and lane detection.
2. **Controllers**: Implementation of P, PD, and PID controllers for lane following.
3. **Lane Following**: Integration of computer vision and controllers to follow lanes autonomously.

## Requirements

- Duckiebot with camera
- Camera intrinsic calibration file (required for undistortion)
- Wheel calibration (for accurate movement)

## Project Structure

```
packages/
└── computer_vision/
    ├── src/
    │   ├── lane_detection.py           # Camera undistortion and color/lane detection
    │   ├── navigation_control.py       # Robot movement control
    │   ├── color_line_handler.py       # Handles colored line behaviors
    │   ├── lane_following_controller.py # P, PD, PID controllers for lane following
    │   └── lane_following.py           # Main integration node
    ├── launch/
    │   ├── lane_following.launch       # Launch file for all nodes
    │   ├── color_line_detection.launch # Launch file for color line detection
    │   ├── p_controller.launch         # Launch file for P controller
    │   ├── pd_controller.launch        # Launch file for PD controller
    │   └── pid_controller.launch       # Launch file for PID controller
    ├── CMakeLists.txt                  # Build configuration
    └── package.xml                     # Package metadata
launchers/
    ├── default.sh                      # Default launcher script
    ├── color_line_detection.sh         # Launcher for color line detection
    ├── p_controller.sh                 # Launcher for P controller
    ├── pd_controller.sh                # Launcher for PD controller
    ├── pid_controller.sh               # Launcher for PID controller
    └── lane_following.sh               # Launcher for full lane following
```

## How to Run

### Prerequisites

1. Make sure your Duckiebot is charged
2. Calibrate the wheels (follow the Duckietown documentation)
3. Adjust camera focus if needed
4. Perform camera intrinsic calibration (save the file to `~/camera_intrinsic.yaml`)

### Running the Code

1. Clone this repository to your workspace

2. Build the packages:

   ```bash
   dts devel build -f
   ```

3. Run specific parts of the exercise using the launcher scripts:

   **Part 1: Computer Vision and Color-Based Behaviors**

   ```bash
   dts duckiebot demo --demo_name color_line_detection --duckiebot_name <ROBOT_NAME>
   ```

   **Part 2: Controllers for Straight Lane Following**

   For P controller:

   ```bash
   dts duckiebot demo --demo_name p_controller --duckiebot_name <ROBOT_NAME>
   ```

   For PD controller:

   ```bash
   dts duckiebot demo --demo_name pd_controller --duckiebot_name <ROBOT_NAME>
   ```

   For PID controller:

   ```bash
   dts duckiebot demo --demo_name pid_controller --duckiebot_name <ROBOT_NAME>
   ```

   **Part 3: Full Lane Following**

   ```bash
   dts duckiebot demo --demo_name lane_following --duckiebot_name <ROBOT_NAME>
   ```

### Running Individual Nodes

If you need to run individual nodes for testing, you can use:

```bash
# For lane detection node
dts duckiebot demo --demo_name default --duckiebot_name <ROBOT_NAME> --cmd "rosrun computer_vision lane_detection.py"

# For color line handler
dts duckiebot demo --demo_name default --duckiebot_name <ROBOT_NAME> --cmd "rosrun computer_vision color_line_handler.py"

# For navigation control
dts duckiebot demo --demo_name default --duckiebot_name <ROBOT_NAME> --cmd "rosrun computer_vision navigation_control.py"
```

## Controllers

Three controllers are implemented for lane following:

1. **P Controller**: Simple proportional control, works well for small errors but may oscillate.
2. **PD Controller**: Adds derivative term to reduce oscillations, smoother response.
3. **PID Controller**: Adds integral term to eliminate steady-state error, best for consistent tracking.

## Tuning Parameters

Several parameters can be tuned for optimal performance:

- Controller gains (`Kp`, `Kd`, `Ki`) in `lane_following_controller.py`
- HSV color thresholds in `lane_detection.py`
- Movement parameters in `navigation_control.py`

## Troubleshooting

- **Camera not connecting**: Check camera connections and permissions
- **Lane detection issues**: Adjust HSV thresholds for your lighting conditions
- **Movement problems**: Check wheel calibration and adjust speed parameters
- **Controller oscillations**: Tune controller gains for smoother performance

## Contributors

- [Your Name]

## License

This project is licensed under the MIT License - see the LICENSE file for details.
