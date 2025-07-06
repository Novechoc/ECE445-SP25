# Long-horizon Task Completion with Robotic Arms by Human Instructions

This repository contains the code and documentation for the ECE 445 Senior Design project, "Long-horizon Task Completion with Robotic Arms by Human Instructions." The project focuses on developing a robotic system to assist individuals with limited mobility by understanding high-level human instructions and autonomously performing complex, multi-step tasks.

## Abstract

This project developed a robotic system to assist individuals with limited mobility by understanding human instructions and autonomously performing long-horizon tasks like table cleaning. The system integrates Perception (Vision Language Models like Qwen-VL, Grounded SAM), Planning (VLMs for task/motion planning), Control (Raspberry Pi 5, ROS 2), and Action (custom force-feedback gripper) modules. Key achievements include successful module integration, advanced AI implementation (90% object identification accuracy), and custom hardware development (force-sensing PCB, lead screw gripper). This work demonstrates a viable approach for creating intelligent robotic assistants capable of complex instruction interpretation and real-world interaction, advancing autonomous and helpful robotics.

## Key Features

- **Advanced AI Integration:** Utilizes state-of-the-art Vision Language Models (Qwen-VL, Grounded SAM) for robust scene understanding, object identification, and task planning.
- **High-Level Instruction Following:** Capable of interpreting general human commands (e.g., "Put everything into the box") and decomposing them into a sequence of executable actions.
- **Custom Force-Feedback Gripper:** A custom-designed and 3D-printed gripper with a lead screw mechanism and an integrated FSR402 force sensor allows for precise and safe object manipulation.
- **Modular ROS 2 Architecture:** The system is built on ROS 2, running on a Raspberry Pi 5, ensuring real-time, reliable communication between all hardware and software components.
- **End-to-End Task Execution:** Demonstrates a complete pipeline from visual perception and planning to physical action, enabling the robot to perform long-horizon tasks in a dynamic environment.

## System Architecture

The system is composed of four primary modules:

1.  **Perception Module:** Uses a 4K camera and Vision Language Models to identify and locate objects in the robot's workspace. It generates a scene description and provides the 3D coordinates of objects.
2.  **Planning Module:** Takes the user's instruction and the scene description to generate a feasible, multi-step plan. It uses a VLM to reason about the physical constraints and task sequence.
3.  **Control Module:** The central hub running on a Raspberry Pi 5. It uses ROS 2 to manage communication and synchronize actions between the perception and planning servers, the UR3e arm controller, and the gripper's motor and sensor.
4.  **Action Module:** Consists of the UR3e robotic arm and the custom-built gripper. It executes the motion commands and uses force feedback to confirm successful grasps.

## Hardware and Software

### Hardware
- **Robotic Arm:** Universal Robots UR3e
- **Central Controller:** Raspberry Pi 5
- **Visual Sensor:** 4K Raspberry Pi Camera
- **Gripper:** Custom 3D-printed design with a stepper motor and lead screw.
- **Force Sensor:** FSR402 with a custom PCB for signal processing and amplification.

### Software
- **OS:** Ubuntu 24.04 LTS
- **Robotics Framework:** ROS 2
- **AI Models:** Qwen-VL, Grounded SAM
- **API Framework:** FastAPI with Uvicorn for serving the AI models.
- **Key Python Libraries:** `rclpy`, `gpiozero`, `serial`, `opencv-python`.

## Setup and Installation

1.  **Hardware Setup:**
    - Connect the UR3e arm, Raspberry Pi 5, and camera via Ethernet and CSI, respectively.
    - Attach the custom gripper to the UR3e's end-effector flange.
    - Connect the gripper's stepper motor controller and FSR sensor PCB to the Raspberry Pi 5's UART and GPIO pins.

2.  **Software Environment:**
    - Install Ubuntu 24.04 and ROS 2 on the Raspberry Pi 5.
    - Set up a separate server (or use a powerful local machine) to host the Perception (Grounded SAM) and Planning (Qwen-VL) models, exposing them via a FastAPI endpoint.
    - Clone this repository to your Raspberry Pi.

3.  **Install Dependencies:**
    - Install the required system packages and Python libraries.
    ```bash
    pip install -r requirements.txt
    ```

4.  **Build and Source ROS 2 Packages:**
    - Navigate to your ROS 2 workspace.
    - Build the custom packages (`fsr_sensor`, `gripper_control`):
    ```bash
    colcon build
    ```
    - Source the workspace to make the packages available:
    ```bash
    source install/setup.bash
    ```

## Usage

1.  **Launch the System:**
    - Start the core control nodes for the gripper and force sensor using the provided launch file:
    ```bash
    ros2 launch gripper_control combined_launch.py
    ```
2.  **Run the Main Application:**
    - Execute the main script that orchestrates the perception, planning, and action loop.
3.  **Provide Instructions:**
    - Give a high-level command to the system through the designated user interface. The robot will then capture the scene, generate a plan for approval, and execute the task.

## Future Work

- Enhance robustness in cluttered and unfamiliar environments.
- Improve handling of object occlusions and reflective surfaces.
- Scale the system to handle more complex and nuanced human instructions.
- Refine human-robot interaction to better manage ambiguity and implicit intent.
