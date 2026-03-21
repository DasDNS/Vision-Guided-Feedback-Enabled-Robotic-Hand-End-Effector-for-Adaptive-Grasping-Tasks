# Vision-Guided, Feedback-Enabled Robotic Hand End Effector

This repository combines the **full system codebase** for a vision-guided robotic hand end effector used for adaptive grasping tasks.

The system brings together three main parts:

- **Vision side**: detects the object and decides the grasp pattern to request
- **Laptop/UI side**: receives messages from the vision laptop, handles user interaction, and coordinates communication
- **MCU/hand control side**: runs the grasp controller, reads tactile and current feedback, and actuates the fingers

The project is intended to keep the **combined full system** in one place so that development, testing, and version control are easier to manage across all major integration stages.

## System Overview

The robotic hand performs grasping using a combination of:

- **Vision-guided grasp initiation**
- **Finger pattern selection**
- **Tactile sensing using FSRs**
- **Motor current sensing using INA226 sensors**
- **Feedback-based grasp adjustment through an FSM-based controller**

A typical flow is:

1. The vision side identifies the object or grasp category.
2. A grasp/finger pattern is sent toward the laptop/UI side.
3. The laptop/UI relays commands to the MCU.
4. The MCU actuates the fingers.
5. Tactile and motor-current feedback are monitored during closure.
6. The controller adjusts, holds, settles, or recovers the grasp depending on sensed conditions.

## Repository Purpose

This repo is meant to store:

- the integrated **MCU firmware**
- the **laptop/UI code**
- communication logic between the laptop and vision system
- intermediate and final test versions
- documentation needed to understand the evolution of the full system

## Typical Project Layout

Your repo may look something like this:

```text
Combined_Full_System/
├── Test_1_.../
├── Test_2_.../
├── ...
├── Test_22_Demonstration/
├── MCU/
├── UI/
├── Vision/
└── README.md
```

In your current workflow, each test folder represents a development stage or milestone.

## Main Components

### 1. MCU Firmware
The MCU side is responsible for:

- servo actuation
- reading tactile sensors
- reading current sensors
- enforcing grasp logic
- running the finite state machine (FSM)

Typical embedded responsibilities include:

- finger initialization and reset
- fast/slow closing phases
- tightening
- hold and settle logic
- recovery when contact is lost or grasp quality changes

### 2. Laptop / UI Node
The laptop code acts as the integration layer between the vision side and the MCU.

It typically handles:

- serial communication with the microcontroller
- ROS 2 message exchange
- pattern request / acknowledgment flow
- manual and automatic operation modes
- operator-facing controls and status feedback

### 3. Vision Laptop Interface
The vision side decides **when to grasp** and **which finger pattern** should be used.

Depending on the implementation, it may:

- detect object presence
- classify object shape
- choose a grasp pattern
- send a command/request to the laptop/UI side

## Key Features

- Vision-guided grasp triggering
- Pattern-based finger actuation
- Tactile sensing integration
- Motor current sensing integration
- Closed-loop grasp adjustment
- FSM-based controller design
- ROS-enabled communication between nodes
- Serial communication between laptop and MCU
- Stepwise development through multiple test folders

## Development History

This repository also serves as a **development history archive**.

The multiple `Test_*` folders preserve the progression from:

- early servo motion testing
- UI-only experiments
- FSM integration
- ROS communication integration
- acknowledgment fixes
- recovery-state improvements
- auto-mode and demonstration-ready versions

That makes the repo useful not only for active development, but also for:

- debugging regressions
- thesis/project documentation
- comparing algorithm and protocol changes over time

## Status

This repository contains the combined full-system development history for the robotic hand end effector, including integration-focused test versions and demonstration-stage code.

## Author

Dasuni Saparamadu

