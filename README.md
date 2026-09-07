# Aurora — Robolutionaries FTC Robot Code

Competition robot control software for **Aurora**, built by **Robolutionaries** for the FIRST Tech Challenge.

## 🏆 Awards

Robolutionaries earned **12 FTC awards**, ranging from local qualifiers to the **World Championship level**, recognized for design and autonomous efficiency. This codebase includes the autonomous routines and vision system that helped drive that performance, including a custom **OpenCV model identifying a moving game prop with 93% accuracy** and iterative mechanism design that improved average scoring by **75%**.

## Overview

This repository contains the Java control software running on Aurora's Control and Expansion Hubs, built on the FTC SDK and **Road Runner** for autonomous motion planning. It covers field-centric teleop driving, closed-loop slide control, multiple absolute localization strategies, and fully scripted autonomous routines for scoring game elements.

## Key Systems

### Drive & Localization
- **`MecanumDrive`** — core mecanum drivetrain using Road Runner kinematics, holonomic path following, motor feedforward, and dead-wheel/drive-encoder localization
- **`AbsoluteLocalizerDrive`** — abstract base class unifying any absolute positioning sensor (Pinpoint, OTOS) behind a single pose-estimation interface, so the drivetrain doesn't need to know which sensor is in use
- **`PinpointDrive`** — localization via the goBILDA Pinpoint odometry computer, with configurable pod offsets and encoder directions
- **`SparkFunOTOSDrive`** — localization via the SparkFun Optical Tracking Odometry Sensor, with linear/angular scalar calibration and live status/error monitoring
- **`TuningOpModes`** — auto-registers Road Runner's full tuning suite (feedforward tuners, ramp loggers, localization tests) against whichever drive class is active, for fast iteration between localization strategies

### Autonomous
- **`FourSampleAuto`**, **`OdometryAuto`** — fully scripted, spline-based autonomous routines using Road Runner's `TrajectoryActionBuilder`, sequencing drivetrain motion with synchronized claw, wrist, intake, and transfer-arm actions to score multiple game elements per run
- Custom `Action` implementations (`SlideAction`, `ServoAction`, `CRServoAction`) allow slide PID control and multi-servo choreography to run concurrently with drive motion rather than blocking it

### TeleOp
- **`FieldCentricMechanum`**, **`PrawnSuitMode`** — field-centric driver control using IMU heading to decouple robot-relative input from field-relative motion, with PID-controlled linear slide positioning and gamepad-mapped control of intake, claw, wrist, and transfer arm subsystems
- Live tuning and telemetry through **FTC Dashboard**, including real-time PID gain adjustment without redeploying code

## Tech Stack

- **Language:** Java
- **Framework:** FTC SDK, Road Runner (motion planning & trajectory following)
- **Control:** PID/PIDF (FTCLib), custom `Action`-based sequencing
- **Localization:** goBILDA Pinpoint, SparkFun OTOS, drive encoders
- **Tuning/Telemetry:** FTC Dashboard

## Team

Built and maintained by **Robolutionaries**, an FTC robotics team competing from local qualifiers through the World Championship level.
