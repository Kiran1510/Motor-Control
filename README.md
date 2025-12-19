# Motor Control System

A precision motor control system using Arduino for angle control with encoder feedback, developed as part of a self-balancing cube project.

## Overview

This project implements a Bang-Bang based control system for angular positioning using a Nidec 24H motor with integrated encoder feedback. The system is designed for applications requiring accurate motor positioning and velocity control.

## Hardware Components

### Electronics
- **Motor**: Nidec 24H brushless motor with encoder
- **Microcontroller**: Arduino-compatible board
- **Custom PCB**: KiCAD design included for motor driver interface

### Specifications
- Encoder resolution: For every 360 degrees the motor revolves, the motor shows 200 step counts.
- Control frequency: None, since this is an adaptive Bang-Bang type controller.
- Input voltage: 8~12V DC (8V is recommended for better accuracy as the speed of the motor seems to be directly proportional to the voltage)

## Project Structure
```
Motor-Control/
├── Firmware/
│   └── angle_control_nidec.ino    # Main Arduino control code
├── Hardware/
│   ├── Self Balancing Cube.kicad_pcb
│   ├── Self Balancing Cube.kicad_sch
│   └── Self Balancing Cube.kicad_pro
├── Docs/
│   └── KiCAD schematic.png        # Circuit schematic
└── README.md
```

## Features

- **PID Control**: Tunable PID parameters for precise angle control
- **Encoder Feedback**: Real-time position tracking
- **Serial Interface**: Monitor and adjust parameters via serial communication
- **Adaptive Control**: [Add if you implemented adaptive bang-bang or other features]

## Setup Instructions

### Hardware Setup

1. Flash the Arduino with `angle_control_nidec.ino`
2. Connect the Nidec motor according to the schematic in `/Docs`
3. Ensure proper power supply connections
4. Connect encoder outputs to specified Arduino pins

### Software Requirements

- Arduino IDE (version 1.8.x or higher)
- Libraries listed in the code

### Installation
```bash
# Clone the repository
git clone https://github.com/Kiran1510/Motor-Control.git

# Open the Arduino sketch
cd Motor-Control/Firmware
# Open angle_control_nidec.ino in Arduino IDE
```

## Usage

1. Upload the firmware to your Arduino board
2. Open Serial Monitor (9600 baud)
3. Send commands to control motor position. The commands are listed in the serial monitor.

## Control Parameters

I've used a Bang-Bang type controller for running this motor since it is an industrial type motor that isn't designed for precise angle outputs. The motor lacks proper documentation or datasheets, hence all of the knowledge 
I've learned from it is by trial and error. The pinouts for the motor and their functions is included in the arduino code.

## PCB Design

The custom PCB design files are available in the `/Hardware` directory. The board includes:
- Motor driver interface
- Encoder signal conditioning
- Power regulation

View the KiCAD files directly on GitHub for 2D/3D visualization of the PCB design.

## Troubleshooting

**Motor overshooting target position:**
- Make sure voltage is 8V minimum and increase voltage to 12V if no improvement in performance
- Check encoder connections

**Encoder reading errors:**
- Verify encoder power supply
- Check signal wire integrity
- Ensure proper pull-up resistors

## Future Improvements

- [ ] Implement velocity control
- [ ] Implement PID Control and compare with this current approach
- [ ] Optimize control loop timing
- [ ] Add temperature monitoring

## Project Context

This motor control system was developed as a personal project and part of a self-balancing cube project for my personal interest in controls systems engineering. The system demonstrates precision control techniques applicable to robotics and mechatronics applications.

## Author

**Kiran Sairam**
- MS Robotics, Northeastern University

## License

MIT License
