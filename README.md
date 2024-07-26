# LightFrame

## Overview

LightFrame is a cross-platform utility to capture the screen and adjust an LED strip's colors accordingly. It supports various platforms including Linux (both X11 and Wayland), macOS, and Windows. The utility reads the screen, calculates the average color of different screen segments, and sends this data over a serial connection to control an LED strip behind an ESP8266 that runs [light_frame_receiver](https://github.com/stephanballer/light_frame_receiver).

## Features

- Supports X11 and Wayland on Linux
- Supports macOS and Windows
- Configurable number of LEDs and other parameters
- Serial communication to send color data
- Automatically starts on device connection using udev rules (Linux)

## Requirements

- CMake 3.10 or higher
- OpenCV
- X11 libraries (for X11 support)
- Wayland and Grim (for Wayland support)
- Serial communication libraries

## Installation

### Linux (Ubuntu/Debian-based distributions)

```sh
sudo apt update
sudo apt install cmake libopencv-dev libx11-dev libwayland-client0 grim
```

### Arch Linux

```sh
sudo pacman -S cmake opencv xorg-server-devel wayland grim
```

### macOS

```sh
brew install cmake opencv
```

### Windows

- Download and install CMake from [cmake.org](https://cmake.org/download/)
- Install OpenCV following the [official instructions](https://docs.opencv.org/master/d3/d52/tutorial_windows_install.html)

## Building the Project

1. **Clone the Repository**:

   ```sh
   git clone https://github.com/stephanballer/light_frame.git
   cd light_frame
   ```

2. **Create a Build Directory**:

   ```sh
   mkdir build
   cd build
   ```

3. **Run CMake**:

   For X11 support:
   ```sh
   cmake -DUSE_X11=ON ..
   ```

   For Wayland support:
   ```sh
   cmake -DUSE_WAYLAND=ON ..
   ```

4. **Build the Project**:

   ```sh
   make
   ```

5. **Install the Project**:

   ```sh
   sudo make install
   ```

6. **Reload udev Rules (if on Linux)**:

   ```sh
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

## Usage

### Linux

The program will automatically run when a device matching the udev rule is connected. If you need to run it manually:

```sh
backlight_capture /dev/ttyUSBX
```

Replace `/dev/ttyUSBX` with the appropriate serial device.

## Configuration

You can configure the number of LEDs and other parameters via the config file:

- Linux: _/etc/backlightcapture/config.txt_
- Macos: _/Library/Application Support/BacklightCapture/config.txt_
- Windows: _C:\\ProgramData\\BacklightCapture\\config.txt_
  
or via command-line arguments. For example:

```sh
backlight_capture /dev/ttyUSBX -H 21 -aH 1 -V 9 -aV 0 -t 32 -s 4 -b 0.2 -f 60
```

- `-H`, `--horizontal-leds`: Number of horizontal LEDs
- `-aH`, `--add-horizontal`: Add virtual LEDs left and right
- `-V`, `--vertical-leds`: Number of vertical LEDs
- `-aV`, `--add-vertical`: Add virtual LEDs top and bottom
- `-t`, `--border-thickness`: Border thickness while calculating average color
- `-s`, `--skip-pixels`: Number of pixels to skip while calculating average color
- `-b`, `--brightness`: LED brightness
- `-f`, `--max-fps`: Maximum frames per second

## In progress

- DPMS support for MacOS and X11
- Windows support

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [OpenCV](https://opencv.org/)
- [CMake](https://cmake.org/)
- [X11](https://www.x.org/wiki/)
- [Wayland](https://wayland.freedesktop.org/)
- [Grim](https://github.com/emersion/grim)