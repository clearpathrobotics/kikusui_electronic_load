# kikusui_electronic_load

> [!IMPORTANT]  
> This branch is for [ROS 2 Humble](https://docs.ros.org/en/humble/index.html).

> [!NOTE]  
> This driver has been tested with a [Kikusui PLZ1004W electronic load](./readme_assets/kikusui_plz_4wh_m.pdf).
> This drives places the electronic load in a constant power ( _CP_ ) state, lets you set a power draw in Watts, and then toggle the load _On_ or _Off_.

<br />

## Hardware

To be used with a [Kikusui PLZ1004W](readme_assets/kikusui_plz_4wh_m.pdf) electronic load.
We have tested this using an [FTDI USB-RS232-WE-BT-0.0](./readme_assets/ftdi_usb-rs232-we-1800-bt-00.pdf) cable.
The cable needs a DE9 female connector, with a pinout of:
1.  Not used
2.  Transmit (TX), ORANGE
3.  Receive (RX), YELLOW
4.  Not used
5.  GND, BLACK
6.  Not used
7.  Not used
8.  Not used
9.  Not used

TODO: add a cable drawing and CPR item number.

<br />

## Installation

1.  Create a workspace and src directory, `~/ros2_ws/src/`.
2.  Clone this git repository to the directory `~/ros2_ws/src/`.
    You now have a directory structure of:

    ```bash
    ~/ros2_ws/
        └── src/
            └── kikusui_electronic_load/
                ├── debian/
                |   ├── ...
                |   └── ...
                ├── kikusui_electronic_load/
                |   ├── ...
                |   └── ...
                ├── kikusui_electronic_load_interfaces/
                |   ├── ...
                |   └── ...
                └── README.md
    ```

3.  Install the udev rule:
    ```
    sudo cp ~/ros2_ws/src/kikusui_electronic_load/debian/udev /etc/udev/rules.d/51-kikusui.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```
4.  Build the workspace:
    ```
    cd ~/ros2_ws
    colcon build
    ```

5.  Source the workspace:
    ```
    source ~/ros2_ws/install/setup.bash
    ```

<br />

## Reference
-   [Closed source ROS 1 driver from clearpath](https://gitlab.clearpathrobotics.com/research/kikusui_load_interface)
-   [Need to use Cmake for SRV](https://answers.ros.org/question/322771/ros2-services-in-python/)
-   [Creating a C++ package with ament_make, or creating a Python package with ament_python](https://docs.ros.org/en/foxy/How-To-Guides/Developing-a-ROS-2-Package.html)
    -   [Creating a C++ and Python package with ament_cmake_python](https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Python-Documentation.html#using-ament-cmake-python)
-   [data_files](https://stackoverflow.com/questions/27829754/include-entire-directory-in-python-setup-py-data-files)
-   The remote interfaces comply with IEEE Std 488.2-1992 and [SCPI Specification 1999.0](https://www.ivifoundation.org/docs/scpi-99.pdf).

<br />

## Example SCPI commands

| Command              | Purpose                                                                            |
| :------------------- | :--------------------------------------------------------------------------------- |
| `*RST`               | Clears all settings                                                                |
| `SOUR:FUNC:MODE CP`  | changes the mode to _constant power_                                               |
| `SOUR:POW:RANG HIGH` | Changes the power range to _high_                                                  |
| `SOUR:POW:RANG MED`  | Changes the power range to _medium_                                                |
| `SOUR:POW:RANG LOW`  | Changes the power range to _low_                                                   |
| `SOUR:POW 100`       | Changes the load's power to _100 W_ ( or any integer specified from 0 W - 1000 W ) |
| `INP ON`             | Turns the load _on_                                                                |
| `INP OFF`            | Turns the load _off_                                                               |

## TODO

1.  Add udev rules details for the Kikusui FTDI.
    These are currently set as " _todo_ " in the file `./debian/udev`.
