# kikusui_electronic_load

<br />

## Hardware

To be used with a [Kikusui PLZ1004W](readme_assets/kikusui_plz_4wh_m.pdf) electronic load.
We have tested this using an FTDI USB-RS232-0.0 cable.
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

1.  Add a launch file
