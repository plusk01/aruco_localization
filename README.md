ArUco Localization ROS Wrapper
==============================

This project utilizes the [ArUco library](https://www.uco.es/investiga/grupos/ava/node/26) and exposes several of the examples in that library through ROS. This project can also be used to localize a camera/robot using an ArUco Marker Map.

## Dependencies ##

Install the [ArUco library](https://www.uco.es/investiga/grupos/ava/node/26) (Written/tested with version 2.0.19):

    $ mkdir build
    $ cd build
    $ cmake .. # -DUSE_OWN_EIGEN=no
    $ make
    $ sudo make install

**Note:** This ROS package (in the `CMakeLists.txt`) assumes that the `Findaruco.cmake` file was installed to `/usr/local/lib/cmake/`. Look at the output of `sudo make install` to verify this location.

## Usage ##

The `aruco_localization` node publishes two topics: `estimate` and `measurements`. Given an ArUco marker dictionary, any markers in that dictionary family will be identified and the measurement to that specific marker will be reported in the `measurements` topic. The `estimate` topic provides the overall pose estimate of a marker map. The marker map that is being tracked is defined in the `markermap_config` file, which is a YAML file that lists all of the markers and their positions within a marker map. An example YAML file can be found [here](https://github.com/plusk01/desktopquad/blob/master/catkin_ws/src/desktopquad/params/map.yaml).

## Resources ##

- [OpenCV contrib](https://docs.opencv.org/3.3.0/d9/d6d/tutorial_table_of_content_aruco.html)
- [ArUco library](https://www.uco.es/investiga/grupos/ava/node/26)
