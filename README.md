ArUco Localization ROS Wrapper
==============================

This project utilizes the [ArUco library](https://www.uco.es/investiga/grupos/ava/node/26) and exposes several of the examples in that library through ROS. This project can also be used to localize a camera/robot using an ArUco Marker Map.

## Dependencies ##

Install the [ArUco library](https://www.uco.es/investiga/grupos/ava/node/26) (Written/tested with version 2.0.19):

    ```bash
    $ mkdir build
    $ cd build
    $ cmake .. # -DUSE_OWN_EIGEN=no
    $ make
    $ sudo make install
    ```

    **Note:** This ROS package (in the `CMakeLists.txt`) assumes that the `Findaruco.cmake` file was installed to `/usr/local/lib/cmake/`. Look at the output of `sudo make install` to verify this location.

## Resources ##

- [OpenCV contrib]()
- [ArUco library]()
