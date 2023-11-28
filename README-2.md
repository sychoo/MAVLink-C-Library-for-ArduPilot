# Install ArduPilot
Tutorials: https://docs.google.com/document/d/1OgCC17BiFaRYml6ymLix7IrCFkasUEywPy2u3vIW_HE/edit

# Simple C example

Simple example receiving and sending MAVLink v2 over UDP based on POSIX APIs (e.g. Linux, BSD, macOS).

## Install MAVLink

In top level directory, build and install the MAVLink headers locally into the install folder:

```
cmake -Bbuild -H. -DCMAKE_INSTALL_PREFIX=install
cmake --build build --target install
```

## Build example

In the example directory, build the example while passing the local install directory:

```
cd examples/c
cmake -Bbuild -H. -DCMAKE_PREFIX_PATH=$(pwd)/../../install
cmake --build build
```

============== Updated for Ardupilot ==============
# installation for ardupilotmega
```
# build MAVLink library
cmake -Bbuild -H. -DCMAKE_INSTALL_PREFIX=install -DMAVLINK_DIALECT=ardupilotmega -DMAVLINK_VERSION=2.0
cmake --build build --target install

# build the MAVLink controller code
cd examples/c
cmake -Bbuild -H. -DCMAKE_PREFIX_PATH=$(pwd)/../../install
cmake --build build
```

# Debugging which Mavlink Message should be used
```
# First go to top level repository, search for keyword within the following directory
cd install/include/mavlink
```

# Error
```
CMake Error at CMakeLists.txt:7 (find_package):
  By not providing "FindMAVLink.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "MAVLink", but
  CMake did not find one.

  Could not find a package configuration file provided by "MAVLink" with any
  of the following names:

    MAVLinkConfig.cmake
    mavlink-config.cmake

  Add the installation prefix of "MAVLink" to CMAKE_PREFIX_PATH or set
  "MAVLink_DIR" to a directory containing one of the above files.  If
  "MAVLink" provides a separate development package or SDK, be sure it has
  been installed.
```

When encounter this error, try not to use Makefile, compile the files by typing the commands manually in the commandline.

# Mission
To start the mission in Ardupilot, it first have to send a mission count, signals start transmission of mission items, and then send the mission items.

# To Do
- How to list a fence (so that it shows up on the visualization map)
    - no success, fence list command in MAVProxy starts the visualization, while the simply executing the fetch_fence command does not.
    - Error message: Vehicle waypoint too advanced. Suspecting because of I didn't receive and unpack the message
    - this is where it was originally defined in MAVProxy
    - verdict: it is not possible: self.draw controls the UI which cannot be invoked via MAVLink Messages
    - simply redefine the fence to override the previous one. clear_fence doesn't work

- how to clear the fence and reload another fence
    - doesn't work

- How to receive the battery level from the drone?
    - done, using Battery Status, in the place of receive_somes

- How to define the actions when the drone hits the fence?

Error: Fence on the MAVProxy Map cannot be refreshed.
Solution: repeatedly execute the `fence list` command in MAVProxy, and the fence will be refreshed.

# TODO
- specify fence breaching actions (get drone to hold when the fence is breached)

- offboard mode using mavlink messages (https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)

- receive a signal when the fence is breached
- AP_Fence_Breached

- Calculate the distance between the current position to teh home point

- How to stop the mission?

# Hacking
- first setmode, takeoff and start the mission
- then start, define, and maintain the geo fence, monitor breaching
- if breached, hold the drone, or return to home
- how to clear the mission

Note that mission cannot be started when exceeding the geo fence
- demonstrate regular geo fencing
- shrinking the geo fence

need to receive something first before being able to send something (for initial command)