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
- how to clear the fence and reload another fence
- How to receive the battery level from the drone?
- How to define the actions when teh drone hits the fence?