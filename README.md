
# Usage for MAVLink-C Library

Step 0. Install ArduPilot library by following the tutorial below
`https://docs.google.com/document/d/1OgCC17BiFaRYml6ymLix7IrCFkasUEywPy2u3vIW_HE/edit`

Step 1. Clone official MAVLink repository
`git clone https://github.com/mavlink/mavlink --recursive`

Step 2. Replace the example/c folder with the custom repository
`git clone https://github.com/sychoo/MAVLink-C-Library-for-ArduPilot`

Step 3. Build the `demo.c` program by invoking the Makefile. In the case that it doesn't work, simply executing the command within the `Makefile`
`make`

Step 4. Run the compiled program
`./build/demo` which will simulate the ADS-B signal that will be sent to the drone.
