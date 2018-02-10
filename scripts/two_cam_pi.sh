#!/bin/sh
# run two cams on pi at 640 x 480 res 60 fps on two http streams/udp ports

# requires test_udp_trigger to be build for roborio and put somewhere on the PATH
test_udp_trigger -o raspberrypi.local 6789
test_udp_trigger -o raspberrypi.local 6788
