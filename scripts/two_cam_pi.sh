#!/bin/sh
# run two cams on pi at 640 x 480 res 60 fps on two htpp streams/udp ports
# uncomment last two lines to test triggering

cd ../cpp/build 
./tools/mocap_record 0 640 480 60 8081 6789 &
./tools/mocap_record 1 640 480 60 8082 6788 &
#./tools/test_udp_trigger -o localhost  6789 &
#./tools/test_udp_trigger -o localhost  6788