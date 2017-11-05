# MoCap (Motion Capture) Bot

This is the code that we use in our motion capture tests to bag sensor data. It should read sensor data, camera data, etc. in the same manner that the Test Bot programs do.

The port numbers are in `RobotMap.cpp`.

The output should be one file that logs all time stamped sensor data. Not sure what kind of file this will be. If our camera images are 640x480, 24 bit color, that would be 921 kB per frame. If our mo-cap test is 10 seconds and we run at 30fps, that means 921*30*10 = 276 MB of just image data. The RoboRIO only has 512MB of flash so we have to only record for short times. Alternatively, we just use a flash drive.
