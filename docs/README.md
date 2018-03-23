#List of dependencies
*Get OpenCV
	*I used this tutorial https://www.learnopencv.com/install-opencv3-on-ubuntu/
*Download and unzip AruCo
	*in build folder cmake ..
	*make
	*make install
*Copy of eigen3 from phil somewhere that orocos can find it
*Download and unzip MarkerMapper
	in build folder cmake
	*make
	*make install
*Get ROS in order to use orocos
*Download and unzip Orocos Bayesian filtering
	*in orocos_bfl in build cmake -DMATRIX_LIB=eigen ..
	*make
	*make install
*Remove source /opt/ros/kinetic/setup.bash from ~/.bashrc
*In phil/cpp/build
	*cmake -DCMAKE_BUILD_TYPE=Release ..
	*make
	*make install
*Get java 8
*Most recent version of eclipse
*Set up WPILib on eclipse
	*https://wpilib.screenstepslive.com/s/currentCS/m/getting_started/l/599679-installing-eclipse-c-java
*Get Shuffleboard
	*follow instructions here https://github.com/PHIL-MQP/phil-shuffleboard-plugin

##How to run phil_main
*Put .yml files in  /phil/cpp/build
*Run shuffleboard looking at phil widget
	*java -jar ~/wpilib/tools/Shuffleboard.jar
*Run Outlook viewer
	*java -jar ~/wpilib/tools/OutlineViewer.jar
*in phil/cpp/build/tools
	*./publish_rio_data ..
*in phil/cpp/build
	*./phil_main config.yml –no-camera

