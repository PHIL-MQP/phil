all: phil_rio phil_linux

phil_rio:
	mkdir -p ./cpp/rio_build
	cd ./cpp/rio_build && cmake .. -DRIO=ON -DCMAKE_CXX_COMPILER=/usr/bin/arm-frc-linux-gnueabi-g++ && make

phil_linux:
	mkdir -p ./cpp/build
	cd ./cpp/build && cmake .. -DRIO=OFF && make
