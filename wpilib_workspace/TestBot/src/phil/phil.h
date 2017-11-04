#pragma once

#include <memory>
#include <string>

#include <AHRS.h>
#include <Encoder.h>

namespace phil {

const std::string kTableName = "phil_table";
const std::string kEncodersKey = "encoders";
const std::string kINSKey = "ins";
const std::string kPoseKey = "pose";

struct pose_t {
  double x;
  double y;
  double phi;

  double dxdt;
  double dydt;
  double dphidt;
};

class Phil {
private:
	static Phil *instance;

	Phil();

	Encoder *left_encoder;
	Encoder *right_encoder;
	AHRS *ahrs;
	std::shared_ptr<NetworkTable> table;

public:
	static Phil *GetInstance();

	void GiveSensors(Encoder *left_encoder, Encoder *right_encoder, AHRS *ahrs);

	void ReadSensorsAndProcessLocally();
	void ReadSensorsAndProcessOnTK1();
	pose_t GetPosition();
};

}
