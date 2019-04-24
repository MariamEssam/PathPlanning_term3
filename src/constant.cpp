#include "constant.h"


double constant::TIME_DIFF = 0.2;
double constant::MAX_S = 6945.554;
double constant::LANE_WIDTH = 4;
//Incorrect
double constant::PREDICTION_INTERVAL = 0.15;
//The number of points that we have per cycle
double constant::POINTS_PER_CYCLE = 10;
int constant::LANE_COUNT = 3;
double constant::MAX_SPEED = 48.7;
double constant::SPEED_INCREMENT = .224;
double constant::SAFE_DISTANCE = 20.0;
double constant::PREDICTIONS_COUNT =5;
double constant::DISTANCE = 30;
double constant::MIDDLE_LANE = constant::LANE_WIDTH / 2;
double constant::INTERVAL = .02;
constant::constant()
{
	
}


constant::~constant()
{
}
