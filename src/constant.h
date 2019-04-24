#pragma once

class constant
{
public:
	constant();
	~constant();
	//Time diff 
	double static TIME_DIFF;
	//Lane width
	double static LANE_WIDTH;
	// The max s value before wrapping around the track back to 0
    double static MAX_S;
	double static PREDICTION_INTERVAL;
	double static PREDICTIONS_COUNT;
	double static POINTS_PER_CYCLE;
	int static LANE_COUNT;
	double static MAX_SPEED;
	double static SPEED_INCREMENT;
	double static SAFE_DISTANCE;
	double static DISTANCE;
	double static MIDDLE_LANE;
	double static INTERVAL;
};

