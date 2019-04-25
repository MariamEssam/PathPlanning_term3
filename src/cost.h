//#pragma once
#ifndef COST_INCLUDED
#define COST_INCLUDED
#include "PlannerFSM.h"
#include <functional>
#include <iostream>
class cost
{
public:
	cost();
	~cost();
	double Calculatecost(double car_s, double ref_vel, vector<snapshot> trajectory,
		map<int, vector<prediction>>predictions, FSMStates state);
	FSMStates state;
	double estimatedcost;
	struct collision {
		bool hasCollision = false;
		int step = 1000;
	};

	struct TrajectoryData
	{
		int proposed_lane;
		int current_lane;
		double avg_speed;
		double prop_closest_approach;
		double actual_closest_approach;
		collision collides;
	};

	typedef std::function<double(const cost&, vector<snapshot> trajectory,
		map<int, vector<prediction>> predictions, TrajectoryData data)> DelegateType;

	vector<DelegateType> delegates = { (DelegateType)&cost::inefficiency_cost,
		(DelegateType)&cost::collision_cost,
		(DelegateType)&cost::buffer_cost,
		(DelegateType)&cost::change_lane_cost,
		(DelegateType)&cost::free_line_cost
	};

	// priority levels for costs
	int const COLLISION = pow(10, 6);
	int const DANGER = 3 * pow(10, 5);
	int const COMFORT = pow(10, 4);
	int const EFFICIENCY = pow(10, 3);

	double MAX_SPEED;
	double const DESIRED_BUFFER = constant::SAFE_DISTANCE * 2;

	int const PLANNING_HORIZON = 1;

	double const PREDICTION_INTERVAL = 0.15;
	double const MANOEUVRE = 4.0;
	double const OBSERVED_DISTANCE = 65;
	double const MAX_DISTANCE = 999999;

	double change_lane_cost(vector<snapshot> trajectory,
		map<int, vector<prediction>> predictions, TrajectoryData data) const;

	double inefficiency_cost(vector<snapshot> trajectory,
		map<int, vector<prediction>> predictions, TrajectoryData data) const;

	double collision_cost(vector<snapshot> trajectory,
		map<int, vector<prediction>> predictions, TrajectoryData data) const;

	double buffer_cost(vector<snapshot> trajectory,
		map<int, vector<prediction>> predictions, TrajectoryData data) const;

	double free_line_cost(vector<snapshot> trajectory,
		map<int, vector<prediction>> predictions, TrajectoryData data) const;

	TrajectoryData get_helper_data(double car_s, double ref_s, vector<snapshot> trajectory,
		map<int, vector<prediction>> predictions, FSMStates state);

	bool check_collision(double car_s, double ref_speed, snapshot snap, prediction s_now,
		FSMStates checkstate, bool lack_of_space);

	map<int, vector<prediction>> filter_predictions_by_lane(
		map<int, vector<prediction>> predictions, int lane);
};
#endif
