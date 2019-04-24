#pragma once
#include "Vehicle.h"
#include "constant.h"
#include <map>
#include "cost.h"

class PlannerFSM
{
public:
	PlannerFSM(Vehicle& car);
	~PlannerFSM();
	bool verbosity = false;
	Vehicle& autonumous_car;
	double car_s;
	void update_state(map<int, vector<prediction>> predictions);
	FSMStates get_next_state(map<int, vector<prediction>> predictions);
	FSMStates state=FSMStates::Ready;
	vector<snapshot> trajectory_for_state(FSMStates proposed_state, map<int, vector<prediction>> predictions, int horizon);
	snapshot get_snapshot();
	double ref_vel = 0;
	int proposed_lane = -1;
	void restore_state_from_snapshot(snapshot snapshot);
	void realize_state(map<int, vector<prediction>> predictions);
	void realize_constant_speed();
	void _update_ref_speed_for_lane(map<int, vector<prediction>> predictions, int checked_lane);
	void realize_keep_lane(map<int, vector<prediction>> predictions);
	void realize_lane_change(map<int, vector<prediction>> predictions, string direction);
	void realize_prep_lane_change(map<int, vector<prediction>> predictions, string direction);
	double get_expected_velocity();
	void setwaypoints(vector<double> &map_waypoints_x, vector<double> &map_waypoints_y,
		vector<double> &map_waypoints_s, vector<double> &map_waypoints_dx,
		vector<double> &map_waypoints_dy);
private:
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;
};

