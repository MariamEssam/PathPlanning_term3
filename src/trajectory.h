#pragma once
#include <algorithm>
#include <vector>
#include "spline.h"
#include "Helper.h"
#include "constant.h"

using namespace std;
class trajectory
{
public:
	trajectory();
	~trajectory();
	double ref_x;
	double ref_y;
	double ref_yaw;
	Helper helper;
	// actual (x, y) we use for the planner
	vector<double> next_x_vals;
	vector<double> next_y_vals;
	vector<double> previous_path_x;
	vector<double> previous_path_y;
	void set_previous_path(vector<double> previous_path_x, vector<double> previous_path_y);
	void convert2Local(vector<double>& ptsx, vector<double>& ptsy);
	vector<double> convert2global(double x, double y);
	void update_trajectory(vector<double> ptsx, vector<double> ptsy, double ref_vel);
	void generate_trajectory(double car_s, double original_x, double original_y, double original_yaw, int lane, double ref_vel);
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

