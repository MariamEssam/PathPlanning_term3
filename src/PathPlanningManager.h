#pragma once
#include "constant.h"
#include <vector>
#include "Vehicle.h"
#include <map>
#include "json.hpp"
#include "PlannerFSM.h"
#include <iostream>
using namespace std;
using nlohmann::json;

class PathPlanningManager
{
public:
	PathPlanningManager(vector<double> &map_waypoints_x,vector<double> &map_waypoints_y,
		                vector<double> &map_waypoints_s,vector<double> &map_waypoints_dx,
		                vector<double> &map_waypoints_dy);
	~PathPlanningManager();
	void updatewatchinglist(json datafusion);
	void update_auto_car_state(double car_s, double x, double y, 
		                      double yaw, double s, double d, double speed);
	void generatePath(vector<double> previous_path_x, vector<double> previous_path_y);
	vector<double> get_x_values();
	vector<double> get_y_values();
private:
	//Watching list (the surronding vehicles that our vehicle should care about)
	map<int, Vehicle*> watchingvehicles;
	//Predictions for the surronding vehicles
	map<int, vector<prediction>> predictions;
	Vehicle ourvehicle;
	
	double original_yaw;
	PlannerFSM fsm=PlannerFSM(ourvehicle);
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;
	trajectory traject;
};

