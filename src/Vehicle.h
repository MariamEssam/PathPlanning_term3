//#pragma once
#ifndef VEHICLE_H
#define VEHICLE_H
#include "constant.h"
#include <math.h>
#include <vector>
#include "Helper.h"
#include "trajectory.h"
using namespace std;

struct prediction {
	double s;
	double d;
	double vx;
	double vy;
	double x;
	double y;
	int lane;

public:
	double getVelocity(){ return sqrt(vx*vx + vy*vy); }
	double get_distance(double other_x, double other_y, double other_s) {
		double diff_car = sqrt((x - other_x)*(x - other_x) + (y - other_y)*(y - other_y));
		double diff_frenet = other_s - s;
		if (diff_car - std::abs(diff_frenet) < 100) { // no circle overlap
			return diff_frenet;
		}
		else {
			return copysign(diff_car, diff_frenet);
		}
	}
};
enum FSMStates
{
	Ready=0,//Init
	KL=1,//keep lane
	LCL=4,//Lane change left
	LCR=5,//Lane Change 
	PLCL=2,//Prepare lane change left
	PLCR=3//Prepare lane change right

};
class Vehicle
{
public:
	Vehicle();
	Vehicle(int l_id,double l_x, double l_y, double l_Vx, double l_Vy, double l_s, double l_d);
	~Vehicle();
	int id;
	double x;
	double y;
	double Vx;
	double Vy;
	double ax;
	double ay;
	double yaw;
	double s;
	double d;
	int lane = 1;
	void update_accel(double vx, double vy);
	void update_yaw( double vx, double vy);
	void updatewatchingVehiclePos(double x, double y, double s, double d);
	vector<prediction> generate_predictions(vector<double> &map_waypoints_x, vector<double> &map_waypoints_y,
		vector<double> &map_waypoints_s, vector<double> &map_waypoints_dx,
		vector<double> &map_waypoints_dy);
	int updates = 0;
	void update_params(double x, double y, double yaw, double s, double d, double speed);
	Helper helper;
	bool is_in_front_of(prediction pred, int checked_lane);
	bool is_behind_of(prediction pred, int lane);
	bool is_close_to(prediction pred, int lane);
	void increment(double t, const vector<double> &maps_x,
		const vector<double> &maps_y);
	prediction state_at(double t, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y,
		vector<double> &map_waypoints_s, vector<double> &map_waypoints_dx,
		vector<double> &map_waypoints_dy);
};
struct snapshot {
	double x;
	double y;
	double vx;
	double vy;
	double ax;
	double ay;
	double s;
	double d;
	double yaw;
	int lane;
	int proposed_lane;
	FSMStates state;
	double ref_vel;


	double get_speed() {
		return sqrt(pow(vx,2) + pow(vy, 2));
	}

	double get_acceleration() {
		return sqrt(ax*ax + ay*ay);
	}

	/*void display() {
		cout << "snapshot: x " << x << " y " << y << " dx " << dx << " dy "
			<< dy << " ddx " << ddx << " ddy " << ddy << " s " << s << " d " << d << " yaw " << yaw
			<< " lane " << lane << endl;
	}*/
};
#endif