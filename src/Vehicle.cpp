#include "Vehicle.h"



Vehicle::Vehicle()
{
	this->id = -1;
}

Vehicle::Vehicle(int l_id, double l_x, double l_y, double l_Vx, double l_Vy, double l_s, double l_d)
{
	this->id = l_id;
	this->x = l_x;
	this->y = l_y;
	this->Vx = l_Vx;
	this->Vy = l_Vy;
	this->s = l_s;
	this->d = l_d;
}

Vehicle::~Vehicle()
{
}

void Vehicle::update_accel(double vx, double vy)
{
	//Update the acceleration of x component delta x /diff time
	this->ax = (vx - this->Vx) / constant::TIME_DIFF;
	//Ignore small values 
	if (this->ax < 0.01) {
		this->ax = 0;
	}
	this->ay = (vy - this->Vy) / constant::TIME_DIFF;
	if (this->ay < 0.01) {
		this->ay = 0;
	}
	this->Vx = vx;
	this->Vy = vy;
}
void Vehicle::update_yaw(double vx, double vy) 
{
	double new_angle = atan2(vy, vx);
	this->yaw = (fabs(new_angle) < 0.1) ? 0 : new_angle;
}
void Vehicle::updatewatchingVehiclePos(double x, double y, double s, double d)
{
	this->x = x;
	this->y = y;
	this->s = s;
	//Incorrect
	int new_lane = (int)d / 4;
	if (new_lane != this->lane) {
		if (++updates > 6) {
			this->lane = new_lane;
			updates = 0;
		}
	}
	else {
		updates = 0;
	}
	this->d = d;
}
void Vehicle::update_params(double x, double y, double yaw, double s, double d, double speed) {
	this->x = x;
	this->y = y;
	this->yaw = helper.deg2rad(yaw);
	update_accel(speed*cos(this->yaw), speed*sin(this->yaw));
	this->s = s;
	this->d = d;
}
vector<prediction> Vehicle::generate_predictions(vector<double> &map_waypoints_x, vector<double> &map_waypoints_y,
	vector<double> &map_waypoints_s, vector<double> &map_waypoints_dx,
	vector<double> &map_waypoints_dy)
{
	vector<prediction> predictions;
	for (int i = 0; i < constant::POINTS_PER_CYCLE; i++)
	{
		prediction pred;
		//Incorrect: This part need to be revisted the calculations are not right
		double time_delta = i*constant::PREDICTION_INTERVAL;
		//X componenent 
		if (fabs(this->ax) < 0.001)
		{
			pred.x = this->x + (this->Vx*time_delta);
			pred.vx = Vx;
		}
		else
		{
			//Incorrect /2 should for vx and ax not ax only
			pred.x = this->x + (this->Vx*time_delta)+ (this->ax*time_delta*time_delta)/2;
			pred.vx = Vx;
		}
		//Y componenent 
		if (fabs(this->ay) < 0.001)
		{
			pred.y = this->y + (this->Vy*time_delta);
			pred.vy = Vy;
		}
		else
		{
			//Incorrect /2 should for vx and ax not ax only
			pred.y = this->y + (this->Vy*time_delta) + (this->ay*time_delta*time_delta) / 2;
			pred.vy = Vy;
		}
		double new_angle = atan2(pred.vy, pred.vx);
		double l_yaw = (new_angle < 0.1) ? 0 : new_angle;
		vector<double> fernet = helper.getFrenet(this->x, this->y, l_yaw, map_waypoints_x, map_waypoints_y);
		pred.s = fernet[0];
		pred.d = fernet[1];
		//Incorrect (after d we should find the lane)
		pred.lane = this->lane;
		predictions.push_back(pred);
	}
	return predictions;
}
bool Vehicle::is_in_front_of(prediction pred, int checked_lane) {
	return (pred.lane == checked_lane) && pred.get_distance(x, y, s) >= 0;
}

bool Vehicle::is_behind_of(prediction pred, int lane) {
	double distance = -pred.get_distance(x, y, s);
	return (pred.lane == lane) && (distance >= 0 && distance < 2 *constant::SAFE_DISTANCE);
}

bool Vehicle::is_close_to(prediction pred, int lane) {
	double distance = -pred.get_distance(x, y, s);
	return (pred.lane == lane) && distance >= 0 && (distance < constant::SAFE_DISTANCE);
}
void Vehicle::increment(double t,const vector<double> &maps_x,
	const vector<double> &maps_y)
{
	if (abs(this->ay) < 0.001) {
		this->y += this->Vy * t;
	}
	else {
		this->y += this->Vy * t + this->ay*t*t / 2;
		this->Vy += this->ay * t;
	}
	if (abs(this->ax) < 0.001) {
		this->x += this->Vx * t;
	}
	else {
		this->x += this->Vx * t + this->ax*t*t / 2;
		this->Vx += this->ax * t;
	}
	double new_angle = atan2(Vy, Vx);
	this->yaw = (new_angle < 0.1) ? 0 : new_angle;
	vector<double> frenet = helper.getFrenet(this->x, this->y, this->yaw, maps_x, maps_y);
	this->s = frenet[0];
	this->d = frenet[1];
}