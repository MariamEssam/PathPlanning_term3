
#include "cost.h"



cost::cost()
{
	MAX_SPEED = constant::MAX_SPEED;
}


cost::~cost()
{
}
double cost::Calculatecost(double car_s, double ref_vel, vector<snapshot> trajectory,
	map<int, vector<prediction>>predictions, FSMStates state)
{
	TrajectoryData trajectory_data = get_helper_data(car_s, ref_vel, trajectory, predictions, state);

	double lcost = 0.0;
	for (auto cf : delegates) {
		double new_cost = cf(*this, trajectory, predictions, trajectory_data);
		lcost += new_cost;
	}
	return lcost;
}
double cost::change_lane_cost(vector<snapshot> trajectory,
	map<int, vector<prediction>> predictions, TrajectoryData data) const {

	if (data.proposed_lane != data.current_lane) {
		if (data.proposed_lane == 1) {
			return 0;
		}
		return COMFORT;
	}

	return 0;
}

double cost::inefficiency_cost(vector<snapshot> trajectory,
	map<int, vector<prediction>> predictions, TrajectoryData data) const {
	double speed = data.avg_speed;
	double target_speed = MAX_SPEED;
	double diff = target_speed - speed;
	double pct = diff / target_speed;
	double multiplier = pow(pct, 2);
	return 8 * multiplier * EFFICIENCY;
}

double cost::collision_cost(vector<snapshot> trajectory,
	map<int, vector<prediction>> predictions, TrajectoryData data) const {
	if (data.collides.hasCollision) {
		double time_til_collision = 0;
		double exponent = time_til_collision*time_til_collision;
		double mult = exp(-exponent);
		
		return mult * COLLISION;
	}
	return 0;
}

double cost::free_line_cost(vector<snapshot> trajectory,
	map<int, vector<prediction>> predictions, TrajectoryData data) const {
	double closest = data.prop_closest_approach;

	if (closest > OBSERVED_DISTANCE) {
		double multiplier = (MAX_DISTANCE - closest) / MAX_DISTANCE;
		return 20 * multiplier*multiplier;
	}
	double multiplier = (OBSERVED_DISTANCE - closest) / OBSERVED_DISTANCE;
	return 5 * multiplier * multiplier * COMFORT;
}

double cost::buffer_cost(vector<snapshot> trajectory,
	map<int, vector<prediction>> predictions, TrajectoryData data) const {
	double closest = data.actual_closest_approach;
	
	if (closest < constant::SAFE_DISTANCE / 2) {
		return 3 * DANGER;
	}

	if (closest > DESIRED_BUFFER) {
		return 0.0;
	}

	double multiplier = 1.0 - pow((closest / DESIRED_BUFFER), 2);
	return 3 * multiplier * DANGER;
}



cost::TrajectoryData cost::get_helper_data(double car_s, double ref_s, vector<snapshot> trajectory,
	map<int, vector<prediction>>predictions, FSMStates checkstate) {
	TrajectoryData data = TrajectoryData();

	vector<snapshot> t = trajectory;
	// actual state
	snapshot current_snapshot = t[0];
	snapshot first = t[1];
	snapshot last = t[t.size() - 1];

	double dt = trajectory.size()*PREDICTION_INTERVAL;
	// for lane change we see actual line after current state only
	data.current_lane = first.lane;
	data.proposed_lane = last.proposed_lane;
	data.avg_speed = (last.get_speed()*dt - current_snapshot.get_speed()) / dt; // (v2*dt-v1*1)/dt

																				// initialize a bunch of variables
	data.prop_closest_approach = MAX_DISTANCE;
	data.actual_closest_approach = MAX_DISTANCE;

	data.collides = collision();
	data.collides.hasCollision = false;
	bool checkCollisions = current_snapshot.lane != data.proposed_lane;

	map<int, vector<prediction>> cars_in_proposed_lane = filter_predictions_by_lane(predictions, data.proposed_lane);
	map<int, vector<prediction>> cars_in_actual_lane = filter_predictions_by_lane(predictions, data.current_lane);

	for (int i = 0; i < PLANNING_HORIZON; ++i) {
		snapshot snap = trajectory[i];

		for (auto pair : cars_in_actual_lane) {
			prediction state = pair.second[i];
			double dist = -state.get_distance(snap.x, snap.y, snap.s);
			if (dist >= 0 && dist < data.actual_closest_approach) {
				data.actual_closest_approach = dist;
			}
		}
	}

	for (int i = 0; i < PLANNING_HORIZON; ++i) {
		snapshot snap = trajectory[i];

		for (auto pair : cars_in_proposed_lane) {
			prediction state = pair.second[i];
			//double pred_car_s = car_s + i*0.15*ref_s;
			double dist = -state.get_distance(snap.x, snap.y, snap.s);
			if (checkCollisions) {
				bool vehicle_collides = check_collision(car_s, ref_s, snap, state, checkstate,
					data.actual_closest_approach < MANOEUVRE);
				if (vehicle_collides) {
					data.collides.hasCollision = true;
					data.collides.step = i;
				}
				else if (car_s > state.s) {
					dist = MAX_DISTANCE;
				}
			}
			if (dist >= 0 && dist < data.prop_closest_approach) {
				data.prop_closest_approach = dist;
				if (data.proposed_lane == data.current_lane) {
					data.actual_closest_approach = data.prop_closest_approach;
				}
			}
		}
	}

	return data;
}


bool cost::check_collision(double car_s, double ref_speed, snapshot snap, prediction s_now, FSMStates checkstate,
	bool lack_of_space) {
	double s = snap.s;
	double v = snap.get_speed();

	double collide_car_v = s_now.getVelocity();
	double diff = s_now.get_distance(snap.x, snap.y, snap.s);
	double prediction_time = 4 / snap.vx;
	if (car_s > s_now.s) {// TODO
		double predicted_distance1v = diff + prediction_time*(v - collide_car_v);
		double predicted_distance2v = diff + 10 * PREDICTION_INTERVAL*(ref_speed - collide_car_v);
		if ((predicted_distance2v < MANOEUVRE || predicted_distance1v < MANOEUVRE || lack_of_space || diff < -1.0)) {
			
			return true;
		}
	}
	else {
		double predicted_distance1v = -diff + 3 * PREDICTION_INTERVAL*(collide_car_v - v);
		if (predicted_distance1v < 0 || -diff < -MANOEUVRE) {
			
			return true;
		}
	}

	return false;
}

map<int, vector<prediction>> cost::filter_predictions_by_lane(
	map<int, vector<prediction>> predictions, int lane) {
	map<int, vector<prediction>> filtered = {};
	for (auto pair : predictions) {
		// because of poor coord transformation reduce lane definition on 0.5m
		if (pair.second[0].lane == lane) {
			filtered[pair.first] = pair.second;
		}
	}
	return filtered;
}
