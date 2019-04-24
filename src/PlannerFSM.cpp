#include "PlannerFSM.h"



PlannerFSM::PlannerFSM(Vehicle& car):autonumous_car(car)
{
	
}


PlannerFSM::~PlannerFSM()
{
}

void PlannerFSM::update_state(map<int, vector<prediction>> predictions) {
	
	state = get_next_state(predictions);
}

FSMStates PlannerFSM::get_next_state(map<int, vector<prediction>> predictions) {
	vector<FSMStates> states;
	if (state == FSMStates::PLCR) {
		states = vector<FSMStates>{ FSMStates::KL, FSMStates::PLCR, FSMStates::LCR };
	}
	else if (state == FSMStates::PLCL) {
		states = vector<FSMStates>{ FSMStates::KL, FSMStates::PLCL, FSMStates::LCL };
	}
	else if (state == FSMStates::LCR) {
		states = vector<FSMStates>{ FSMStates::KL };
	}
	else if (state == FSMStates::LCL) {
		states = vector<FSMStates>{ FSMStates::KL };
	}
	else {
		states = vector<FSMStates>{ FSMStates::KL };
		if (autonumous_car.lane > 0) {
			states.push_back(FSMStates::PLCL);
		}
		if (autonumous_car.lane < constant::LANE_COUNT - 1) {
			states.push_back(FSMStates::PLCR);
		}
	}

	if (states.size() == 1) {
		return states[0];
	}

	
	auto costs = vector<cost>();
	//double mincost = DBL_MAX;
	int bestcostIndex=-1;
	int Index = -1;
	for (auto state : states) {
		Index++;
		cost estimatedcost;
		estimatedcost.state = state;
		auto trajectory = trajectory_for_state(state, predictions, constant::PREDICTIONS_COUNT);
		estimatedcost.estimatedcost = estimatedcost.Calculatecost(car_s, ref_vel, trajectory, predictions, state);
		costs.push_back(estimatedcost);
		//std::cout << "The  cost is " << estimatedcost.estimatedcost << endl;
	}
	//std::cout << "***************************************************" << endl;
	auto best = min_element(std::begin(costs), std::end(costs),
		[](cost est1, cost est2) {
		return est1.estimatedcost < est2.estimatedcost;
	});
	
	return (*best).state;
}
vector<snapshot> PlannerFSM::trajectory_for_state(FSMStates proposed_state, map<int, vector<prediction>> predictions, int horizon) {
	// remember current state
	auto initial_snapshot = get_snapshot();

	vector<snapshot> trajectory = { initial_snapshot };
	for (int i = 0; i < horizon; ++i) {
		restore_state_from_snapshot(initial_snapshot);
		state = proposed_state;
		realize_state(predictions);
		autonumous_car.increment(i*constant::PREDICTION_INTERVAL,map_waypoints_x,map_waypoints_y);
		trajectory.push_back(get_snapshot());

		// need to remove first prediction for each vehicle.
		for (auto pair : predictions) {
			auto vehicle_pred = pair.second;
			vehicle_pred.erase(vehicle_pred.begin());
		}
	}

	// restore state from snapshot
	restore_state_from_snapshot(initial_snapshot);
	return trajectory;
}
//Copy from the autonumus vehicle to the snapshoot
snapshot PlannerFSM::get_snapshot() {
	snapshot snapshot_temp;
	snapshot_temp.x = this->autonumous_car.x;
	snapshot_temp.y = this->autonumous_car.y;
	snapshot_temp.vx = this->autonumous_car.Vx;
	snapshot_temp.vy = this->autonumous_car.Vy;
	snapshot_temp.s = this->autonumous_car.s;
	snapshot_temp.d = this->autonumous_car.d;
	snapshot_temp.ax = this->autonumous_car.ax;
	snapshot_temp.ay = this->autonumous_car.ay;
	snapshot_temp.yaw = this->autonumous_car.yaw;
	snapshot_temp.lane = this->autonumous_car.lane;
	snapshot_temp.state = this->state;
	snapshot_temp.ref_vel = this->ref_vel;
	snapshot_temp.proposed_lane = this->proposed_lane;

	return snapshot_temp;
}
//Copy from the snapshoot to the Vehicle instance
void PlannerFSM::restore_state_from_snapshot(snapshot snapshot) {
	this->autonumous_car.s = snapshot.s;
	this->autonumous_car.d = snapshot.d;
	this->autonumous_car.x = snapshot.x;
	this->autonumous_car.y = snapshot.y;
	this->autonumous_car.Vx = snapshot.vx;
	this->autonumous_car.Vy = snapshot.vy;
	this->autonumous_car.ax = snapshot.ax;
	this->autonumous_car.ay = snapshot.ay;
	this->autonumous_car.yaw = snapshot.yaw;
	this->autonumous_car.lane = snapshot.lane;
	this->state = snapshot.state;
	this->ref_vel = snapshot.ref_vel;
	this->proposed_lane = snapshot.proposed_lane;
}

void PlannerFSM::realize_state(map<int, vector<prediction>> predictions) {
	//Given a state, realize it by adjusting velocity and lane.
	if (state == FSMStates::Ready)
	{
		realize_constant_speed();
	}
	else if (state == FSMStates::KL)
	{
		realize_keep_lane(predictions);
	}
	else if (state == FSMStates::LCL)
	{
		realize_lane_change(predictions, "L");
	}
	else if (state == FSMStates::LCR)
	{
		realize_lane_change(predictions, "R");
	}
	else if (state == FSMStates::PLCL)
	{
		realize_prep_lane_change(predictions, "L");
	}
	else if (state == FSMStates::PLCR)
	{
		realize_prep_lane_change(predictions, "R");
	}
}

void PlannerFSM::realize_constant_speed() { }

void PlannerFSM::_update_ref_speed_for_lane(map<int, vector<prediction>> predictions, int checked_lane) {
	bool too_close = false, keep_speed = false, danger = false;
	double max_speed = constant::MAX_SPEED;

	for (auto pair : predictions) {
		prediction pred = pair.second[0];
		double target_speed = pred.getVelocity();
		if (autonumous_car.is_behind_of(pred, checked_lane) && target_speed < max_speed) {
			// follow the car behavior
			max_speed = target_speed - constant::SPEED_INCREMENT;//increment speed with small amount to avoid jerk
			keep_speed = true;
		}
		if (autonumous_car.is_close_to(pred, checked_lane)) {

			
			if (pred.s < autonumous_car.s + 5) {
				//Collision can happen
				danger = true;
			}
			too_close = true;
		}
	}
	double velocity = ref_vel;
	if (too_close) {
		if (danger) {
			if (velocity > 40.0) {
				velocity -= 2 * constant::SPEED_INCREMENT;
			}
			else {
				velocity -= constant::SPEED_INCREMENT;
			}
		}
		else {
			if (velocity < max_speed) {
				velocity += constant::SPEED_INCREMENT;
			}
			else if (velocity > max_speed) {
				if (velocity > 40.0) {
					velocity -= 2 * constant::SPEED_INCREMENT;
				}
				else {
					velocity -= constant::SPEED_INCREMENT;
				}
			}
		}
	}
	else {
		if (keep_speed && velocity > 25 && velocity > max_speed*2.7) {
			velocity -= constant::SPEED_INCREMENT;
		}
		else {
			velocity += constant::SPEED_INCREMENT;
		}
		if (velocity > constant::MAX_SPEED) {
			velocity = constant::MAX_SPEED;
		}
	}

	if (velocity < 0) {
		velocity = 0;
	}
	ref_vel = velocity;
}

void PlannerFSM::realize_keep_lane(map<int, vector<prediction>> predictions) {
	proposed_lane = autonumous_car.lane;
	_update_ref_speed_for_lane(predictions, proposed_lane);
}

void PlannerFSM::realize_lane_change(map<int, vector<prediction>> predictions, string direction) {
	int delta = -1;
	if (direction.compare("R") == 0)
	{
		delta = 1;
	}
	autonumous_car.lane += delta;
	proposed_lane = autonumous_car.lane;
	_update_ref_speed_for_lane(predictions, proposed_lane);
}

void PlannerFSM::realize_prep_lane_change(map<int, vector<prediction>> predictions, string direction) {
	int delta = -1;
	bool close = false;
	if (direction.compare("R") == 0)
	{
		delta = 1;
	}
	proposed_lane = autonumous_car.lane + delta;
	vector<vector<prediction>> at_behind;
	for (auto pair : predictions) {
		int v_id = pair.first;
		vector<prediction> v = pair.second;
		if (autonumous_car.is_in_front_of(v[0], proposed_lane)) {
			at_behind.push_back(v);
		}
		if (autonumous_car.is_close_to(v[0], autonumous_car.lane)) {
			if (v[0].get_distance(autonumous_car.x, autonumous_car.y, autonumous_car.s) < 4) {
				close = true;
			}
		}
	}
	if (at_behind.size() > 0)
	{
		double velocity = ref_vel;
		if (close) {
			if (velocity > 40.0) {
				velocity -= 2 * constant::SPEED_INCREMENT;
			}
			else {
				velocity -= constant::SPEED_INCREMENT;
			}
		}
		else {
			velocity += constant::SPEED_INCREMENT;
		}
		if (velocity > constant::MAX_SPEED) {
			velocity = constant::MAX_SPEED;
		}
		ref_vel = velocity;
	}
}

double PlannerFSM::get_expected_velocity() {
	return ref_vel;
}
void PlannerFSM::setwaypoints(vector<double> &map_waypoints_x, vector<double> &map_waypoints_y,
	vector<double> &map_waypoints_s, vector<double> &map_waypoints_dx,
	vector<double> &map_waypoints_dy)
{
	this->map_waypoints_x = map_waypoints_x;
	this->map_waypoints_y = map_waypoints_y;
	this->map_waypoints_dx = map_waypoints_dx;
	this->map_waypoints_dy = map_waypoints_dy;
	this->map_waypoints_s = map_waypoints_s;
}