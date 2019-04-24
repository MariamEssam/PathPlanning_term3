#include "PathPlanningManager.h"



PathPlanningManager::PathPlanningManager(vector<double> &map_waypoints_x, vector<double> &map_waypoints_y,
	vector<double> &map_waypoints_s, vector<double> &map_waypoints_dx,
	vector<double> &map_waypoints_dy)
{
	this->map_waypoints_x = map_waypoints_x;
	this->map_waypoints_y = map_waypoints_y;
	this->map_waypoints_dx = map_waypoints_dx;
	this->map_waypoints_dy = map_waypoints_dy;
	this->map_waypoints_s = map_waypoints_s;

}


PathPlanningManager::~PathPlanningManager()
{
}
void PathPlanningManager::updatewatchinglist(json datafusion)
{
	//Clear previous predictions
	predictions.clear();
	//loop on all the fusion data[id, x, y, Vx, Vy, s, d]
	for (auto data : datafusion)
	{
		//Data exisit in the vehicle area
		if (data[5] <= constant::MAX_S &&data[6] >= 0)
		{
			if (watchingvehicles.find(data[0]) == watchingvehicles.end())
			{
				//if the unique vehicle ID doesn't exisit in the watching list
				watchingvehicles[data[0]] = new Vehicle(data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
			}
			else
			{
				Vehicle* l_veh = NULL;
				l_veh=watchingvehicles[data[0]];
				(*l_veh).update_yaw(data[3], data[4]);
				(*l_veh).update_accel(data[3], data[4]);
				(*l_veh).updatewatchingVehiclePos(data[1], data[2], data[5], data[6]);
				vector<prediction> car_preds = (*l_veh).generate_predictions(map_waypoints_x, map_waypoints_y,
			    map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
				predictions[(*l_veh).id] = car_preds;

			}
		}
		else
		{
			auto it = watchingvehicles.find(data[0]);
			if (it != watchingvehicles.end()) 
			{
				delete (*it).second;
				watchingvehicles.erase((int)data[0]);
			}
		}
	}
	/*for (int i = 0; i < watchingvehicles.size(); i++)
	{
		Vehicle* ve= watchingvehicles[i];
		std::cout<<"The ID: "<<ve->id<<" The lane is"<<ve->lane<<endl;
	}*/
}
void PathPlanningManager::update_auto_car_state(double car_s, double x, double y, double yaw, double s, double d, double speed) {
	original_yaw = yaw;
	fsm.car_s = car_s;
	fsm.autonumous_car.update_params(x, y, yaw, s, d, speed);
	//std::cout << "The ID: " << fsm.autonumous_car.id << " The lane is" << fsm.autonumous_car.lane << endl;
}
void PathPlanningManager::generatePath(vector<double> previous_path_x, vector<double> previous_path_y)
{
	//Update way points
	fsm.setwaypoints(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
	//Find the next state that has the minimum cost
	fsm.update_state(predictions);
	fsm.realize_state(predictions);
	traject.setwaypoints(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
	traject.set_previous_path(previous_path_x, previous_path_y);
	traject.generate_trajectory(fsm.car_s, ourvehicle.x, ourvehicle.y, original_yaw, ourvehicle.lane, fsm.get_expected_velocity());
	
}
vector<double> PathPlanningManager::get_x_values() {
	return traject.next_x_vals;
}

vector<double> PathPlanningManager::get_y_values() {
	return traject.next_y_vals;
}