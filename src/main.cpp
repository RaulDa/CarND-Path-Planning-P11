#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "PathPlanner.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

// behavior planning (state machine) ------------------------------------------------------
// use sensor fusion data to avoid collision with front car and change lanes when necessary
// input: sensor_fusion, lane, car_s, prev_size
// output: ref_vel, lane
void behavior_planning_(vector<vector<double>> sensor_fusion, double car_s, double car_d, int prev_size, int *lane, double *ref_vel){

	static int state = 0; //0 = keep_lane, 1 = prep. change lane, 2 = change left, 3 = change right
	static bool first_deceleration = false;

	static double speed_front;
	static int counter_freeze_speed_front = 0;

	static vector<double> speeds_center;
	static vector<double> speeds_left;
	static vector<double> speeds_right;

	bool change_center_safe = false;
	bool change_left_safe = false;
	bool change_right_safe = false;

	bool prepared = false;

	bool too_close = false;
	double mean_speed = 0;

	int *lane_pt = lane;
	double *ref_vel_pt = ref_vel;

	static double dist = 0;

  	switch(state){

  		case(0): //keep_lane
		    // find rev_v to use
		  	for(int i = 0; i < sensor_fusion.size(); i++){
		  	//for(int i = 0; i < 2; i++){
		  		// car is in my lane
		  		float d = sensor_fusion[i][6];
		  		if(d < (2+4*(*lane_pt)+2) && d > (2+4*(*lane_pt)-2)){
		  			double vx = sensor_fusion[i][3];
		  			double vy = sensor_fusion[i][4];
		  			double check_speed = sqrt(vx*vx+vy*vy);
		  			double check_car_s = sensor_fusion[i][5];

		  			// if using previous points can project s value out
		  			// predict s coordinate in future for other car
		  			check_car_s += ((double)prev_size*.02*check_speed);

		  			// check s values greater than mine and s gap (30m)
		  			if((check_car_s > car_s) && ((check_car_s-car_s) < 30)){
		  				// Do some logic here, lower reference velocity so we do not crash into the
		  				// car in front of us, could also flag to try to change lanes
		  				//ref_vel = 29.5; // mph
		  				//too_close = true;

		  				//if(*lane_pt > 0){
		  				//	*lane_pt = 0;
		  				//}
		  				speed_front = check_speed;
		  				state = 1; // prep. change lane
		  				break;
		  			}
		  		}
		  	}

  			if((*ref_vel_pt) < 49.5){

  		  	    if (first_deceleration){
  		  	    	*ref_vel_pt += .224;//.224;
  		  	    }
  		  	    else{
  		  	    	*ref_vel_pt +=.560;//224;
  		  	    }
  		  	}

  		    /*
  	  		static bool first_deceleration = false;

  	  		if(too_close){
  	  			*ref_vel_pt -= .168;//.224;
  	  			first_deceleration = true;
  	  		}
  	  		else if(*ref_vel_pt < 49.5){

  	  			if (first_deceleration){
  	  				*ref_vel_pt += .168;//.224;
  	  			}
  	  			else{
  	  				*ref_vel_pt +=.560;//224;
  	  			}
  	  		}
  	  		*/

  			break;

  		case(1): // prep. change lane

			/*change_center_safe = false;
  			change_left_safe = false;
  			change_right_safe = false;
  			too_close = false;*/

			if((*lane_pt) == 0 || (*lane_pt) == 2){
				// find rev_v to use
				change_center_safe = true;
				for(int i = 0; i < sensor_fusion.size(); i++){
				//for(int i = 0; i < 2; i++){
					// car is in my lane
					float d = sensor_fusion[i][6];
					if(d < (2+4+2) && d > (2+4-2)){
			  			double vx = sensor_fusion[i][3];
			  			double vy = sensor_fusion[i][4];
			  			double check_speed = sqrt(vx*vx+vy*vy);
						double check_car_s = sensor_fusion[i][5];

			  			// if using previous points can project s value out
			  			// predict s coordinate in future for other car
			  			check_car_s += ((double)prev_size*.02*check_speed);

						if(car_s-20 < check_car_s && check_car_s < car_s+30){
							change_center_safe = false;
						}

						if((car_s-25 < check_car_s && check_car_s < car_s-20) || (car_s+30 < check_car_s && check_car_s < car_s+35)){
							speeds_center.push_back(check_speed);
						}
					}
				}
			}

			if((*lane_pt) == 1){
				// find rev_v to use
				change_left_safe = true;
				for(int i = 0; i < sensor_fusion.size(); i++){
					//for(int i = 0; i < 2; i++){
					// car is in my lane
					float d = sensor_fusion[i][6];
					if(d < 4){
			  			double vx = sensor_fusion[i][3];
			  			double vy = sensor_fusion[i][4];
			  			double check_speed = sqrt(vx*vx+vy*vy);
						double check_car_s = sensor_fusion[i][5];

			  			// if using previous points can project s value out
			  			// predict s coordinate in future for other car
			  			check_car_s += ((double)prev_size*.02*check_speed);

						if(car_s-20 < check_car_s && check_car_s < car_s+30){
							change_left_safe = false;
						}

						if((car_s-25 < check_car_s && check_car_s < car_s-20) || (car_s+30 < check_car_s && check_car_s < car_s+35)){
							speeds_left.push_back(check_speed);
						}
					}
				}

				// find rev_v to use
				change_right_safe = true;
				for(int i = 0; i < sensor_fusion.size(); i++){
					//for(int i = 0; i < 2; i++){
					// car is in my lane
					float d = sensor_fusion[i][6];
					if(d > 8){
			  			double vx = sensor_fusion[i][3];
			  			double vy = sensor_fusion[i][4];
			  			double check_speed = sqrt(vx*vx+vy*vy);
						double check_car_s = sensor_fusion[i][5];

			  			// if using previous points can project s value out
			  			// predict s coordinate in future for other car
			  			check_car_s += ((double)prev_size*.02*check_speed);

						if(car_s-20 < check_car_s && check_car_s < car_s+30){
							change_right_safe = false;
						}

						if((car_s-25 < check_car_s && check_car_s < car_s-20) || (car_s+30 < check_car_s && check_car_s < car_s+35)){
							speeds_right.push_back(check_speed);
						}
					}
				}
			}

			if(change_center_safe){
				if(speeds_center.empty()){
					*lane_pt = 1;
					state = 2;
					prepared = true;
				}
				else{

					mean_speed = 0;

	  				for(int i = 0; i < speeds_center.size(); i++){
	  				       mean_speed += speeds_center[i];
	  				}
	  				mean_speed = mean_speed/speeds_center.size();

	  				if(mean_speed-2 < (*ref_vel_pt)/2.24 && (*ref_vel_pt)/2.24 < mean_speed+2){
						*lane_pt = 1;
						state = 2;
						prepared = true;
	  				}
	  				else{
	  					prepared = false;
	  				}
				}

				speeds_left.clear();
				speeds_center.clear();
				speeds_right.clear();
			}
			else if(change_left_safe){
				if(speeds_left.empty()){
					*lane_pt = 0;
					state = 3;
					prepared = true;
				}
				else if(change_right_safe && speeds_right.empty()){
					*lane_pt = 2;
					state = 4;
					prepared = true;
				}
				else{

					mean_speed = 0;

	  				for(int i = 0; i < speeds_left.size(); i++){
	  				       mean_speed += speeds_left[i];
	  				}
	  				mean_speed = mean_speed/speeds_left.size();

	  				if(mean_speed-2 < (*ref_vel_pt)/2.24 && (*ref_vel_pt)/2.24 < mean_speed+2){
						*lane_pt = 0;
						state = 3;
						prepared = true;
	  				}
	  				else{
	  					prepared = false;
	  				}
				}

				speeds_left.clear();
				speeds_center.clear();
				speeds_right.clear();
			}
			else if(change_right_safe){
				if(speeds_right.empty()){
					*lane_pt = 2;
					state = 4;
					prepared = true;
				}
				else{
					mean_speed = 0;

	  				for(int i = 0; i < speeds_right.size(); i++){
	  				       mean_speed += speeds_right[i];
	  				}
	  				mean_speed = mean_speed/speeds_right.size();

	  				if(mean_speed-2 < (*ref_vel_pt)/2.24 && (*ref_vel_pt)/2.24 < mean_speed+2){
						*lane_pt = 2;
						state = 4;
						prepared = true;
	  				}
	  				else{
	  					prepared = false;
	  				}
				}

				speeds_left.clear();
				speeds_center.clear();
				speeds_right.clear();
			}

			if(!prepared){
				// find rev_v to use
				for(int i = 0; i < sensor_fusion.size(); i++){
				//for(int i = 0; i < 2; i++){
					// car is in my lane
					float d = sensor_fusion[i][6];
					if(d < (2+4*(*lane_pt)+2) && d > (2+4*(*lane_pt)-2)){
						double vx = sensor_fusion[i][3];
						double vy = sensor_fusion[i][4];
						double check_speed = sqrt(vx*vx+vy*vy);
						double check_car_s = sensor_fusion[i][5];

						// if using previous points can project s value out
						// predict s coordinate in future for other car
						check_car_s += ((double)prev_size*.02*check_speed);

						// check s values greater than mine and s gap (30m)
						if((check_car_s > car_s) && ((check_car_s-car_s) < 30)){
							// Do some logic here, lower reference velocity so we do not crash into the
							// car in front of us, could also flag to try to change lanes
							//ref_vel = 29.5; // mph
							speed_front = check_speed;
							dist = check_car_s-car_s;
							too_close = true;

							//if(*lane_pt > 0){
							//	*lane_pt = 0;
							//}
						}
					}
				}

	  	    	if(too_close){
	  	    		if((*ref_vel_pt)/2.24 > speed_front-1){
	  	    		*ref_vel_pt -= .336 +dist*0.015;//.224;
	  	    		}
	  	    	  	first_deceleration = true;
	  	    	}
	  	    	else if((*ref_vel_pt)/2.24 < speed_front+1){

	  	    	  	if (first_deceleration){
	  	    	  		*ref_vel_pt += .224;//.224;
	  	    	  	}
	  	    	  	else{
	  	    	  		*ref_vel_pt +=.560;//224;
	  	    	  	}

	  	    	  counter_freeze_speed_front++;
	  	    	  if (counter_freeze_speed_front == 100000){
	  	    		  speed_front = 49.5/2.24;
	  	    		  counter_freeze_speed_front = 0;
	  	    		  cout << "counter reset!!" << endl;
	  	    	  }
	  	        }

				speeds_left.clear();
				speeds_center.clear();
				speeds_right.clear();
			}

  	    	break;

  		case(2):

			if(4 < car_d && car_d < 8){
				state = 0;
			}
			else{

			}
/*
			mean_speed = 0;

  			if(speeds_center.empty()){
  				*lane_pt = 1;
  				state = 0;
  				speeds_center.clear();
  			}
  			else{
  				for(int i = 0; i < speeds_center.size(); i++){
  				       mean_speed += speeds_center[i];
  				}
  				mean_speed = mean_speed/speeds_center.size();

  	  			if(*ref_vel_pt > mean_speed && *ref_vel_pt > mean_speed+1){
  	  		  	    *ref_vel_pt -=.212;//224;
  	  		  	}
  	  			else if(*ref_vel_pt < mean_speed && *ref_vel_pt < mean_speed-1){
  	  				*ref_vel_pt +=.212;
  	  			}
  	  			else{
  	  				*lane_pt = 1;
  	  				state = 0;
  	  				speeds_center.clear();
  	  			}
  			}*/
  			break;

  		case(3):
					if(0 < car_d && car_d < 4){
						state = 0;
					}
					else{

					}
/*
			mean_speed = 0;

  			if(speeds_left.empty()){
  				*lane_pt = 0;
  				state = 0;
  				speeds_left.clear();
  			}
  			else{
  				for(int i = 0; i < speeds_left.size(); i++){
  				       mean_speed += speeds_left[i];
  				}
  				mean_speed = mean_speed/speeds_left.size();

  	  			if(*ref_vel_pt > mean_speed && *ref_vel_pt > mean_speed+1){
  	  		  	    *ref_vel_pt -=.212;//224;
  	  		  	}
  	  			else if(*ref_vel_pt < mean_speed && *ref_vel_pt < mean_speed-1){
  	  				*ref_vel_pt +=.212;
  	  			}
  	  			else{
  	  				*lane_pt = 0;
  	  				state = 0;
  	  				speeds_left.clear();
  	  			}
  			}*/
  			break;

  		case(4):
							if(8 < car_d && car_d < 12){
								state = 0;
							}
							else{

							}
/*
			mean_speed = 0;

  			if(speeds_right.empty()){
  				*lane_pt = 2;
  				state = 0;
  				speeds_right.clear();
  			}
  			else{
  				for(int i = 0; i < speeds_right.size(); i++){
  				       mean_speed += speeds_right[i];
  				}
  				mean_speed = mean_speed/speeds_right.size();

  	  			if(*ref_vel_pt > mean_speed && *ref_vel_pt > mean_speed+1){
  	  		  	    *ref_vel_pt -=.212;//224;
  	  		  	}
  	  			else if(*ref_vel_pt < mean_speed && *ref_vel_pt < mean_speed-1){
  	  				*ref_vel_pt +=.212;
  	  			}
  	  			else{
  	  				*lane_pt = 2;
  	  				state = 0;
  	  				speeds_right.clear();
  	  			}
  			}*/
  			break;


  	}

}


int main() {
  uWS::Hub h;

  // Create path planner
  PathPlanner path_plan;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  // 0 -> left lane, 1 -> middle lane, 2 -> right lane
  int lane = 1;

  // reference velocity to target in mph
  double ref_vel = 0;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &path_plan](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];
          	vector<vector<double>> sensor_fusion_ = sensor_fusion;

          	int prev_size = previous_path_x.size();

            // store last s value if previous path available
          	if(prev_size > 0){
          		car_s = end_path_s;
          	}

          	// behavior planning (state machine) ------------------------------------------------------
          	// use sensor fusion data to avoid collision with front car and change lanes when necessary
          	// input: sensor_fusion, lane, car_s, prev_size
          	// output: ref_vel, lane

          	path_plan.behavior_planning(sensor_fusion_, car_s, car_d, prev_size, &lane, &ref_vel);

          	// create 5 spline points-------------------------------------------------
          	// two first points depending on available path (previous point and actual)
          	// three next points at distances 30, 60 and 90m from actual s coordinate
          	// if available, we take last two points of previous_path
          	// if not, we create two taking car actual position as starting reference
          	// points converted to local coordinate system at the end
          	// inputs: prev_size, car_x, car_y, car_yaw, previous_path, lane
          	// output: ptsx, ptsy (5 points), ref_x, ref_y, ref_yaw (gen. traj. needs it for conv. coord)

          	// Create list of widely spaced waypoints (30m)
          	vector<double> ptsx;
          	vector<double> ptsy;

          	// Reference x, y, yaw states
          	double ref_x = car_x;
          	double ref_y = car_y;
          	double ref_yaw = deg2rad(car_yaw);

          	// if previous size is almost empty, use car as starting reference
          	if (prev_size<2){
          		// Use two points that make the path tangent to the car
          		double prev_car_x = ref_x - cos(car_yaw);
          		double prev_car_y = ref_y - sin(car_yaw);

          		ptsx.push_back(prev_car_x);
          		ptsx.push_back(ref_x);

          		ptsy.push_back(prev_car_y);
          		ptsy.push_back(ref_y);
          	}
          	// use previous path´s end point as starting reference
          	else
          	{
          		// redefine reference state as previous path end point
          		ref_x = previous_path_x[prev_size-1];
          		ref_y = previous_path_y[prev_size-1];

          		double ref_x_prev = previous_path_x[prev_size-2];
          		double ref_y_prev = previous_path_y[prev_size-2];
          		ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

          		// use two points that make the path tangent to the previous path´s and point
          		ptsx.push_back(ref_x_prev);
          		ptsx.push_back(ref_x);

          		ptsy.push_back(ref_y_prev);
          		ptsy.push_back(ref_y);

          	}

          	// in Frenet add 30m spaced points ahead of the starting reference
          	vector<double> next_wp0 = getXY(car_s+30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp1 = getXY(car_s+60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp2 = getXY(car_s+90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          	ptsx.push_back(next_wp0[0]);
          	ptsx.push_back(next_wp1[0]);
          	ptsx.push_back(next_wp2[0]);

          	ptsy.push_back(next_wp0[1]);
          	ptsy.push_back(next_wp1[1]);
          	ptsy.push_back(next_wp2[1]);

          	for (int i=0; i<ptsx.size(); i++){
          		// shift car reference angle to 0 degrees
          		double shift_x = ptsx[i] - ref_x;
          		double shift_y = ptsy[i] - ref_y;

          		ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
          		ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          	}

          	// generate trajectory ----------------------------------------------------------------
          	// from 5 spline points create rest of path planning points applying spline function
          	// number of points depending on ref_vel
          	// first points will be previous_path ones and calculated spline ones are added after those
          	// inputs: ptsx, ptsy, previous_path, ref_vel..... ref_x, ref_y, ref_yaw (conv. coord.)
          	// output: next_x_vals, next_y_vals

          	// create spline
          	tk::spline s;

          	// set points to spline
          	s.set_points(ptsx, ptsy);

          	// define actual points we will use for the planner
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	// start with all of the previous path points from last time
          	for(int i = 0; i < previous_path_x.size(); i++)
          	{
          	      next_x_vals.push_back(previous_path_x[i]);
          	      next_y_vals.push_back(previous_path_y[i]);
          	}

          	// calculate how to break up spline points so that we travel at our desired ref. velocity
          	double target_x = 30.0;
          	double target_y = s(target_x);
          	double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

          	double x_add_on = 0;

          	// fill up rest of path planner after filling it with previous points
          	for (int i = 1; i <= 50-previous_path_x.size(); i++){

          		double N = (target_dist/(.02*ref_vel/2.24));
          		double x_point = x_add_on + target_x/N;
          		double y_point = s(x_point);

          		x_add_on = x_point;

          		double x_ref = x_point;
          		double y_ref = y_point;

          		// rotate back to normal after rotating it earlier
          		x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
          		y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

          		x_point += ref_x;
          		y_point += ref_y;

          		next_x_vals.push_back(x_point);
          		next_y_vals.push_back(y_point);

          	}

          	//cout << next_x_vals.size() << endl;

          	/*double dist_inc = 0.3;
          	for(int i = 0; i < 50; i++)
          	{
          		  double next_s = car_s + (i+1)*dist_inc;
          		  double next_d = 6;
          		  vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          		  next_x_vals.push_back(xy[0]);
          		  next_y_vals.push_back(xy[1]);
          	      //next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
          	      //next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
          	}*/

          	/*double pos_x;
          	double pos_y;
          	double angle;
          	int path_size = previous_path_x.size();

          	for(int i = 0; i < path_size; i++)
          	{
          	    next_x_vals.push_back(previous_path_x[i]);
          	    next_y_vals.push_back(previous_path_y[i]);
          	}

          	if(path_size == 0)
          	{
          	    pos_x = car_x;
          	    pos_y = car_y;
          	    angle = deg2rad(car_yaw);
          	}
          	else
          	{
          	    pos_x = previous_path_x[path_size-1];
          	    pos_y = previous_path_y[path_size-1];

          	    double pos_x2 = previous_path_x[path_size-2];
          	    double pos_y2 = previous_path_y[path_size-2];
          	    angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
          	}

          	double dist_inc = 0.5;
          	for(int i = 0; i < 50-path_size; i++)
          	{
          	    next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
          	    next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
          	    pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
          	    pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
          	}*/

          	json msgJson;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
