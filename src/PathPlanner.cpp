/*
 * PathPlanner.cpp
 *
 *  Created on: May 26, 2018
 *  Author: Raul Davila / Udacity
 */

#include "PathPlanner.h"
#include "spline.h"

#include <vector>
#include <math.h>
#include <iostream>

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between points
double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Get closest waypoint
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

// Get next waypoint
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

// Behavior planning routine (See detailed description in header file)
void PathPlanner::behavior_planning(vector<vector<double>> sensor_fusion, carCoordinates car_coord, int prev_size, int *lane, double *ref_vel) {

	static int state = keepLane; // 0 = keep lane, 1 = prep. change lane,
								 // 2 = change center, 3 = change left, 4 = change right
	static bool first_deceleration = false;

	// Speed of front vehicle and counter
	static double speed_front;
	static int counter_freeze_speed_front = 0;

	// Speeds of near vehicles on desired change lane
	static vector<double> speeds_center;
	static vector<double> speeds_left;
	static vector<double> speeds_right;

	// Mean speed of near vehicles on desired change lane
	double mean_speed = 0;

	// Flags for activating lane change
	bool change_center_safe = false;
	bool change_left_safe = false;
	bool change_right_safe = false;

	// True if the lane change is safe
	bool prepared = false;

	// True if ego is closer than front safe distance
	bool too_close = false;

	// Pointers of outputs
	int *lane_pt = lane;
	double *ref_vel_pt = ref_vel;

	// Distance to front car. Measured for adjusting acceleration in function of it
	static double dist = 0;

  	switch(state){

  		case(keepLane):
		    // Iterate over all cars on the road
		  	for(int i = 0; i < sensor_fusion.size(); i++){
		  		float d = sensor_fusion[i][6];
		  		// Check if car is in our lane
		  		if(d < (lane_width+lane_width*(*lane_pt)) && d > (lane_width*(*lane_pt))){
		  			// Get s and speed
		  			double vx = sensor_fusion[i][3];
		  			double vy = sensor_fusion[i][4];
		  			double check_speed = sqrt(vx*vx+vy*vy);
		  			double check_car_s = sensor_fusion[i][5];

		  			// If using previous points can project s value out
		  			check_car_s += ((double)prev_size*.02*check_speed);

		  			// Check s values greater than mine and s gap
		  			if((check_car_s > car_coord.car_s) && ((check_car_s-car_coord.car_s) < safe_distance_front)){
		  				// Store speed of front vehicle
		  				speed_front = check_speed;
		  				// State change
		  				state = prepareChangeLane;
		  				break;
		  			}
		  		}
		  	}

  		    // Speed adjustment
  			if((*ref_vel_pt) < max_speed){

  				// Greater acceleration for start of simulation
  		  	    if (first_deceleration){
  		  	    	*ref_vel_pt += acceleration;
  		  	    }
  		  	    else{
  		  	    	*ref_vel_pt += first_acceleration;
  		  	    }
  		  	}

  			break;

  		case(prepareChangeLane):

		    // If we are in border lane, check if conditions to change to center lane are fulfilled
			if((*lane_pt) == leftLane || (*lane_pt) == rightLane){
				// Set flag that allows change to center initially to True
				change_center_safe = true;
				// Check all cars on road
				for(int i = 0; i < sensor_fusion.size(); i++){
					// Check if car is in center lane
					float d = sensor_fusion[i][6];
					if(d < 2*lane_width && d > lane_width){
						// Get speed and s coordinate
			  			double vx = sensor_fusion[i][3];
			  			double vy = sensor_fusion[i][4];
			  			double check_speed = sqrt(vx*vx+vy*vy);
						double check_car_s = sensor_fusion[i][5];

			  			// If using previous points can project s value out
			  			check_car_s += ((double)prev_size*.02*check_speed);

			  			// If there is a car in desired lane at [-safe_distance_back, safe_distance_front] (being 0m my current car s reference), change is not safe
						if(car_coord.car_s-safe_distance_back < check_car_s && check_car_s < car_coord.car_s+safe_distance_front){
							change_center_safe = false;
						}

						// If there is a car in desired lane at [-safe_distance_back_vel_check, -safe_distance_back] or [safe_distance_front, safe_distance_front_vel_check] (being 0m my current car s reference), store speeds of these vehicles
						if((car_coord.car_s-safe_distance_back_vel_check < check_car_s && check_car_s < car_coord.car_s-safe_distance_back) || (car_coord.car_s+safe_distance_front < check_car_s && check_car_s < car_coord.car_s+safe_distance_front_vel_check)){
							speeds_center.push_back(check_speed);
						}
					}
				}
			}

  		    // Check whether we are in center lane
			if((*lane_pt) == centerLane){
				// Set flag that allows change to left initially to True
				change_left_safe = true;
				// Check all cars on road
				for(int i = 0; i < sensor_fusion.size(); i++){
					// Check whether car is in left lane
					float d = sensor_fusion[i][6];
					if(d < lane_width){
						// Store speed and s coordinate
						double vx = sensor_fusion[i][3];
			  			double vy = sensor_fusion[i][4];
			  			double check_speed = sqrt(vx*vx+vy*vy);
						double check_car_s = sensor_fusion[i][5];

			  			// if using previous points can project s value out
			  			check_car_s += ((double)prev_size*.02*check_speed);

			  			// If there is a car in desired lane at [-safe_distance_back, safe_distance_front] (being 0m my current car s reference), change is not safe
						if(car_coord.car_s-safe_distance_back < check_car_s && check_car_s < car_coord.car_s+safe_distance_front){
							change_left_safe = false;
						}

						// If there is a car in desired lane at [-safe_distance_back_vel_check, -safe_distance_back] or [safe_distance_front, safe_distance_front_vel_check] (being 0m my current car s reference), store speeds of these vehicles
						if((car_coord.car_s-safe_distance_back_vel_check < check_car_s && check_car_s < car_coord.car_s-safe_distance_back) || (car_coord.car_s+safe_distance_front < check_car_s && check_car_s < car_coord.car_s+safe_distance_front_vel_check)){
							speeds_left.push_back(check_speed);
						}
					}
				}

				// Set flag that allows change to right initially to True
				change_right_safe = true;
				// Check all cars on road
				for(int i = 0; i < sensor_fusion.size(); i++){
					// Check whether car is in right lane
					float d = sensor_fusion[i][6];
					if(d > 2*lane_width){
						// Store speed and s coordinate
			  			double vx = sensor_fusion[i][3];
			  			double vy = sensor_fusion[i][4];
			  			double check_speed = sqrt(vx*vx+vy*vy);
						double check_car_s = sensor_fusion[i][5];

			  			// If using previous points can project s value out
			  			check_car_s += ((double)prev_size*.02*check_speed);

			  			// If there is a car in desired lane at [-safe_distance_back, safe_distance_front] (being 0m my current s car reference), change is not safe
						if(car_coord.car_s-safe_distance_back < check_car_s && check_car_s < car_coord.car_s+safe_distance_front){
							change_right_safe = false;
						}

						// If there is a car in desired lane at [-safe_distance_back_vel_check, -safe_distance_back] or [safe_distance_front, safe_distance_front_vel_check] (being 0m my current s car reference), store speeds of these vehicles
						if((car_coord.car_s-safe_distance_back_vel_check < check_car_s && check_car_s < car_coord.car_s-safe_distance_back) || (car_coord.car_s+safe_distance_front < check_car_s && check_car_s < car_coord.car_s+safe_distance_front_vel_check)){
							speeds_right.push_back(check_speed);
						}
					}
				}
			}

			// If no car in [-safe_distance_back,safe_distance_front] of desired lane (0m is my actual car s reference)
			if(change_center_safe){
				// If no car in [-safe_distance_back_vel_check, -safe_distance_back], [safe_distance_front,safe_distance_front_vel_check]
				if(speeds_center.empty()){
					// Change to center lane
					*lane_pt = centerLane;
					// State change
					state = changeCenterLane;
					// Avoid velocity adjustment in this cycle
					prepared = true;
				}
				else{
					// Calculate mean speed of cars in [-safe_distance_back_vel_check, -safe_distance_back], [safe_distance_front,safe_distance_front_vel_check]
					mean_speed = 0;
	  				for(int i = 0; i < speeds_center.size(); i++){
	  				       mean_speed += speeds_center[i];
	  				}
	  				mean_speed = mean_speed/speeds_center.size();

	  				// If mean speed and my speed have difference of [-safe_speed_difference,safe_speed_difference] change is safe
	  				if(mean_speed-safe_speed_difference < (*ref_vel_pt)/2.24 && (*ref_vel_pt)/2.24 < mean_speed+safe_speed_difference){
	  					// Change to center lane
						*lane_pt = centerLane;
						// State change
						state = changeCenterLane;
						// Avoid velocity adjustment in this cycle
						prepared = true;
	  				}
	  				else{
	  					// No lane change in this cycle, so adjust velocity
	  					prepared = false;
	  				}
				}

				// Clear speed vectors
				speeds_left.clear();
				speeds_center.clear();
				speeds_right.clear();
			}
			// If no car in [-safe_distance_back,safe_distance_front] of left lane (0m is my actual car s reference)
			else if(change_left_safe){
				// If no car in [-safe_distance_back_vel_check, -safe_distance_back], [safe_distance_front,safe_distance_front_vel_check]
				if(speeds_left.empty()){
					// Change to left lane
					*lane_pt = leftLane;
					// State change
					state = changeLeftLane;
					// Avoid velocity adjustment in this cycle
					prepared = true;
				}
				// If no car in [-safe_distance_back_vel_check, -safe_distance_back], [safe_distance_front,safe_distance_front_vel_check] of right lane
				else if(change_right_safe && speeds_right.empty()){
					// Change to right lane
					*lane_pt = rightLane;
					// State change
					state = changeRightLane;
					// Avoid velocity adjustment in this cycle
					prepared = true;
				}
				else{
					// Calculate mean speed of cars in [-safe_distance_back_vel_check, -safe_distance_back], [safe_distance_front,safe_distance_front_vel_check]
					mean_speed = 0;
	  				for(int i = 0; i < speeds_left.size(); i++){
	  				       mean_speed += speeds_left[i];
	  				}
	  				mean_speed = mean_speed/speeds_left.size();

	  				// If mean speed and my speed have difference of [-safe_speed_difference,safe_speed_difference] change is safe
	  				if(mean_speed-safe_speed_difference < (*ref_vel_pt)/2.24 && (*ref_vel_pt)/2.24 < mean_speed+safe_speed_difference){
	  					// Change to left lane
						*lane_pt = leftLane;
						// State change
						state = changeLeftLane;
						// Avoid velocity adjustment in this cycle
						prepared = true;
	  				}
	  				else{
	  					// No lane change in this cycle, so adjust velocity
	  					prepared = false;
	  				}
				}

				// Clear speed vectors
				speeds_left.clear();
				speeds_center.clear();
				speeds_right.clear();
			}
			// If no car in [-safe_distance_back,safe_distance_front] of left lane (0m is my actual car s reference)
			else if(change_right_safe){
				// If no car in [-safe_distance_back_vel_check, -safe_distance_back], [safe_distance_front,safe_distance_front_vel_check]
				if(speeds_right.empty()){
					// Change to right lane
					*lane_pt = rightLane;
					// State change
					state = changeRightLane;
					// Avoid velocity adjustment in this cycle
					prepared = true;
				}
				else{
					// Calculate mean speed of cars in [-safe_distance_back_vel_check, -safe_distance_back], [safe_distance_front,safe_distance_front_vel_check]
					mean_speed = 0;
	  				for(int i = 0; i < speeds_right.size(); i++){
	  				       mean_speed += speeds_right[i];
	  				}
	  				mean_speed = mean_speed/speeds_right.size();

	  				// If mean speed and my speed have difference of [-safe_speed_difference,safe_speed_difference] change is safe
	  				if(mean_speed-safe_speed_difference < (*ref_vel_pt)/2.24 && (*ref_vel_pt)/2.24 < mean_speed+safe_speed_difference){
	  					// Change to right lane
						*lane_pt = rightLane;
						// State change
						state = changeRightLane;
						// Avoid velocity adjustment in this cycle
						prepared = true;
	  				}
	  				else{
	  					// No lane change in this cycle, so adjust velocity
	  					prepared = false;
	  				}
				}

				// Clear speed vectors
				speeds_left.clear();
				speeds_center.clear();
				speeds_right.clear();
			}

			// No state change in this cycle. Prepare velocity adjustment
			if(!prepared){
				// Check all cars on the road
				for(int i = 0; i < sensor_fusion.size(); i++){
					float d = sensor_fusion[i][6];
					// Check whether car on my lane
					if(d < (lane_width+lane_width*(*lane_pt)) && d > (lane_width*(*lane_pt))){
						// Measure velocity and s coordinate
						double vx = sensor_fusion[i][3];
						double vy = sensor_fusion[i][4];
						double check_speed = sqrt(vx*vx+vy*vy);
						double check_car_s = sensor_fusion[i][5];

						// If using previous points can project s value out
						check_car_s += ((double)prev_size*.02*check_speed);

						// Check s values greater than mine and s gap
						if((check_car_s > car_coord.car_s) && ((check_car_s-car_coord.car_s) < safe_distance_front)){
							// Store speed of front vehicle
							speed_front = check_speed;
							// Store distance to front vehicle
							dist = check_car_s-car_coord.car_s;
							// Too close flag to true
							too_close = true;
						}
					}
				}

				// Check whether car in front too close
	  	    	if(too_close){
	  	    		// Check whether velocity greater than front car velocity
	  	    		if((*ref_vel_pt)/2.24 > speed_front-speed_interval){
	  	    			// Deceleration in function of vehicle distance
	  	    			*ref_vel_pt -= deceleration_intercept + dist*deceleration_slope;
	  	    		}
	  	    	  	first_deceleration = true;
	  	    	}
	  	    	// Check whether velocity lower than front car velocity
	  	    	else if((*ref_vel_pt)/2.24 < speed_front+speed_interval){

	  	    		// Acceleration
	  	    	  	if (first_deceleration){
	  	    	  		*ref_vel_pt += acceleration;
	  	    	  	}
	  	    	  	else{
	  	    	  		*ref_vel_pt += first_acceleration;
	  	    	  	}

	  	    	    // Check freeze speed and reset counter if necessary
	  	    	  	counter_freeze_speed_front++;
	  	    	  	if (counter_freeze_speed_front == counter_reset){
	  	    	  		speed_front = max_speed/2.24;
	  	    	  		counter_freeze_speed_front = 0;
	  	    	  	}
	  	        }

	  	    	// Clear speed vectors
				speeds_left.clear();
				speeds_center.clear();
				speeds_right.clear();
			}

  	    	break;

  		case(changeCenterLane):

			// Check whether car is already in desired lane
			if(lane_width < car_coord.car_d && car_coord.car_d < 2*lane_width){
				// State change
				state = keepLane;
			}
			else{

			}
  			break;

  		case(changeLeftLane):

			// Check whether car is already in desired lane
			if(0 < car_coord.car_d && car_coord.car_d < lane_width){
				// State change
				state = keepLane;
			}
			else{

			}
  			break;

  		case(changeRightLane):

		    // Check whether car is already in desired lane
			if(2*lane_width < car_coord.car_d && car_coord.car_d < 3*lane_width){
				// State change
				state = keepLane;
			}
			else{

			}
  			break;
  		default:
  			break;

  	}

}

// Generate spline points (See detailed description in header file)
splinePointsReturn PathPlanner::generate_spline_points(int prev_size, carCoordinates car_coord, previousPath previous_path, int lane, vector<vector<double>> map_waypoints){

	splinePointsReturn ret_value;

  	ret_value.ref_x = car_coord.car_x;
  	ret_value.ref_y = car_coord.car_y;
  	ret_value.ref_yaw = car_coord.car_yaw;

  	// If previous size is almost empty, use car as starting reference
  	if (prev_size<2){
  		// Use two points that make the path tangent to the car
  		double prev_car_x = ret_value.ref_x - cos(car_coord.car_yaw);
  		double prev_car_y = ret_value.ref_y - sin(car_coord.car_yaw);

  		ret_value.ptsx.push_back(prev_car_x);
  		ret_value.ptsx.push_back(ret_value.ref_x);

  		ret_value.ptsy.push_back(prev_car_y);
  		ret_value.ptsy.push_back(ret_value.ref_y);
  	}
  	// Use previous path´s end point as starting reference
  	else
  	{
  		// Redefine reference state as previous path end point
  		ret_value.ref_x = previous_path.previous_path_x[prev_size-1];
  		ret_value.ref_y = previous_path.previous_path_y[prev_size-1];

  		double ref_x_prev = previous_path.previous_path_x[prev_size-2];
  		double ref_y_prev = previous_path.previous_path_y[prev_size-2];
  		ret_value.ref_yaw = atan2(ret_value.ref_y-ref_y_prev, ret_value.ref_x-ref_x_prev);

  		// Use two points that make the path tangent to the previous path´s and point
  		ret_value.ptsx.push_back(ref_x_prev);
  		ret_value.ptsx.push_back(ret_value.ref_x);

  		ret_value.ptsy.push_back(ref_y_prev);
  		ret_value.ptsy.push_back(ret_value.ref_y);

  	}

  	// In Frenet add 30m spaced points ahead of the starting reference
  	vector<double> next_wp0 = getXY(car_coord.car_s+30, 2+4*lane, map_waypoints[0], map_waypoints[1], map_waypoints[2]);
  	vector<double> next_wp1 = getXY(car_coord.car_s+60, 2+4*lane, map_waypoints[0], map_waypoints[1], map_waypoints[2]);
  	vector<double> next_wp2 = getXY(car_coord.car_s+90, 2+4*lane, map_waypoints[0], map_waypoints[1], map_waypoints[2]);

  	ret_value.ptsx.push_back(next_wp0[0]);
  	ret_value.ptsx.push_back(next_wp1[0]);
  	ret_value.ptsx.push_back(next_wp2[0]);

  	ret_value.ptsy.push_back(next_wp0[1]);
  	ret_value.ptsy.push_back(next_wp1[1]);
  	ret_value.ptsy.push_back(next_wp2[1]);

  	for (int i=0; i<ret_value.ptsx.size(); i++){
  		// Shift car reference angle to 0 degrees
  		double shift_x = ret_value.ptsx[i] - ret_value.ref_x;
  		double shift_y = ret_value.ptsy[i] - ret_value.ref_y;

  		ret_value.ptsx[i] = (shift_x * cos(0-ret_value.ref_yaw) - shift_y * sin(0-ret_value.ref_yaw));
  		ret_value.ptsy[i] = (shift_x * sin(0-ret_value.ref_yaw) + shift_y * cos(0-ret_value.ref_yaw));
  	}
  	return ret_value;
}

// Generate trajectory (See detailed description in header file)
vector<vector<double>> PathPlanner::generate_trajectory(splinePointsReturn splinePointsStr, previousPath previous_path, double ref_vel){

	vector<vector<double>> ret_value;

  	// Create spline
  	tk::spline s;

  	// Set points to spline
  	s.set_points(splinePointsStr.ptsx, splinePointsStr.ptsy);

  	// Define actual points we will use for the planner
  	vector<double> next_x_vals;
  	vector<double> next_y_vals;

  	// Start with all of the previous path points from last time
  	for(int i = 0; i < previous_path.previous_path_x.size(); i++)
  	{
  	      next_x_vals.push_back(previous_path.previous_path_x[i]);
  	      next_y_vals.push_back(previous_path.previous_path_y[i]);
  	}

  	// Calculate how to break up spline points so that we travel at our desired ref. velocity
  	double target_x = 30.0;
  	double target_y = s(target_x);
  	double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

  	double x_add_on = 0;

  	// Fill up rest of path planner after filling it with previous points
  	for (int i = 1; i <= 50-previous_path.previous_path_x.size(); i++){

  		double N = (target_dist/(.02*ref_vel/2.24));
  		double x_point = x_add_on + target_x/N;
  		double y_point = s(x_point);

  		x_add_on = x_point;

  		double x_ref = x_point;
  		double y_ref = y_point;

  		// Rotate back to normal after rotating it earlier
  		x_point = (x_ref * cos(splinePointsStr.ref_yaw)-y_ref*sin(splinePointsStr.ref_yaw));
  		y_point = (x_ref * sin(splinePointsStr.ref_yaw)+y_ref*cos(splinePointsStr.ref_yaw));

  		x_point += splinePointsStr.ref_x;
  		y_point += splinePointsStr.ref_y;

  		next_x_vals.push_back(x_point);
  		next_y_vals.push_back(y_point);

  	}

  	// Return calculated trajectory
  	ret_value.push_back(next_x_vals);
  	ret_value.push_back(next_y_vals);

	return ret_value;
}
