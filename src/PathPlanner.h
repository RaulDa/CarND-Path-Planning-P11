/*
 * PathPlanner.h
 *
 *  Created on: May 26, 2018
 *  Author: Raul Davila / Udacity
 */

#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <vector>

// Collected car coordinates
struct carCoordinates {
	double car_x;
	double car_y;
	double car_s;
	double car_d;
	double car_yaw;
	double car_speed;
};

// x and y coordinates of previous path points
struct previousPath {
	std::vector<double> previous_path_x;
	std::vector<double> previous_path_y;
};

// 5 spline points and references for coordinate system conversion
struct splinePointsReturn {
	std::vector<double> ptsx;
	std::vector<double> ptsy;
	double ref_x;
	double ref_y;
	double ref_yaw;
};

enum State{
	keepLane,
	prepareChangeLane,
	changeCenterLane,
	changeLeftLane,
	changeRightLane
};

enum Lane{
	leftLane,
	centerLane,
	rightLane
};

class PathPlanner {

public:

	// Safe distance front (30m)
	int safe_distance_front = 25;

	// Safe distance back (20m)
	int safe_distance_back = 10;

	// Safe distance front velocity check (35m)
	int safe_distance_front_vel_check = 35;

	// Safe distance back velocity check (25m)
	int safe_distance_back_vel_check = 20;

	// Safe difference with mean speed desired line (2m/s)
	int safe_speed_difference = 4;

	// Maximum velocity (49.5 mph)
	double max_speed = 49.5;

	// Speed balance (1m/s)
	int speed_interval = 1;

	// Lane width (4m)
	int lane_width = 4;

	// Acceleration factor after first deceleration
	double acceleration = .224;

	// Acceleration factor before first deceleration
	double first_acceleration = .560;

	// Deceleration slope (linear equation in function of distance to front car)
	double deceleration_slope = 0.015;

	// Deceleration intercept
	double deceleration_intercept = .336;

	// Counter reset max speed
	int counter_reset = 10000;

	// Constructor
	PathPlanner() {}

	// Destructor
	~PathPlanner() {}

	/**
	 * behavior_planning Uses sensor fusion data to make high level decisions on the road.
	 *   For instance, avoiding collisions with front car, changing lanes when necessary
	 *   or adjusting velocity to front car one if changing lanes is not safe.
	 *     Inputs: sensor_fusion, car_coord, prev_size
	 *     Outputs: lane, ref_vel
	 * @param sensor_fusion Sensor fusion data
	 * @param car_coord Actual car coordinates. Frenet s and d are used
	 * @param prev_size Previous size of path data
	 * @param lane Actual lane
	 * @param ref_vel Car velocity in mph
	 */
	void behavior_planning(std::vector<std::vector<double>> sensor_fusion, carCoordinates car_coord, int prev_size, int *lane, double *ref_vel);

  	/**
  	 * generate_spline_points Create 5 spline points for generating trajectory (Udacity base code)
  	 *  Two first points depending on available path (previous point and actual)
  	 *  Three next points at distances 30, 60 and 90m from actual s coordinate
  	 *  If available, we take last two points of previous_path
  	 *  If not, we create two taking car actual position as starting reference
  	 *  Points converted to local coordinate system at the end
  	 * @param prev_size Previous size of path data
  	 * @param car_coord Actual car coordinates. x, y, yaw and s are used
	 * @param previous_path x and y coordinates of previous points
	 * @param lane Actual lane
	 * @param map_waypoints x, y and s coordinates of map waypoints
	 * @param return the x and y coordinates of the 5 spline points (ptsx, ptsy)
	 *               references needed for coordinate system conversion by generate_trajectory (ref_x, ref_y, ref_yaw)
	*/
	splinePointsReturn generate_spline_points(int prev_size, carCoordinates car_coord, previousPath previous_path, int lane, std::vector<std::vector<double>> map_waypoints);

  	/* generate_trayectory Generates final trajectory (Udacity base code)
  	// From 5 spline points create rest of path planning points applying spline function
  	// Number of points depending on ref_vel
  	// First points will be previous_path ones and calculated spline ones are added after those
  	// @param splinePointsStr x and y coordinates of the 5 spline points (ptsx, ptsy)
  	 *                        references needed for coordinate system conversion by generate_trajectory (ref_x, ref_y, ref_yaw)
  	 * @param previous_path x and y coordinates of previous path
  	 * @param ref_vel Car velocity in mph
  	// @param return x and y coordinates of final trajectory
  	 */
	std::vector<std::vector<double>> generate_trajectory(splinePointsReturn splinePointsStr, previousPath previous_path, double ref_vel);
};



#endif /* PATH_PLANNER_H_ */
