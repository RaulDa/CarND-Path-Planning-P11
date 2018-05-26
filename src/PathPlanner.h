/*
 * PathPlanner.h
 *
 *  Created on: May 26, 2018
 */

#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <vector>


/*
struct Particle {

	int id;
	double x;
	double y;
	double theta;
	double weight;
	std::vector<int> associations;
	std::vector<double> sense_x;
	std::vector<double> sense_y;
};*/



class PathPlanner {

	/*
	// Number of particles to draw
	int num_particles;

	// Flag, if filter is initialized
	bool is_initialized;

	// Vector of weights of all particles
	std::vector<double> weights;

	// GPS standard deviation for initialization
	double std_GPS_x;
	double std_GPS_y;
	double std_GPS_theta;*/

public:
	/*
	// Set of current particles
	std::vector<Particle> particles;*/

	// Constructor
	PathPlanner() {}

	// Destructor
	~PathPlanner() {}

	/**
	 * behavior_planning Uses sensor fusion data to make high level decisions on the road.
	 *   For instance, avoiding collisions with front car, changing lanes when necessary
	 *   or adjusting velocity to front car one if changing lanes is not safe.
	 * @param sensor_fusion Sensor fusion data
	 * @param car_s Actual car s Frenet coordinate
	 * @param car_d Actual car d Frenet coordinate
	 * @param prev_size Previous size of path data
	 * @param lane Actual lane
	 * @param ref_vel Car velocity in mph
	 */
	void behavior_planning(std::vector<std::vector<double>> sensor_fusion, double car_s, double car_d, int prev_size, int *lane, double *ref_vel);
};



#endif /* PATH_PLANNER_H_ */
