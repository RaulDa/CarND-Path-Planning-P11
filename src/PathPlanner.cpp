/*
 * PathPlanner.cpp
 *
 *  Created on: May 26, 2018
 */

#include "PathPlanner.h"

#include <vector>
#include <math.h>
#include <iostream>

using namespace std;

void PathPlanner::behavior_planning(vector<vector<double>> sensor_fusion, double car_s, double car_d, int prev_size, int *lane, double *ref_vel) {

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
