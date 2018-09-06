/*
 * star_trajectory.cpp
 *
 *  Created on: Sep 6, 2018
 *      Author: yilun
 */
#include "ts_path_planner.h"
#include <matrix/math.hpp>

void TailsitterPathPlanner::star_generator_main(char*argv[])
{
	math::Matrix<11, 4> Verticies;
	float rho = strtof(argv[1], 0);
	printf("rho is: %.2f\n", (double)rho);
	float R = rho/0.525731f*0.200811f;
	float delta = 2*3.14159f/10.f;
	float Z = -1.4f;

	for (int i = 0; i < 11; i ++) {
		if (i % 2 == 0) {
			Verticies(i, 0) = rho*(float)sin(i*delta);
			Verticies(i, 1) = rho*(float)cos(i*delta);
		}
		else{
			Verticies(i, 0) = R*(float)sin(i*delta);
			Verticies(i, 1) = R*(float)cos(i*delta);
		}
		Verticies(i, 2) = Z;
		if (i > 0){
			//float dy = Verticies(i, 2) - Verticies(i - 1, 2);
			float theta = (float)atan2((Verticies(i, 1) - Verticies(i - 1, 1)), (Verticies(i, 0) - Verticies(i - 1, 0)));
			float hdg_diff_1 = abs(Verticies(i - 1, 3) - (theta - 3.141592f/2.f));
			float hdg_diff_2 = abs(Verticies(i - 1, 3) - (theta + 3.141592f/2.f));
			Verticies(i, 3) = hdg_diff_1 < hdg_diff_2?(theta - 3.141592f/2.f):(theta + 3.141592f/2.f);
		}
	}


	for (int j = 0; j < 11; j ++){
		publish_waypoint(Verticies(j, 0),
				Verticies(j, 1),
				Verticies(j, 2),
				Verticies(j, 3));
		while(_setpoint_updated) usleep(1e5); //waiting for reference signal to reach destination
		usleep(5e6); //wait for vehicle to stablize before sending next waypoint
	}
}
