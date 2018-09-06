/*
 * star_trajectory.cpp
 *
 *  Created on: Sep 6, 2018
 *      Author: yilun
 */
#include "ts_path_planner.h"
#include <matrix/math.hpp>

void TailsitterPathPlanner::star_generator_main()
{
	math::Matrix<11, 4> Verticies;
	float rho = 0.525731f;
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
		Verticies(i, 3) = 180.f;
	}


	for (int i = 0; i < 11; i ++){
		publish_waypoint(Verticies(i, 0),
				Verticies(i, 1),
				Verticies(i, 2),
				Verticies(i, 3) / 180.f * 3.14f);
		while(_setpoint_updated) usleep(1e5); //waiting for reference signal to reach destination
		usleep(2e6); //wait for vehicle to stablize before sending next waypoint
	}

}
