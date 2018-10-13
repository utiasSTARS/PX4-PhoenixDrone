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
	math::Matrix<6, 3> Verticies;
	float rho = _params.star_rho;
	printf("rho is: %.2f\n", (double)rho);
	//float R = rho/0.525731f*0.200811f;
	float delta = 2*3.14159f/5.f;
	float Z = -1.0f;

	for (int i = 0; i < 6; i ++) {
		Verticies(i, 0) = rho*(float)sin(i*delta);
		Verticies(i, 1) = rho*(float)cos(i*delta);
		Verticies(i, 2) = Z;
		//Verticies(i, 3) = ((90.f-18.f) - i * 36.f)/180.f*3.14159f;

	}


	int sequence [6] = {0, 2, 4, 1, 3, 5};

	for (int i = 0; i < 6; i++){
		publish_waypoint(Verticies(sequence[i], 0),
				Verticies(sequence[i], 1),
				Verticies(sequence[i], 2),
				((180.f-18.f) + (i) * 36.f)/180.f*3.14159f);
		while(_setpoint_updated) usleep(1e5); //waiting for reference signal to reach destination
		usleep(5e6); //wait for vehicle to stablize before sending next waypoint
		if (i!= 5)		publish_waypoint(Verticies(sequence[i], 0),
					Verticies(sequence[i], 1),
					Verticies(sequence[i], 2),
					((180.f-18.f) + (i+1) * 36.f)/180.f*3.14159f);
		while(_setpoint_updated) usleep(1e5);
		usleep(4e6);
	}

}

