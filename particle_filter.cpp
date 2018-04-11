/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	default_random_engine random;
	num_particles = 1000;

	double std_x, std_y,  std_theta;

	std_x = std[0];
	std_y = std[1];
	std_theta = std[2];

	//cout<<"weights:"len(std[])<<endl;
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	//weights = [];
	for (int i=1; i<=num_particles; i++)
	{
		Particle particle;
		particle.id = i;
		particle.x = dist_x(random);
		particle.y = dist_y(random);
		particle.theta = dist_theta(random);


		particles.push_back(particle);
			//particles[i] = dist_y(i);
			//particles[i] = dist_theta(i);

		weights[i] = 1.0;
		//cout<<"particles:"particles[i]<<endl;
		//cout<<"weights:" << weights[i] << endl;
	}
	is_initialized = true;






}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine random;

	if(is_initialized)
	{

		for (int i=1; i<=num_particles; i++)
		{
			double x, y, theta;

			x = particles[i].x;
			y = particles[i].y;
			theta = particles[i].theta;

			if(yaw_rate == 0 )
			{
				x = x + velocity * cos(theta) * (delta_t);
				y = y + velocity * sin(theta) * (delta_t);
				theta = theta;
			}
			else
			{
				x = x + velocity * (sin(theta + yaw_rate * (delta_t)) - sin(theta)) / yaw_rate;
				y = y + velocity * (cos(theta) - cos(theta + yaw_rate * (delta_t))) / yaw_rate;
				theta = theta + yaw_rate * (delta_t);
			}
			normal_distribution<double> N_x(x, std_pos[0]);
			normal_distribution<double> N_y(y, std_pos[1]);
			normal_distribution<double> N_theta(theta, std_pos[2]);


			particles[i].x = N_x(random);
			particles[i].y = N_y(random);
			particles[i].theta = N_theta(random);
		}

	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.
	for(int i =0 ; i<= observations.size(); i++)
	{
		LandmarkObs obs = observations[i];

		double mindist = numeric_limits<double>::max();

		int id = -1;

		for(int j =0; j<= predicted.size(); j++)
		{
			LandmarkObs pre = predicted[j];

			double mdist= dist(obs.x,obs.y,pre.x,pre.y);

			if(mdist < mindist)
			{

				mindist = pre.id;
				id = pre.id;
			}
		}

		observations[i].id = id;

	}


}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	for (int i= 0; i <= num_particles; i++)
	{
		double p_x,p_y,p_theta;

		p_x = particles[i].x;
		p_y = particles[i].y;
		p_y = particles[i].theta;

		//landmark predictions
		vector<LandmarkObs> landmark_prediction;
		for (int j=0; j < map_landmarks.landmark_list.size(); j++)
		{
			float land_x = map_landmarks.landmark_list[j].x_f;
			float land_y = map_landmarks.landmark_list[j].y_f;
			float land_id = map_landmarks.landmark_list[j].id_i;

			double distance= dist(p_x,p_y,land_x,land_y);

			if(distance <= sensor_range)
			{
				landmark_prediction.push_back(LandmarkObs{land_id,land_x,land_y});
			}

		}


		//landmark observations
		vector<LandmarkObs> landmark_observaton
		for (int j=0; j < observations.size(); j++)
		{
			LandmarkObs objObservation;

			objObservation.id = j;
			objObservation.x  = particles[i].x + (observations[i].x * cos(particles[i].theta) - observations[i].y  * sin(particles[i].theta));
			objObservation.y  = particles[i].y + (observations[i].x * sin(particles[i].theta) - observations[i].y * cos(particles[i].theta));

			landmark_observaton.push_back(objObservation);

		}

		dataAssociation(landmark_prediction,landmark_observaton);

		//Update weights
		particles[i].weight =1.0;
		double sig_x = std_landmark[0];
		double sig_y = std_landmark[1];
		double gauss_norm = (1/(2 * M_PI * sig_x * sig_y));

		for (int j=0; j< landmark_observaton.size(); j++)
		{
			double l_ox,l_oy;
			l_ox = landmark_observaton[j].x;
			l_oy = landmark_observaton[j].y;

			for (int k=0; k<landmark_prediction.size(), k++)
			{
				double l_px,l_py;
				l_px = landmark_prediction[k].x;
				l_py = landmark_prediction[k].y;
				if(landmark_observaton[k].id == landmark_prediction[k].id)
				{

					double exponent = exp(-1.0 * (pow(l_ox - l_px,2))/(2 * pow(sig_x,2)) + (pow(l_oy - l_py,2)/(2 * pow(sig_y,2))));
					particles[i].weight *=gauss_norm * exponent;
				}
			}

		}
		weights[i] = particles[i].weight;

	}


}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	vector<Particle> new_particles;

	random_device rd;
    default_random_engine gen(rd());
	for (int i = 0; i < num_particles; ++i) 
	{
	    discrete_distribution<int> index(weights.begin(), weights.end());
	    new_particles[i] = particles[index(gen)];
	    
	}
	particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
