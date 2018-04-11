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

	if(!is_initialized)
	{
		default_random_engine random;
		num_particles = 50;

		double std_x, std_y,  std_theta;

		std_x = std[0];
		std_y = std[1];
		std_theta = std[2];

		weights.resize(num_particles);
		//cout<<"print1"<<endl;
		//cout<<"weights:"len(std[])<<endl;
		normal_distribution<double> dist_x(x, std_x);
		normal_distribution<double> dist_y(y, std_y);
		normal_distribution<double> dist_theta(theta, std_theta);

		//weights = [];
		for (int i=0; i<num_particles; i++)
		{
			//cout<<"print2"<<endl;
			Particle particle;
			particle.id = i;
			particle.x = dist_x(random);
			particle.y = dist_y(random);
			particle.theta = dist_theta(random);
			particle.weight = 1.0;
			//cout<<particle.x<<endl;
			//cout<<particle.y<<endl;
			particles.push_back(particle);
		}
		is_initialized = true;
	}


}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine random;

	for (int i=0; i < num_particles; i++)
	{
		double x, y, theta;

		x = particles[i].x;
		y = particles[i].y;
		theta = particles[i].theta;

		if(fabs(yaw_rate) < 0.01)
		{
			particles[i].x += velocity * cos(theta) * (delta_t);
			particles[i].y += velocity * sin(theta) * (delta_t);
		}
		else
		{
			particles[i].x += (velocity / yaw_rate) * (sin(theta + (yaw_rate * delta_t)) - sin(theta));
			particles[i].y += (velocity / yaw_rate) * (cos(theta) - cos(theta + (yaw_rate * delta_t))) ;
			particles[i].theta += (yaw_rate * delta_t);
		}
		normal_distribution<double> N_x(0, std_pos[0]);
		normal_distribution<double> N_y(0, std_pos[1]);
		normal_distribution<double> N_theta(0, std_pos[2]);


		particles[i].x += N_x(random);
		particles[i].y += N_y(random);
		particles[i].theta += N_theta(random);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

		int id;
		for(int i =0 ; i< observations.size(); i++)
		{
			LandmarkObs obs = observations[i];

			double mindist = numeric_limits<double>::max();
			for(int j =0; j < predicted.size(); j++)
			{
				LandmarkObs pre = predicted[j];
				double mdist= dist(obs.x,obs.y,pre.x,pre.y);
				if(mdist < mindist)
				{
					mindist = mdist;
					id = j;
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
		//return;
		for (int i= 0; i < num_particles; i++)
		{
			//landmark observations
			vector<LandmarkObs> landmark_observaton;
			for (int j=0; j < observations.size(); j++)
			{
				LandmarkObs objObservation;

				objObservation.id = observations[j].id;
				objObservation.x  = particles[i].x + (observations[j].x * cos(particles[i].theta)) - (observations[j].y  * sin(particles[i].theta));
				objObservation.y  = particles[i].y + (observations[j].x * sin(particles[i].theta)) + (observations[j].y * cos(particles[i].theta));

				landmark_observaton.push_back(objObservation);

			}


			//landmark predictions
			vector<LandmarkObs> landmark_prediction;
			for (int j=0; j < map_landmarks.landmark_list.size(); j++)
			{
				double distance= dist(map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f, particles[i].x,  particles[i].y);

				if(distance < sensor_range)
				{
					LandmarkObs objPrediction;
					objPrediction.x = map_landmarks.landmark_list[j].x_f;
					objPrediction.y = map_landmarks.landmark_list[j].y_f;
					objPrediction.id = map_landmarks.landmark_list[j].id_i;

					landmark_prediction.push_back(objPrediction);
				}
			}
			dataAssociation(landmark_prediction,landmark_observaton);

			std::vector<int> associations;
			std::vector<double> sense_x;
			std::vector<double> sense_y;
			for (int l=0; l<observations.size(); l++)
			{
				associations.push_back(landmark_prediction[landmark_observaton[l].id].id);
				sense_x.push_back(landmark_observaton[l].x);
				sense_y.push_back(landmark_observaton[l].y);
			}

			SetAssociations(particles[i],associations,sense_x,sense_y);

			//Update weights
			double prob = 1.0;
			double sig_x = std_landmark[0];
			double sig_y = std_landmark[1];
			double gauss_norm = (1/(2 * M_PI * sig_x * sig_y));
			for (int j=0; j< landmark_observaton.size(); j++)
			{
				double l_ox, l_oy, l_px, l_py;;
				l_ox = landmark_observaton[j].x;
				l_oy = landmark_observaton[j].y;
				l_px = landmark_prediction[landmark_observaton[j].id].x;
				l_py = landmark_prediction[landmark_observaton[j].id].y;

				double exponent = exp(-1.0 * (((pow(l_ox - l_px,2))/(2 * pow(sig_x,2))) + ((pow(l_oy - l_py,2)/(2 * pow(sig_y,2))))));
				prob *= gauss_norm * exponent;


			}
			particles[i].weight = prob;
			weights[i] = prob;

		}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

		vector<Particle> resampled;

		//Random device generator using std::discrete_distribution
		std::random_device rd;
		std::mt19937 gen(rd());

		discrete_distribution<int> random_number(0 , num_particles-1);
		discrete_distribution<int> disct_dis(weights.begin(), weights.end());
		unsigned index =random_number(rd);

		double max_weight = *max_element(weights.begin(), weights.end());

		//resampling
		double beta =0.0;
		for (int i =0; i <num_particles; i++)
		{
			beta+=disct_dis(gen) * 2.0 * max_weight;
			while(beta > weights[index])
			{
				beta -= weights[index];
				index = (index + 1) % num_particles;
			}
			resampled.push_back(particles[index]);
		}
		particles = resampled;

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

		return particle;
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
