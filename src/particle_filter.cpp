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
        
    // Create normal distributions for x, y and theta.
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);
    
    //init number of particles to use
    num_particles = 500;
    // Resize the `particles` vector to fit desired number of particles
    particles.resize(num_particles);
    weights.resize(num_particles);
    double init_weight = 1.0 / num_particles;
    
    static default_random_engine gen;
    gen.seed(123);
    
    for (int i = 0; i < num_particles; i++){
        particles[i].id = i;
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
        particles[i].weight = init_weight;
    }	
    
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/
    
    //some constants to save computation power
    const double vel_del = velocity * delta_t;
    const double yaw_del = yaw_rate * delta_t;
    const double vel_yaw = velocity / yaw_rate;
    
    normal_distribution<double> dist_x(0.0, std_pos[0]);
    normal_distribution<double> dist_y(0.0, std_pos[1]);
    normal_distribution<double> dist_theta(0.0, std_pos[2]);
    
    static default_random_engine gen;
    gen.seed(321);
    
    for (int i = 0; i < num_particles; i++){
        if (fabs(yaw_rate) < 0.001){
            particles[i].x += vel_del * cos(particles[i].theta);
            particles[i].y += vel_del * sin(particles[i].theta);
            //particles[i].theta //unchanged if yaw_rate is too small
        } else{
            const double theta_new = particles[i].theta + yaw_del;
            particles[i].x += vel_yaw * (sin(theta_new) - sin(particles[i].theta));
            particles[i].y += vel_yaw * (-cos(theta_new) + cos(particles[i].theta));
            particles[i].theta = theta_new;
        }
        
        // Add random Gaussian noise
        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
    //   implement this method and use it as a helper during the updateWeights phase.
    
    double min_distance, distance;
    int min_i;
    
    for (int i = 0; i < observations.size(); i++) {
        auto obs = observations[i];
        min_distance = INFINITY;
        min_i = -1;
        
        for (int j = 0; j < predicted.size(); j++) {
            distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
            
            if (distance < min_distance) {
                min_distance = distance;
                min_i = j;
            }
        }
        // Use index of landmark as the ID (rather than the id field)
        observations[i].id = min_i;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
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
    
    // Landmark measurement uncertainty [x [m], y [m]]
    //double std_landmark [2] = {0.3, 0.3};
    const double cov_x = std_landmark[0] * std_landmark[0];
    const double cov_y = std_landmark[1] * std_landmark[1];
    const double normalizer = 2 * M_PI * std_landmark[0] * std_landmark[1];
    
    for (int i = 0; i < particles.size(); i++) {
    //for(unsigned p_ctr=0; p_ctr < particles.size(); p_ctr++) {
        std::vector<LandmarkObs> predicted;
        
        for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
        //for (auto lm : map_landmarks.landmark_list) {
            LandmarkObs pred;
            
            pred.x = map_landmarks.landmark_list[j].x_f;
            pred.y = map_landmarks.landmark_list[j].y_f;
            pred.id = map_landmarks.landmark_list[j].id_i;
            auto dx_p = pred.x - particles[i].x;
            auto dy_p = pred.y - particles[i].y;
            
            // Add only if in range
            if (dx_p * dx_p + dy_p * dy_p <= sensor_range * sensor_range) {
                predicted.push_back(pred);
            }
        }
        
        std::vector<LandmarkObs> transformed_obs;
        double total_prob = 1.0f;
        
        // transform coordinates of all observations (for current particle)
        for (int k = 0; k < observations.size(); k++) {
        //for(auto obs_lm : observations) {
            LandmarkObs obs;
            
            // First rotate the local coordinates to the right orientation
            obs.x = particles[i].x + observations[k].x * cos(particles[i].theta) - observations[k].y * sin(particles[i].theta);
            obs.y = particles[i].y + observations[k].x * sin(particles[i].theta) + observations[k].y * cos(particles[i].theta);
            obs.id = observations[k].id;
            transformed_obs.push_back(std::move(obs));
        }
        
        // Stores index of associated landmark in the observation
        dataAssociation(predicted, transformed_obs);
        
        for(int l = 0; l < transformed_obs.size(); l++) {
            // Assume sorted by id and starting at 1
            auto dx = (transformed_obs[l].x - predicted[transformed_obs[l].id].x);
            auto dy = (transformed_obs[l].y - predicted[transformed_obs[l].id].y);
            total_prob *= exp(-0.5 * ((dx * dx / cov_x) + (dy * dy / cov_y))) / normalizer;
            //total_prob *= exp(-(dx * dx / (2 * cov_x) + dy * dy / (2 * cov_y))) / normalizer;
        }
        
        particles[i].weight = total_prob;
        weights[i] = total_prob;
    }
}

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight. 
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    
    discrete_distribution<> dist_particles(weights.begin(), weights.end());
    
    vector<Particle> new_particles;
    new_particles.resize(num_particles);
    
    static default_random_engine gen;
    gen.seed(123);
    
    for (int i = 0; i < num_particles; i++) {
        new_particles[i] = particles[dist_particles(gen)];
    }
    
    particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y) {
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    
    //Clear the previous associations
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();
    
    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
    
    return particle;
}

string ParticleFilter::getAssociations(Particle best) {
    vector<int> v = best.associations;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseX(Particle best) {
    vector<double> v = best.sense_x;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseY(Particle best) {
    vector<double> v = best.sense_y;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}
