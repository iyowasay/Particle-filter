/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <limits>
#include <cassert>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;

static std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {

  // std::default_random_engine gen;
  num_particles = 100;  // TODO: Set the number of particles

  // particles.resize(num_particles);
  // weights.resize(num_particles, 1.0);

  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  for(int i=0;i < num_particles;++i)
  {
    Particle P;
    P.id = i;
    P.x = dist_x(gen);
    P.y = dist_y(gen);
    P.theta = dist_theta(gen);
    P.weight = 1.0;
    // particles[i] = P;
    particles.push_back(P);
    weights.push_back(1.0);
  }

  assert(particles.size()==num_particles);
  assert(weights.size()==num_particles);

  is_initialized = true;
  std::cout << "Initialized!!!" << std::endl;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

  for(int k = 0;k < num_particles; ++k)
  {
    if(fabs(yaw_rate) < 0.00001)
    {
      // if the yaw rate is very close to 0, we assume the car is going straight forward.
      particles[k].x += velocity*delta_t*cos(particles[k].theta); 
      particles[k].y += velocity*delta_t*sin(particles[k].theta);
    }
    else
    {
      particles[k].x += velocity/yaw_rate*(sin(particles[k].theta+delta_t*yaw_rate) - sin(particles[k].theta));
      particles[k].y += velocity/yaw_rate*(cos(particles[k].theta) - cos(particles[k].theta+delta_t*yaw_rate));
      particles[k].theta += delta_t*yaw_rate;
    }
    

    // add noise to the prediction, these noise are from the sensor measurement
    // std::default_random_engine gen;
    normal_distribution<double> dist_x(particles[k].x, std_pos[0]);
    normal_distribution<double> dist_y(particles[k].y, std_pos[1]);
    normal_distribution<double> dist_theta(particles[k].theta, std_pos[2]);

    particles[k].x = dist_x(gen);
    particles[k].y = dist_y(gen);
    particles[k].theta = dist_theta(gen);
  }


}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {

  // double dis_theshold = 1.0; 
  // the distance threshold, if within this range then should be checked and compared.
  for(int i=0;i < observations.size();++i)
  {
    LandmarkObs observed = observations[i];
    int good_id = -1;
    double cur_distance=0.0;
    double min_distance = std::numeric_limits<double>::max();
    for(unsigned int j=0;j < predicted.size();++j)
    {
      cur_distance = dist(observed.x, observed.y, predicted[j].x, predicted[j].y);
      if(cur_distance < min_distance)
      {
        good_id = predicted[j].id;
        min_distance = cur_distance;
      }
    }

    observations[i].id = good_id;
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  weights.clear();

  for(int i = 0;i < num_particles;++i)
  {
    double cur_x = particles[i].x;
    double cur_y = particles[i].y;
    double cur_theta = particles[i].theta;

    // create the prediction and observation measurement in world coordinate from the map/landmark information 
    std::vector<LandmarkObs> predicted_vec;
    for(unsigned int m = 0;m < map_landmarks.landmark_list.size();++m)
    {
      double landmark_x = map_landmarks.landmark_list[m].x_f;
      double landmark_y = map_landmarks.landmark_list[m].y_f;
      double diff_ = dist(cur_x, cur_y, landmark_x, landmark_y);
      // double dy_ = particles[i].y - map_landmarks.landmark_list[m].y_f;
      if(diff_ <= sensor_range)
      {
        int select_id = map_landmarks.landmark_list[m].id_i;
        predicted_vec.push_back(LandmarkObs{select_id, landmark_x, landmark_y});
      }

    }

    std::vector<LandmarkObs> observed_vec;
    for(unsigned int n = 0;n < observations.size();++n)
    {
      LandmarkObs ob = observations[n];
      // Homogeneous matrix transformation of observations (from car coordinate to world/map coordinate)
      double x_map = cur_x + ob.x*cos(cur_theta) - ob.y*sin(cur_theta);
      double y_map = cur_y + ob.x*sin(cur_theta) + ob.y*cos(cur_theta);  

      observed_vec.push_back(LandmarkObs{ob.id, x_map, y_map});
    }

    // association 
    dataAssociation(predicted_vec, observed_vec);
    particles[i].weight = 1.0;
 
    // each observation
    for(auto obs : observed_vec)
    {
      // find out the coordinate of associated landmark
      double lm_x, lm_y;
      for(auto pred: predicted_vec)
      {
        if(obs.id==pred.id)
        {
          lm_x = pred.x; // the x coordinate of this landmark on the world map 
          lm_y = pred.y;
          break;
        }
      }
      // calculate new weight
      double temp = (obs.x - lm_x)*(obs.x - lm_x) / (2.0 * std_landmark[0]*std_landmark[0]) + (obs.y - lm_y)*(obs.y - lm_y) / (2.0 * std_landmark[1]*std_landmark[1]);
      particles[i].weight *= 1 / (2.0 * M_PI * std_landmark[0] * std_landmark[1]) * exp(-temp);

    }

    weights.push_back(particles[i].weight);
  }

  assert(weights.size()==num_particles);
  // normalize the updated weights
  double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
  for(int w = 0;w < weights.size();++w)
  {
    weights[w] /= sum;
    particles[w].weight /= sum;
  }

}

void ParticleFilter::resample() {

  // resampling wheel method
  std::vector<Particle> temp_particles(num_particles);
  std::vector<double> temp_weights(num_particles);
  double BETA = 0.0;
  // int index = std::discrete_distribution();
  int index = std::rand() % num_particles; 
  double max_weight = *(std::max_element(weights.begin(),weights.end()));
  for(int j =0;j < num_particles;++j)
  {
    BETA += (static_cast<float> (std::rand())/static_cast<float> (RAND_MAX))*2*max_weight;
    while(weights[index] < BETA)
    {
      BETA -= weights[index];
      index = (index+1) % num_particles;
    }
    temp_particles[j] = particles[index];
    // temp_weights[j] = weights[index];
  }
  // update the weights vector
  particles = temp_particles;
  // weights = temp_weights;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}