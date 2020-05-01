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

#ifdef __MACH__
  #include <mach/mach_time.h>
#endif

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
 
  num_particles = 20;  // TODO: Set the number of particles
  
  std::normal_distribution<double> dist_x (x, std[0]), dist_y(y, std[1]), dist_theta(theta, std[2]);
  
  for (int i = 0; i < num_particles; ++i)
  {
    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1;
    particles.push_back(p);
  }
  

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  

  std::normal_distribution<double> dist_x (0, std_pos[0]), dist_y(0, std_pos[1]), dist_theta(0, std_pos[2]);

  for (auto it = particles.begin(); it != particles.end(); ++it)
  {
    if (yaw_rate != 0)
    {
      it->x += velocity/yaw_rate*(sin(it->theta + yaw_rate*delta_t) - sin(it->theta)) + dist_x(gen);
      it->y += velocity/yaw_rate*(cos(it->theta) - cos(it->theta + yaw_rate*delta_t)) + dist_y(gen);
      it->theta += yaw_rate*delta_t+dist_theta(gen);
    }
    else
    {
      it->x += velocity*delta_t*cos(it->theta) + dist_x(gen);
      it->y += velocity*delta_t*sin(it->theta) + dist_y(gen);
      it->theta += dist_theta(gen);
    }
  }
  
  
}

std::vector<std::pair<LandmarkObs, LandmarkObs>> ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations, double max_dist) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  std::vector<std::pair<LandmarkObs, LandmarkObs>> pairings;
  for (auto const &obs: observations)
  {
    auto best_pred = predicted.end();
    double smallest_distance = std::numeric_limits<double>::max();
    for (auto it = predicted.begin(); it != predicted.end(); ++it)
    {
      double dst = dist(obs.x, obs.y, it->x, it->y);
      if (dst < smallest_distance)
      {
        best_pred = it;
        smallest_distance = dst;
      }
    }
    if (best_pred != predicted.end())
    {
      pairings.push_back(std::make_pair(obs, *best_pred));
      predicted.erase(best_pred);
    }
    else
    {
      // make a fake prediction just outside the sensor range (is it far enough away?!)
      pairings.push_back(std::make_pair(obs, LandmarkObs(obs.id, obs.x+max_dist, obs.y)));
    }
  }
  return pairings;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  
  
  double gd_const_term =1/sqrt(2*M_PI*std_landmark[0]*std_landmark[1]);
  double gd_xmul =-(1/(2*std_landmark[0]));
  double gd_ymul =-(1/(2*std_landmark[1]));

  
  for (auto it = particles.begin(); it != particles.end(); ++it)
  {

    
    vector<LandmarkObs> obs_on_map;
    for (auto const &obs : observations)
    {
      LandmarkObs translated_obs;
      translated_obs.id = obs.id;
      translated_obs.x = obs.x*cos(it->theta) - obs.y*sin(it->theta) + it->x;
      translated_obs.y = obs.x*sin(it->theta) + obs.y*cos(it->theta) + it->y;
      obs_on_map.push_back(translated_obs);
    }
    
    
    std::vector<LandmarkObs> predictions;
    double srx1 = it->x - sensor_range;
    double srx2 = it->x + sensor_range;
    double sry1 = it->y - sensor_range;
    double sry2 = it->y + sensor_range;
    
    for (auto const &landmark : map_landmarks.landmark_list)
    {
      if (landmark.x_f >= srx1 && landmark.x_f <= srx2 && landmark.y_f >= sry1 && landmark.y_f <= sry2)
      {
        if (dist(landmark.x_f, landmark.y_f, it->x, it->y) < sensor_range)
        {
          predictions.push_back(LandmarkObs(landmark.id_i, landmark.x_f, landmark.y_f));
        }
      }
    }
    
    
    double weight = 1;
    std::vector<std::pair<LandmarkObs, LandmarkObs>> pairings = dataAssociation(predictions, obs_on_map, sensor_range);


    
    for (auto const &pairing : pairings)
    {
      /*double gaussdens = 1/sqrt(2*M_PI*std_landmark[0]) *
      exp(
          -(1/(2*std_landmark[0]))*pow(pairing.first.x - pairing.second.x, 2)
          -(1/(2*std_landmark[1]))*pow(pairing.first.y - pairing.second.y, 2)
          );*/
      double gaussdens = gd_const_term * exp(
          gd_xmul*(pairing.first.x - pairing.second.x)*(pairing.first.x - pairing.second.x) +
          gd_ymul*(pairing.first.y - pairing.second.y)*(pairing.first.y - pairing.second.y)
      );
     
      weight *= gaussdens;
    }
    it->weight = weight;
    

  }

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  
  int num_particles = particles.size();
  
  //it would be cleaner to keep the weights separate from the particles, but I don't want to
  // change the grading code or duplicate(cache) it
  std::vector<double> weights (num_particles);
  std::transform(particles.begin(), particles.end(), weights.begin(), [](Particle p) {
    return p.weight;
  });
  std::discrete_distribution<int> disc_dist(weights.begin(), weights.end());
  
  std::vector<Particle> new_particles;
  new_particles.reserve(num_particles);
  for (int i = 0; i < num_particles; ++i)
  {
    new_particles.push_back(particles[disc_dist(gen)]);
  }

  
  particles = new_particles;


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
