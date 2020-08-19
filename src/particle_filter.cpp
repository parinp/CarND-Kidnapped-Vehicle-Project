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

#include "helper_functions.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles

  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

  normal_distribution<double> dist_x(x,std_x);
  normal_distribution<double> dist_y(y,std_y);
  normal_distribution<double> dist_theta(theta,std_theta);

  default_random_engine gen;

  
  for(int i = 0;i<num_particles;++i){
    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;

    particles.push_back(p);
  }
  
  is_initialized = true;

  cout<<"Init pass"<<endl;

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

  
  normal_distribution<double> dist_x(0,std_pos[0]);
  normal_distribution<double> dist_y(0,std_pos[1]);
  normal_distribution<double> dist_theta(0,std_pos[2]);
  
  default_random_engine gen;
  double theta;

  for(int i = 0 ; i < num_particles ;++i){

    theta = particles[i].theta;

    if(fabs(yaw_rate)>0.001){
      
      particles[i].x += (velocity/yaw_rate)*(sin(theta+yaw_rate*delta_t)-sin(theta));
      particles[i].y += (velocity/yaw_rate)*(cos(theta)-cos(theta+yaw_rate*delta_t));
      particles[i].theta += yaw_rate*delta_t; 

    }else{
      particles[i].x += velocity*delta_t*cos(theta);
      particles[i].y += velocity*delta_t*sin(theta);
      
    }
    
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }
  
   cout << "Prediction pass" << endl;
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  int numObs = observations.size();
  int numPre = predicted.size();

  for(int i = 0; i<numObs;i++){

    double min_dist = numeric_limits<double>::max();
    int map_id = -1;

    for(int j = 0 ; j<numPre;j++){
      
      double distance = dist(observations[i].x,observations[i].y,predicted[j].x,predicted[j].y);

      if(distance<min_dist){
        min_dist = distance;
        map_id = predicted[j].id;
      }

    }


    observations[i].id = map_id;
  }

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
  
   double xm, ym, xc, yc;
   vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;

   for(int i = 0;i<num_particles;i++){

     double xp = particles[i].x;
     double yp = particles[i].y;
     double theta_p = particles[i].theta;
     
     vector<LandmarkObs> pred_landmarks;

     for(unsigned int j = 0;j<landmarks.size();j++){
       
       double distance = dist(xp,yp,landmarks[j].x_f,landmarks[j].y_f);

       if(distance < sensor_range){
         
         pred_landmarks.push_back(LandmarkObs{landmarks[j].id_i,landmarks[j].x_f,landmarks[j].y_f}); 

       }   
     }

     vector<LandmarkObs> transformed_Obs;

     for(unsigned int k = 0;k<observations.size();k++){
       xc = observations[k].x;
       yc = observations[k].y;

       xm = xc*cos(theta_p) - yc*sin(theta_p) + xp;
       ym = xc*sin(theta_p) + yc*cos(theta_p) + yp;
       transformed_Obs.push_back(LandmarkObs{observations[k].id,xm,ym});

     }

     dataAssociation(pred_landmarks,transformed_Obs);

     particles[i].weight = 1.0;

     for(unsigned int l = 0;l<transformed_Obs.size();l++){
       
       int  Obs_id = transformed_Obs[l].id;
       double Obs_x = transformed_Obs[l].x; 
       double Obs_y = transformed_Obs[l].y;
       
       double pred_x;
       double pred_y;

       for(unsigned int m = 0;m<pred_landmarks.size();m++){

         if(pred_landmarks[m].id==Obs_id){
           pred_x = pred_landmarks[m].x;
           pred_y = pred_landmarks[m].y;
         } 
       }

       double std_lx = std_landmark[0]; 
       double std_ly = std_landmark[1];
       
       double norm =   1/(2*M_PI*std_lx*std_ly);
       double x_part = pow((pred_x-Obs_x),2)/(2*pow(std_lx,2));
       double y_part = pow((pred_y-Obs_y),2)/(2*pow(std_ly,2));

       double new_weight = norm*exp(-(x_part+y_part));
       
       particles[i].weight *= new_weight;

     }
   }

   cout << "Update Pass" << endl;
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

   vector<double> weights;
   for (int i = 0; i < num_particles; i++) {
     weights.push_back(particles[i].weight);
   }

   default_random_engine gen;

   uniform_int_distribution<int> dist_ind(0,num_particles-1);

   double max_w = *max_element(weights.begin(),weights.end());
   uniform_real_distribution<double> dist_w(0,max_w);

   int index = dist_ind(gen);
   double beta = 0.0;

   vector<Particle> resampled_p;
   cout<<max_w<<endl;
   for(int i=0;i<num_particles;++i){

     beta += 2.0*dist_w(gen);
     while(beta > weights[index]){
       beta -= weights[index];
       index = ( index+1 ) % num_particles;
     }
     
     resampled_p.push_back(particles[index]);
   }

   particles = resampled_p;
   cout << "Resample pass" << endl;
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
  copy(v.begin(), v.end(), std::ostream_iterator<double>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}