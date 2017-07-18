/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */



#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// Initialize number of particles
	num_particles = 100;

	// Setup distributions
	std::default_random_engine generator;
	std::normal_distribution<double> distribution_x(x, std[0]);
	std::normal_distribution<double> distribution_y(y, std[1]);
	std::normal_distribution<double> distribution_theta(theta, std[2]);

	// Initialize each particle with first measurement and add random noise
	for (int i=0; i<num_particles; i++) {
		// Initialize new particle
		Particle new_particle;

		new_particle.id = i;
		new_particle.x = distribution_x(generator);
		new_particle.y = distribution_y(generator);
		new_particle.theta = distribution_theta(generator);

		particles.push_back(new_particle);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// Setup distributions
	std::default_random_engine generator;
	std::normal_distribution<double> distribution_x(0, std_pos[0]);
	std::normal_distribution<double> distribution_y(0, std_pos[1]);
	std::normal_distribution<double> distribution_theta(0, std_pos[2]);

	// Loop over particles
	for (int id = 0; id < num_particles; id++) {
		// Extract old particle coordinates
		Particle particle = particles[id];
		double x = particle.x;
		double y = particle.y;
		double theta = particle.theta;

		// Prediction step
    double x_new;
    double y_new;
    double theta_new;
    if (fabs(yaw_rate) >= 0.0001) {
      x_new = x + velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
      y_new = y + velocity / yaw_rate * (-cos(theta + yaw_rate * delta_t) + cos(theta));
      theta_new = theta + yaw_rate * delta_t;
    } else {
      x_new = x + velocity*delta_t*sin(theta);
      y_new = y + velocity*delta_t*cos(theta);
    }


		// Add random noise to estimates
		x_new += distribution_x(generator);
		y_new += distribution_y(generator);
		theta_new += distribution_theta(generator);

		// Write predictions back into particle
		particles[id].x = x_new;
		particles[id].y = y_new;
		particles[id].theta = theta_new;
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> observations, Map map_landmarks) {
  //   observed measurement to this particular landmark.
  // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
  //   implement this method and use it as a helper during the updateWeights phase.
  int n_obs = observations.size();

  // Loop over particles
  for (int i = 0; i < num_particles; i++) {
    // Associate current particle in loop with variable
    Particle current_particle = particles[i];

    // Initialize associations for current particle
    std::vector<int> associations(n_obs);
    //std::vector<double> best_dist(n_obs);
    current_particle.dist.clear();

    // Loop over observations to find closest landmark
    for (int j = 0; j < n_obs; j++) {
      // Transform observations to map coordinate system

      // Read coordinates of current observations
      double obs_x = observations[j].x;
      double obs_y = observations[j].y;

      // Rotated observation
      double obs_x_global = cos(-current_particle.theta) * obs_x + sin(-current_particle.theta) * obs_y;
      double obs_y_global = -sin(-current_particle.theta) * obs_x + cos(current_particle.theta) * obs_y;

      // Rotated and translated observation
      obs_x_global += current_particle.x;
      obs_y_global += current_particle.y;

      // Associate measurement with map landmark by computing the nearest landmark
      // To that end, iterate over landmarks to find nearest distance
      double nearest_dist = std::numeric_limits<double>::max();

      int id_associated;
      for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {
        // Get landmark data
        double lm_x = map_landmarks.landmark_list[k].x_f;
        double lm_y = map_landmarks.landmark_list[k].y_f;
        int lm_id = map_landmarks.landmark_list[k].id_i;

        // Compute distance
        double lm_dist = dist(obs_x_global, obs_y_global, lm_x, lm_y);
        if (lm_dist < nearest_dist) {
          nearest_dist = lm_dist;
          //id_associated = lm_id;
          // Experimental
          id_associated = k;
        }
      }
      // Set association vector
      associations[j] = id_associated;
      current_particle.dist.push_back(nearest_dist);
    }
    // Write associations into particle
    current_particle.associations = associations;
    particles[i] = current_particle;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

  // Do data association
  dataAssociation(observations, map_landmarks);

  // Loop over particles
  for (int i = 0; i<particles.size(); i++) {
    Particle current_particle = particles[i];
    double particle_x = current_particle.x;
    double particle_y = current_particle.y;

    double new_weight = 1;
    // Loop over observations
    for (int j = 0; j<observations.size(); j++) {


     // new_weight *= exp(-pow(particle_x - lm_x, 2) / (2*std_landmark[0]*std_landmark[0]) -
      //                   pow(particle_y - lm_y, 2) / (2*std_landmark[1]*std_landmark[1]));
      new_weight *= exp(-pow(current_particle.dist[j], 2) / (2*std_landmark[0]*std_landmark[0]));
    }
    // Set new weight
    particles[i].weight = new_weight;
  }
}

void ParticleFilter::resample() {
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  // Get weight vector
  std::vector<double> weights(num_particles);
  for (int i = 0; i<num_particles; i++) {
    weights[i] = particles[i].weight;
  }

  std::default_random_engine generator;
  std::discrete_distribution<int> distribution(weights.begin(), weights.end());

  std::vector<Particle> new_particles(num_particles);
  for (int i=0; i<num_particles; ++i) {
    int number = distribution(generator);
    new_particles[i] = particles[number];
  }
  particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

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
