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
	num_particles = 3;

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
	std::cout << particles[num_particles - 1].x;


}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	std::cout << "Yaw rate = " << yaw_rate << std::endl;
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
		double x_new = x + velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
		double y_new = y + velocity / yaw_rate * (-cos(theta + yaw_rate * delta_t) + cos(theta));
		double theta_new = theta + yaw_rate * delta_t;

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
  // Print measurements; for debugging only
  for (int j = 0; j < n_obs; j++) {
    std::cout << "Observation number " << j << " has x-coordinate " << observations[j].x << " and y-coordinate " <<
              observations[j].y << "\n\n";
  }

  // Loop over particles
  for (int i = 0; i < num_particles; i++) {
    // Associate current particle in loop with variable
    Particle current_particle = particles[i];

    // Initialize associations for current particle
    std::vector<int> associations(n_obs);
    std::vector<double> sense_x(n_obs);
    std::vector<double> sense_y(n_obs);

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

      std::cout << "\n\nx coordinate for observation " << j << " of particle " << i << ": " << obs_x_global;
      std::cout << "\ny coordinate for observation " << j << " of particle " << i << ": " << obs_y_global;

      // Associate measurement with map landmark by computing the nearest landmark
      // To that end, iterate over landmarks to find nearest distance
      double nearest_dist = std::numeric_limits<double>::max();
      double nearest_x;
      double nearest_y;

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
          id_associated = lm_id;
          nearest_x = lm_x;
          nearest_y = lm_y;
        }
      }
      // Set association vector
      associations[j] = id_associated;
      sense_x[j] = nearest_x;
      sense_y[j] = nearest_y;

      std::cout << "\nAssociated landmark id is " << id_associated << "\n\n";
    }
    // Write associations into particle
    current_particle.associations = associations;
    current_particle.sense_x = sense_x;
    current_particle.sense_y = sense_y;
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

  // Do data association
  dataAssociation(observations, map_landmarks);
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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
