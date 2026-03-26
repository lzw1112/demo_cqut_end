#include "gazebo_utils/include/gazebo_utils.hpp"

namespace gazebo_plugins
{

namespace noise
{

Eigen::Vector2d gaussiaNoise2D(double mu, double sigma) 
{
    // using Box-Muller transform to generate two independent standard normally disbributed normal variables
    // see wikipedia
    double U = (double) std::rand() / (double) RAND_MAX; // normalized uniform random variable
    double V = (double) std::rand() / (double) RAND_MAX; // normalized uniform random variable
    double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
    double Y = sqrt(-2.0 * ::log(U)) * sin(2.0 * M_PI * V); // the other indep. normal variable
    // we'll just use X
    // scale to our mu and sigma
    X = sigma * X + mu;
    Y = sigma * Y + mu;
    return Eigen::Vector2d(X, Y);
}

Eigen::Vector3d gaussiaNoise3D(double mu, double sigma) 
{
    const auto vect_2d = gaussiaNoise2D(mu, sigma);
    return Eigen::Vector3d(vect_2d.x(), vect_2d.y(), 0.0);
}

double getGaussianNoise(double mean, double var) 
{
    std::normal_distribution<double> distribution(mean, var);
    // construct a trivial random generator engine from a time-based seed:
    long seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    return distribution(generator);
}

bool probability(const double likelihood, double &lik_res) 
{
    const double rand = (double) std::rand() / (double) RAND_MAX; // normalized uniform random variable
    lik_res = std::max(likelihood, rand);
    return rand <= likelihood;
}

}

}