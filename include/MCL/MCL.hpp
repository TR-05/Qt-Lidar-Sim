#ifndef MCL_HPP
#define MCL_HPP

#include <vector>
#include <random>
#include <cmath>
#include "MCL/Types.hpp"

class MCL {
public:
    MCL(int particle_count, double mixture_phi);
    void step(const Pose& odom_delta, const std::vector<LidarData>& scan);
    
    Pose getEstimatedPose() const;
    std::vector<Particle> getParticles();

private:
    int num_particles;
    double mixture_ratio;
    std::vector<Particle> particles;
    std::mt19937 gen;

    double random_double(double min, double max);
    Particle resample();
    void normalizeAndStore(std::vector<Particle>& next_gen);

    struct MotionModel {
        void update(Particle& p, const Pose& delta, std::mt19937& g);
    } motion_model;

    struct SensorModel {
        double calculateWeight(const Pose& p, const std::vector<LidarData>& scan);
        double rayCast(const Pose& p, double world_angle);
    } sensor_model;

    struct DualSampler {
        Pose sample(std::mt19937& g);
    } dual_sampler;
};

#endif