#include "MCL/MCL.hpp"
#include <qmath.h>
#include "config.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

MCL::MCL(int particle_count, double mixture_phi) 
    : num_particles(particle_count), mixture_ratio(mixture_phi), gen(std::random_device{}()) {
    particles.clear();
    for (int i = 0; i < num_particles; ++i) {
        particles.push_back({{random_double(0, 144), random_double(0, 144), random_double(0, 2*M_PI)}, 1.0/num_particles});
    }
}

void MCL::step(const Pose& odom_delta, const std::vector<LidarData>& scan) {
    if (std::isnan(odom_delta.x) || std::isnan(odom_delta.y)) return;

    // 1. Prediction (Motion) & Weighting
    // We update position BEFORE weighting so the particles are evaluated where the robot is NOW.
    double total_w = 0;
    for (auto& p : particles) {
        motion_model.update(p, odom_delta, gen);

        if (p.pose.x < 0.0 || p.pose.x > 144.0 || p.pose.y < 0.0 || p.pose.y > 144.0) {
            p.weight = 1e-18;
        } else {
            p.weight = sensor_model.calculateWeight(p.pose, scan);
        }
        total_w += p.weight;
    }

    // 2. Adaptive Recovery
    // If average weight is tiny, we are likely "lost" due to high speed. Boost mixture.
    double avg_w = total_w / num_particles;
    double effective_mixture = (avg_w < 1e-13) ? 0.20 : mixture_ratio;

    // 3. Normalization
    if (total_w > 1e-18) {
        for (auto& p : particles) p.weight /= total_w;
    } else {
        for (auto& p : particles) p.weight = 1.0 / num_particles;
    }

    // 4. Low-Variance Resampling
    // This is the most stable sampler for simulations.
    std::vector<Particle> next_generation;
    next_generation.reserve(num_particles);
    
    double M_inv = 1.0 / num_particles;
    double r = std::uniform_real_distribution<double>(0, M_inv)(gen);
    double c = particles[0].weight;
    int i = 0;

    for (int m = 0; m < num_particles; ++m) {
        double U = r + m * M_inv;
        while (U > c && i < num_particles - 1) {
            i++;
            c += particles[i].weight;
        }

        Particle p = particles[i];

        // Random Recovery Injection
        if (random_double(0, 1) < effective_mixture) {
            p.pose = dual_sampler.sample(gen);
            p.weight = 1.0 / num_particles;
        }
        
        next_generation.push_back(p);
    }
    particles = next_generation;
}

void MCL::MotionModel::update(Particle& p, const Pose& delta, std::mt19937& g) {
    // Distance-proportional noise. 
    // High speeds create a larger "search area" for the particles.
    double dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);
    
    // 12% linear noise + small constant to prevent stagnation
    double lin_std = dist * 0.12 + 0.05;
    double ang_std = std::abs(delta.theta) * 0.1 + 0.02;

    std::normal_distribution<double> noise_lin(0, lin_std);
    std::normal_distribution<double> noise_ang(0, ang_std);
    
    p.pose.x += delta.x + noise_lin(g);
    p.pose.y += delta.y + noise_lin(g);
    p.pose.theta += delta.theta + noise_ang(g);
    
    p.pose.theta = std::fmod(p.pose.theta, 2 * M_PI);
    if (p.pose.theta < 0) p.pose.theta += 2 * M_PI;
}



double MCL::SensorModel::calculateWeight(const Pose& p, const std::vector<LidarData>& scan) {
    double log_prob = 0.0;
    const double sigma_hit = 2.0; // Slightly wider for high speed stability
    const double z_hit = 0.95;
    const double z_rand = 0.05;

    int skip = std::max(1, (int)scan.size() / 10);
    
    for (size_t i = 0; i < scan.size(); i += skip) {
        double expected = rayCast(p, scan[i].angle);
        double dist_diff = scan[i].length - expected;
        
        double p_hit = std::exp(-(dist_diff * dist_diff) / (2 * sigma_hit * sigma_hit));
        log_prob += std::log(z_hit * p_hit + z_rand);
    }
    
    // Squaring the result sharpens the clump without the "instability" of cubes.
    double weight = std::exp(log_prob);
    return (std::isnan(weight) || weight < 1e-18) ? 1e-18 : std::pow(weight, 2.0);
}

double MCL::SensorModel::rayCast(const Pose& p, double world_angle) {
    double dx = std::cos(world_angle);
    double dy = std::sin(world_angle);

    if (std::abs(dx) < 1e-6) dx = (dx >= 0) ? 1e-6 : -1e-6;
    if (std::abs(dy) < 1e-6) dy = (dy >= 0) ? 1e-6 : -1e-6;

    double tMin = 1e6;
    // Field Walls
    if (dx > 0) tMin = std::min(tMin, (144.0 - p.x) / dx);
    else if (dx < 0) tMin = std::min(tMin, -p.x / dx);
    if (dy > 0) tMin = std::min(tMin, (144.0 - p.y) / dy);
    else if (dy < 0) tMin = std::min(tMin, -p.y / dy);

    // Dynamic Obstacles
    for (const auto& obs : WorldObstacles) {
        double ocX = p.x - obs.x;
        double ocY = p.y - obs.y;
        double b = ocX * dx + ocY * dy;
        double c = (ocX * ocX + ocY * ocY) - (obs.radius * obs.radius);
        double discriminant = b * b - c;

        if (discriminant > 0) {
            double t = -b - std::sqrt(discriminant);
            if (t > 0 && t < tMin) tMin = t;
        }
    }
    return tMin;
}

Pose MCL::getEstimatedPose() const {
    if (particles.empty()) return {72.0, 72.0, 0.0};

    auto best_it = std::max_element(particles.begin(), particles.end(), 
        [](const Particle& a, const Particle& b) { return a.weight < b.weight; });
    
    Pose best_p = best_it->pose;

    // Robust averaging: ignore scouts by only averaging particles near the best one.
    double x = 0, y = 0, s = 0, c = 0, total_w = 0;
    const double radius = 5.0; 

    for (const auto& p : particles) {
        double dx = p.pose.x - best_p.x;
        double dy = p.pose.y - best_p.y;
        if ((dx*dx + dy*dy) < (radius * radius)) {
            x += p.pose.x * p.weight;
            y += p.pose.y * p.weight;
            s += std::sin(p.pose.theta) * p.weight;
            c += std::cos(p.pose.theta) * p.weight;
            total_w += p.weight;
        }
    }

    if (total_w < 1e-12) return best_p;
    return {x / total_w, y / total_w, std::atan2(s, c)};
}

Pose MCL::DualSampler::sample(std::mt19937& g) {
    std::uniform_real_distribution<double> d(5, 139), r(0, 2*M_PI);
    return {d(g), d(g), r(g)};
}

double MCL::random_double(double min, double max) {
    return std::uniform_real_distribution<double>(min, max)(gen);
}

std::vector<Particle> MCL::getParticles() { return particles; }