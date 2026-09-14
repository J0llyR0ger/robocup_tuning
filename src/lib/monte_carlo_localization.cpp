#include "lib/monte_carlo_localization.hpp"
#include "utils.hpp"
#undef B1
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

MonteCarloLocalization::MonteCarloLocalization(const FieldMap &field_map) : field_map(field_map) {
    this->set_initial_pose(
        {
            .position = Eigen::Vector2f::Zero(),
            .heading = 0.0f,
        },
        INITIAL_POSITION_NOISE, INITIAL_HEADING_NOISE);
}

void MonteCarloLocalization::set_field_map(const FieldMap &field_map) {
    this->field_map = field_map;
}

void MonteCarloLocalization::set_initial_pose(const Pose &pose, float position_sigma,
                                              float heading_sigma) {
    for (size_t i = 0; i < this->particles.size(); i++) {
        this->particles[i].pose.position =
            pose.position +
            Eigen::Vector2f(this->random_gaussian(), this->random_gaussian()) * position_sigma;

        this->particles[i].pose.heading =
            wrap_heading(pose.heading + this->random_gaussian() * heading_sigma);

        this->particles[i].weight = 1.0f / (float)this->particles.size();
    }

    this->estimated_pose = pose;
}

void MonteCarloLocalization::predict(const Eigen::Vector2f &robot_travel, float heading_change) {
    float travel_distance = robot_travel.norm();

    float position_sigma = std::max(MIN_POSITION_NOISE, POSITION_NOISE_PER_METER * travel_distance);

    float heading_sigma =
        std::max(MIN_HEADING_NOISE, HEADING_NOISE_PER_RADIAN * fabsf(heading_change));

    for (size_t i = 0; i < this->particles.size(); i++) {
        Eigen::Vector2f noisy_travel =
            robot_travel +
            Eigen::Vector2f(this->random_gaussian(), this->random_gaussian()) * position_sigma;

        float noisy_heading_change = heading_change + this->random_gaussian() * heading_sigma;

        float heading = this->particles[i].pose.heading;
        float heading_sin = sinf(heading);
        float heading_cos = cosf(heading);

        Eigen::Vector2f world_travel(
            heading_cos * noisy_travel.x() + heading_sin * noisy_travel.y(),
            -heading_sin * noisy_travel.x() + heading_cos * noisy_travel.y());

        this->particles[i].pose.position += world_travel;
        this->particles[i].pose.heading =
            wrap_heading(this->particles[i].pose.heading + noisy_heading_change);
    }

    this->compute_estimated_pose();
}

// ----- Particle filter measurement step (beam model) -----

void MonteCarloLocalization::update_beam_model(etl::span<const LidarResponsePoint> points) {
    if (points.empty() || this->field_map.wall_count() == 0) {
        this->compute_estimated_pose();
        return;
    }

    size_t beam_count = std::min((size_t)40, points.size());
    size_t step = std::max((size_t)1, points.size() / beam_count);

    constexpr float MIN_PROBABILITY = 1e-6f;
    float sigma_sq = LIDAR_NOISE * LIDAR_NOISE;

    std::array<float, NUM_PARTICLES> log_weights;
    float max_log_weight = -INFINITY;

    for (size_t particle_idx = 0; particle_idx < this->particles.size(); particle_idx++) {
        const Particle &particle = this->particles[particle_idx];

        float particle_log_weight = 0.0f;

        for (size_t beam_idx = 0; beam_idx < beam_count; beam_idx++) {
            const LidarResponsePoint &beam_point =
                points[points.size() - 1 - std::min(points.size() - 1, beam_idx * step)];

            float measured_range = beam_point.position.norm();

            if (measured_range > LIDAR_MAX_DISTANCE) {
                continue;
            }

            float heading = particle.pose.heading;
            float heading_sin = sinf(heading);
            float heading_cos = cosf(heading);

            Eigen::Vector2f world_direction(
                heading_cos * beam_point.position.x() + heading_sin * beam_point.position.y(),
                -heading_sin * beam_point.position.x() + heading_cos * beam_point.position.y());

            float expected_range = this->field_map.raycast(particle.pose.position, world_direction,
                                                           LIDAR_MAX_DISTANCE);

            float error = measured_range - expected_range;

            float p_hit = expf(-0.5f * (error * error) / sigma_sq);
            float p_rand = 1.0f / LIDAR_MAX_DISTANCE;

            float beam_probability = 0.90f * p_hit + 0.10f * p_rand;

            particle_log_weight += logf(std::max(beam_probability, MIN_PROBABILITY));
        }

        log_weights[particle_idx] = particle_log_weight;
        max_log_weight = std::max(max_log_weight, particle_log_weight);
    }

    float total_weight = 0.0f;

    for (size_t i = 0; i < this->particles.size(); i++) {
        float weight = expf(log_weights[i] - max_log_weight);
        this->particles[i].weight = weight;
        total_weight += weight;
    }

    if (total_weight <= 0.0f) {
        float uniform_weight = 1.0f / (float)this->particles.size();
        for (size_t i = 0; i < this->particles.size(); i++) {
            this->particles[i].weight = uniform_weight;
        }
    } else {
        this->normalize_weights();
    }

    this->low_variance_resample();
    this->compute_estimated_pose();
}

Pose MonteCarloLocalization::get_estimated_pose() const { return this->estimated_pose; }

float MonteCarloLocalization::get_position_uncertainty() const {
    float weighted_variance = 0.0f;

    for (size_t i = 0; i < this->particles.size(); i++) {
        const Particle &particle = this->particles[i];

        Eigen::Vector2f error = particle.pose.position - this->estimated_pose.position;

        weighted_variance += particle.weight * error.squaredNorm();
    }

    return sqrtf(std::max(0.0f, weighted_variance));
}

float MonteCarloLocalization::random_uniform() {
    uint32_t x = this->rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    this->rng_state = x;

    return (float)x / (float)UINT32_MAX;
}

float MonteCarloLocalization::random_symmetric() { return 2.0f * this->random_uniform() - 1.0f; }

float MonteCarloLocalization::random_gaussian() {
    float sum = 0.0f;

    for (int i = 0; i < 6; i++) {
        sum += this->random_symmetric();
    }

    return sum * 0.5f;
}

void MonteCarloLocalization::normalize_weights() {
    float total_weight = 0.0f;

    for (size_t i = 0; i < this->particles.size(); i++) {
        total_weight += this->particles[i].weight;
    }

    if (total_weight <= 0.0f) {
        float uniform = 1.0f / (float)this->particles.size();

        for (size_t i = 0; i < this->particles.size(); i++) {
            this->particles[i].weight = uniform;
        }

        return;
    }

    for (size_t i = 0; i < this->particles.size(); i++) {
        this->particles[i].weight /= total_weight;
    }
}

void MonteCarloLocalization::low_variance_resample() {
    this->normalize_weights();

    std::array<Particle, NUM_PARTICLES> resampled_particles;

    float interval = 1.0f / (float)this->particles.size();
    float offset = this->random_uniform() * interval;

    float cumulative = this->particles[0].weight;
    size_t source_idx = 0;

    for (size_t sample_idx = 0; sample_idx < this->particles.size(); sample_idx++) {
        float target = offset + (float)sample_idx * interval;

        while (target > cumulative && source_idx < this->particles.size() - 1) {
            source_idx++;
            cumulative += this->particles[source_idx].weight;
        }

        resampled_particles[sample_idx].pose = this->particles[source_idx].pose;
        resampled_particles[sample_idx].weight = interval;
    }

    this->particles = resampled_particles;
}

void MonteCarloLocalization::compute_estimated_pose() {
    float weighted_x = 0.0f;
    float weighted_y = 0.0f;
    float weighted_sin = 0.0f;
    float weighted_cos = 0.0f;

    for (size_t i = 0; i < this->particles.size(); i++) {
        const Particle &particle = this->particles[i];

        weighted_x += particle.pose.position.x() * particle.weight;
        weighted_y += particle.pose.position.y() * particle.weight;

        weighted_sin += sinf(particle.pose.heading) * particle.weight;
        weighted_cos += cosf(particle.pose.heading) * particle.weight;
    }

    this->estimated_pose.position = {weighted_x, weighted_y};
    this->estimated_pose.heading = wrap_heading(atan2f(weighted_sin, weighted_cos));
}
