#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <numeric>

#include "math.h"

using namespace math;

double Math::radial_angle_between_quats(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
    Eigen::Quaterniond q1_normalised = q1.normalized();
    Eigen::Quaterniond q2_normalised = q2.normalized();

    double dot = q1_normalised.dot(q2_normalised);
    double clamped_dot = std::clamp(dot, -1.0, 1.0);

    double angle = 2 * std::acos(std::abs(clamped_dot));

    return angle;
}

double Math::radial_to_degrees(const double& angle) {
    return angle * (180.0 / M_PI);
}

double Math::angular_speed_between_quats(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, const double& delta_time) {
    double angle = radial_angle_between_quats(q1, q2);
    return angle / delta_time;
}

double Math::angular_speed_at_time(const std::vector<Eigen::Quaterniond>& quaternions, const int& time, const double& dt) {
    if (time <= 0 || time >= quaternions.size() - 1) {
        throw std::out_of_range("Time index is out of range for the quaternion vector.");
    }

    // Central difference
    const Eigen::Quaterniond& q1 = quaternions[time - 1];
    const Eigen::Quaterniond& q2 = quaternions[time + 1];

    return angular_speed_between_quats(q1, q2, 2 * dt);
}

void Math::align_quaternion_signs(std::vector<Eigen::Quaterniond>& quaternions) {
    if (quaternions.empty()) return;

    for (size_t i = 1; i < quaternions.size(); ++i) {
        double dot_product = quaternions[i - 1].dot(quaternions[i]);
        if (dot_product < 0)
            quaternions[i].coeffs() *= -1;
    }
}

void Math::smooth_quaternions(std::vector<Eigen::Quaterniond>& quaternions, double sigma) {
    if (quaternions.empty()) return;

    // Gaussian kernel
    int kernel_radius = static_cast<int>(std::ceil(3 * sigma));  // 3-sigma rule
    std::vector<double> kernel(2 * kernel_radius + 1);
    double sum = 0.0;

    for (int i = -kernel_radius; i <= kernel_radius; ++i) {
        kernel[i + kernel_radius] = std::exp(-0.5 * (i * i) / (sigma * sigma));
        sum += kernel[i + kernel_radius];
    }

    for (double& value : kernel)
        value /= sum;

    // Smooth the quaternions using the kernel
    std::vector<Eigen::Quaterniond> smoothed_quaternions = quaternions;

    for (int i = 0; i < static_cast<int>(quaternions.size()); ++i) {
        Eigen::Quaterniond smoothed(0, 0, 0, 0);
        double weight_sum = 0.0;

        for (int j = -kernel_radius; j <= kernel_radius; ++j) {
            int idx = i + j;
            if (idx < 0 || idx >= quaternions.size()) continue;

            double weight = kernel[j + kernel_radius];
            smoothed.coeffs() += weight * quaternions[idx].coeffs();
            weight_sum += weight;
        }

        smoothed.coeffs() /= weight_sum;
        smoothed.normalize();
        smoothed_quaternions[i] = smoothed;
    }

    quaternions = smoothed_quaternions;
}

std::vector<double> Math::aggregate_speed(const std::vector<std::vector<Eigen::Quaterniond>>& quaternion_tracks, double dt) {
    const int n = static_cast<int>(quaternion_tracks[0].size());
    std::vector<double> speed(n, 0.0);
    for (int t = 1; t < n - 1; ++t) {
        double sum = 0;
        for (const auto& quaternion_track : quaternion_tracks)
        sum += Math::angular_speed_at_time(quaternion_track, t, dt);
        speed[t] = sum / quaternion_tracks.size();
    }
    speed[0] = speed[1]; speed[n-1] = speed[n-2];
    return speed;
}

std::vector<Segment> Math::segment_handshapes(const std::vector<double>& v,
                                   double low, double high, int min_len)
{
    std::vector<Segment> segments;

    bool in_handshape = v[0] < high;  // Is first frame in a handshape?
    
    int start = 0;
    for (int i = 1; i < (int)v.size(); ++i) {
        if (in_handshape && v[i] > high) {  // Handshape ends
            if (i - start >= min_len) segments.push_back({start, i - 1});
            in_handshape = false;
        } else if (!in_handshape && v[i] < low) {  // Handshape starts
            in_handshape = true; start = i;
        }
    }

    // If the last segment is still going, add it
    if (in_handshape && (int)v.size() - start >= min_len)
        segments.push_back({start, (int)v.size() - 1});
    return segments;
}
