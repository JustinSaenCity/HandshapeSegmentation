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
    if (time <= 0 || time >= quaternions.size() - 1)
        throw std::out_of_range("Time index is out of range for the quaternion vector.");


    // Central difference
    const Eigen::Quaterniond& q1 = quaternions[time - 1];
    const Eigen::Quaterniond& q2 = quaternions[time + 1];

    return angular_speed_between_quats(q1, q2, 2 * dt);
}

double Math::angular_acceletation_at_time(const std::vector<Eigen::Quaterniond>& quaternions, const int& time, const double& dt) {
    if (time <= 0 || time >= quaternions.size() - 1)
        throw std::out_of_range("Time index is out of range for the quaternion vector.");

    // Central difference
    const Eigen::Quaterniond& q1 = quaternions[time - 1];
    const Eigen::Quaterniond& q2 = quaternions[time];
    const Eigen::Quaterniond& q3 = quaternions[time + 1];

    double speed1 = angular_speed_between_quats(q1, q2, dt);
    double speed2 = angular_speed_between_quats(q2, q3, dt);

    return (speed2 - speed1) / dt;
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

std::vector<double> Math::aggregate_quaternion_data(const std::vector<std::vector<Eigen::Quaterniond>>& quaternion_tracks, double dt, SegmentBy segment_by) {
    const int n = static_cast<int>(quaternion_tracks[0].size());
    std::vector<double> speed(n, 0.0);
    for (int t = 1; t < n - 1; ++t) {
        double sum = 0;
        for (const auto& quaternion_track : quaternion_tracks) {
            switch (segment_by) {
                case ANGULAR_VELOCITY:
                    sum += Math::angular_speed_at_time(quaternion_track, t, dt);
                    break;
                case ANGULAR_ACCELERATION:
                    sum += Math::angular_acceletation_at_time(quaternion_track, t, dt);
                    break;
            }
        }
        // sum += Math::angular_speed_at_time(quaternion_track, t, dt);
        speed[t] = sum / quaternion_tracks.size();
    }
    speed[0] = speed[1]; speed[n-1] = speed[n-2];
    return speed;
}

std::vector<Segment> Math::segment_handshapes(const std::vector<double>& values,
                                   double low, double high, int min_len)
{
    std::vector<Segment> segments;

    bool in_handshape = values[0] < high;  // Is first frame in a handshape?
    
    int start = 0;
    for (int i = 1; i < (int)values.size(); ++i) {
        if (in_handshape && values[i] > high) {  // Handshape ends
            if (i - start >= min_len) segments.push_back({start, i - 1});
            in_handshape = false;
        } else if (!in_handshape && values[i] < low) {  // Handshape starts
            in_handshape = true;
            start = i;
        }
    }

    // If the last segment is still going, add it
    if (in_handshape && (int)values.size() - start >= min_len)
        segments.push_back({start, (int)values.size() - 1});
    return segments;
}

struct Extremum { int index; bool is_peak; };

std::vector<Segment> Math::segment_handshapes_by_acceleration(const std::vector<double>& acc, double threshold_factor, int min_len, const double min_threshold)
{
    std::vector<Segment> segments;
    const int n = static_cast<int>(acc.size());
    if (n < 3) return segments;

    // Set threshold based on the stddev and the threshold factor
    double mean = std::accumulate(acc.begin(), acc.end(), 0.0) / n;
    double var = 0.0;
    for (double cur_a : acc) var += (cur_a - mean) * (cur_a - mean);
    const double dynamic_threshold = threshold_factor * std::sqrt(var / n);

    const double threshold = std::max(dynamic_threshold, min_threshold);

    // Detect all extrema above the threshold
    std::vector<Extremum> found;
    for (int i = 1; i < n - 1; ++i) {
        if (acc[i] > threshold && acc[i] > acc[i - 1] && acc[i] >= acc[i + 1])
            found.push_back({i, true});
        else if (acc[i] < -threshold && acc[i] < acc[i - 1] && acc[i] <= acc[i + 1])
            found.push_back({i, false});
    }

    // Combine consecutive extrema of the same type, note that the strongest extremum is kept
    std::vector<Extremum> extrema;
    for (const auto& cur_extremum : found) {
        if (!extrema.empty() && extrema.back().is_peak == cur_extremum.is_peak) {
            bool stronger = cur_extremum.is_peak ? acc[cur_extremum.index] > acc[extrema.back().index]
                                                 : acc[cur_extremum.index] < acc[extrema.back().index];
            if (stronger) extrema.back() = cur_extremum;
        } else {
            extrema.push_back(cur_extremum);
        }
    }
    if (extrema.empty()) return {{0, n - 1}};  // If no extrema were found, it is one long segment

    // Helper function to add a segment if it meets the minimum length requirement
    auto add = [&](int start, int end) {
        if (end - start + 1 >= min_len) segments.push_back({start, end});
    };

    // If the first extremum is a peak, the first segment is from 0 to the first peak
    size_t k = 0;
    if (extrema[0].is_peak) {
        add(0, extrema[0].index);
        k = 1;
    }

    // Iterate through the extrema in pairs (valley, peak) to define segments
    for (; k < extrema.size(); k += 2) {  // extrema[k] is always a valley, extrema[k+1] is always a peak
        if (k + 1 < extrema.size())
            add(extrema[k].index, extrema[k + 1].index);
        else
            add(extrema[k].index, n - 1);
    }
    return segments;
}

void Math::smooth_values(std::vector<double>& values, double sigma) {
    if (values.empty()) return;

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

    // Smooth the values using the kernel
    std::vector<double> smoothed_values = values;

    for (int i = 0; i < static_cast<int>(values.size()); ++i) {
        double smoothed = 0.0;
        double weight_sum = 0.0;

        for (int j = -kernel_radius; j <= kernel_radius; ++j) {
            int idx = i + j;
            if (idx < 0 || idx >= values.size()) continue;

            double weight = kernel[j + kernel_radius];
            smoothed += weight * values[idx];
            weight_sum += weight;
        }

        smoothed /= weight_sum;
        smoothed_values[i] = smoothed;
    }

    values = smoothed_values;
}
