#pragma once

#include <Eigen/Geometry>

struct Segment { int start, end; };
enum SegmentBy {
    ANGULAR_VELOCITY,
    ANGULAR_ACCELERATION
};


namespace math
{
    class Math
    {
    public:
        static double radial_angle_between_quats(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2);
        static double radial_to_degrees(const double& angle);
        static double angular_speed_between_quats(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, const double& delta_time);
        static double angular_speed_at_time(const std::vector<Eigen::Quaterniond>& quaternions, const int& time, const double& dt);
        static double angular_acceletation_at_time(const std::vector<Eigen::Quaterniond>& quaternions, const int& time, const double& dt);
        static void align_quaternion_signs(std::vector<Eigen::Quaterniond>& quaternions);
        static void smooth_quaternions(std::vector<Eigen::Quaterniond>& quaternions, double sigma);
        static void smooth_values(std::vector<double>& values, double sigma);
        static std::vector<double> aggregate_quaternion_data(const std::vector<std::vector<Eigen::Quaterniond>>& quaternion_tracks, double dt, SegmentBy segment_by = ANGULAR_ACCELERATION);
        static std::vector<struct Segment> segment_handshapes(const std::vector<double>& values, double low, double high, int min_len);
        static std::vector<struct Segment> segment_handshapes_by_acceleration(const std::vector<double>& acc, double threshold_factor, int min_len);
    };
}