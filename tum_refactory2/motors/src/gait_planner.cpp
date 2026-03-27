#include "gait_planner.hpp"
#include <cmath>

TrajectoryPoint3D generate_bezier_swing(double phase, double duty_factor, const Eigen::Vector3d &p_neutral,
                                        const Eigen::Vector3d &hip_offset, double vx, double vy,
                                        double omega_z, double t_stance, double t_swing,
                                        double step_height)
{
    Eigen::Vector3d v_linear(vx, vy, 0.0);
    Eigen::Vector3d omega_vec(0, 0, omega_z);
    Eigen::Vector3d v_rotation = omega_vec.cross(hip_offset);

    Eigen::Vector3d p_target = p_neutral + (v_linear + v_rotation) * (t_stance / 2.0);
    Eigen::Vector3d p_start = p_neutral - (v_linear + v_rotation) * (t_stance / 2.0);

    double swing_portion = 1.0 - duty_factor;
    double t = phase / swing_portion;

    Eigen::Vector3d p0 = p_start; p0.z() = p_neutral.z();
    Eigen::Vector3d p1 = p_start; p1.z() = p_neutral.z();
    Eigen::Vector3d p2 = p_target; p2.z() = p_neutral.z();
    Eigen::Vector3d p3 = p_target; p3.z() = p_neutral.z();

    double u = 1.0 - t;
    TrajectoryPoint3D res;
    res.pos = (u * u * u) * p0 + 3 * (u * u) * t * p1 + 3 * u * (t * t) * p2 + (t * t * t) * p3;
    res.pos.z() = p_neutral.z() + step_height * (1.0 - cos(2.0 * M_PI * t)) / 2.0;

    if (t_swing > 1e-5) {
        res.acc = (6 * u * (p2 - 2 * p1 + p0) + 6 * t * (p3 - 2 * p2 + p1)) / (t_swing * t_swing);
        res.acc.z() = step_height * 2.0 * (M_PI * M_PI) * cos(2.0 * M_PI * t) / (t_swing * t_swing);
    } else {
        res.acc = Eigen::Vector3d::Zero();
    }
    return res;
}

Atom51GaitPlanner::Atom51GaitPlanner() {
    for (int i = 0; i < 4; ++i) last_pos_[i].setZero();
}

TrajectoryOutput Atom51GaitPlanner::update(double dt, double elapsed_time, int leg_id,
                        double vx, double vy, double wz,
                        double step_duration, double step_h,
                        double duty_factor, const double gait_offsets[4])
{
    TrajectoryOutput out;
    double global_phase = fmod(elapsed_time / step_duration, 1.0);
    double leg_phase = fmod(global_phase + gait_offsets[leg_id], 1.0);
    double swing_portion = 1.0 - duty_factor;
    out.is_stance = (leg_phase >= swing_portion);

    const Eigen::Vector3d LEGS_STAND_XYZ[4] = {
        Eigen::Vector3d(-0.02, 0.093, -0.306),  
        Eigen::Vector3d(-0.02, -0.093, -0.306), 
        Eigen::Vector3d(-0.02, 0.093, -0.306),  
        Eigen::Vector3d(-0.02, -0.093, -0.306)  
    };
    const Eigen::Vector3d HIP_OFFSETS[4] = {
        Eigen::Vector3d(0.0, 0.093, 0.0),
        Eigen::Vector3d(0.0, -0.093, 0.0),
        Eigen::Vector3d(0.0, 0.093, 0.0),
        Eigen::Vector3d(0.0, -0.093, 0.0)};

    Eigen::Vector3d origin = LEGS_STAND_XYZ[leg_id];
    Eigen::Vector3d hip_offset = HIP_OFFSETS[leg_id];
    Eigen::Vector3d v_total = Eigen::Vector3d(vx, vy, 0.0) + Eigen::Vector3d(0, 0, wz).cross(hip_offset);

    if (out.is_stance) {
        double t = (leg_phase - swing_portion) / duty_factor;
        double t_stance = step_duration * duty_factor;
        Eigen::Vector3d p_start = origin - v_total * (t_stance / 2.0);
        Eigen::Vector3d p_target = origin + v_total * (t_stance / 2.0);

        out.position = p_target + t * (p_start - p_target);
        out.position.z() = origin.z();
        out.velocity = (p_start - p_target) / t_stance;
        out.acceleration = Eigen::Vector3d::Zero();
    } else {
        double t_stance = step_duration * duty_factor;
        double t_swing = step_duration * swing_portion;

        TrajectoryPoint3D tp = generate_bezier_swing(leg_phase, duty_factor, origin,
                                                     hip_offset, vx, vy, wz,
                                                     t_stance, t_swing, step_h);
        out.position = tp.pos;
        out.acceleration = tp.acc;
        out.velocity = (out.position - last_pos_[leg_id]) / dt;
    }

    last_pos_[leg_id] = out.position;
    return out;
}
