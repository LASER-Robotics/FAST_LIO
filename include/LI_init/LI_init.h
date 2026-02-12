#ifndef LI_INIT_H
#define LI_INIT_H

#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <csignal>
#include <algorithm>
#include <vector>
#include <string>

#include <Eigen/Eigen>
#include <ceres/ceres.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <so3_math.h>
#include <common_lib.h>

#define FILE_DIR(name)     (string(string(ROOT_DIR) + "Log/"+ name))

using namespace std;
using namespace Eigen;

typedef Vector3d V3D;
typedef Matrix3d M3D;
const V3D STD_GRAV = V3D(0, 0, -G_m_s2);

struct CalibState {
    M3D rot_end;
    V3D ang_vel;
    V3D linear_vel;
    V3D ang_acc;
    V3D linear_acc;
    double timeStamp;

    CalibState() {
        rot_end = Eye3d;
        ang_vel = Zero3d;
        linear_vel = Zero3d;
        ang_acc = Zero3d;
        linear_acc = Zero3d;
        timeStamp = 0.0;
    }

    CalibState operator*(const double &coeff) const {
        CalibState a;
        a.ang_vel = this->ang_vel * coeff;
        a.ang_acc = this->ang_acc * coeff;
        a.linear_vel = this->linear_vel * coeff;
        a.linear_acc = this->linear_acc * coeff;
        return a;
    }

    CalibState &operator+=(const CalibState &b) {
        this->ang_vel += b.ang_vel;
        this->ang_acc += b.ang_acc;
        this->linear_vel += b.linear_vel;
        this->linear_acc += b.linear_acc;
        return *this;
    }

    CalibState &operator-=(const CalibState &b) {
        this->ang_vel -= b.ang_vel;
        this->ang_acc -= b.ang_acc;
        this->linear_vel -= b.linear_vel;
        this->linear_acc -= b.linear_acc;
        return *this;
    }
};

struct Angular_Vel_Cost_only_Rot {
    Angular_Vel_Cost_only_Rot(V3D IMU_ang_vel_, V3D Lidar_ang_vel_) :
            IMU_ang_vel(IMU_ang_vel_), Lidar_ang_vel(Lidar_ang_vel_) {}

    template<typename T>
    bool operator()(const T *q, T *residual) const {
        Eigen::Quaternion<T> q_LI{q[0], q[1], q[2], q[3]};
        Eigen::Matrix<T, 3, 1> resi = q_LI.toRotationMatrix() * Lidar_ang_vel.cast<T>() - IMU_ang_vel.cast<T>();
        residual[0] = resi[0]; residual[1] = resi[1]; residual[2] = resi[2];
        return true;
    }

    static ceres::CostFunction *Create(const V3D IMU_ang_vel_, const V3D Lidar_ang_vel_) {
        return (new ceres::AutoDiffCostFunction<Angular_Vel_Cost_only_Rot, 3, 4>(
                new Angular_Vel_Cost_only_Rot(IMU_ang_vel_, Lidar_ang_vel_)));
    }
    V3D IMU_ang_vel, Lidar_ang_vel;
};

struct Angular_Vel_Cost {
    Angular_Vel_Cost(V3D IMU_ang_vel_, V3D IMU_ang_acc_, V3D Lidar_ang_vel_, double deltaT_LI_) :
            IMU_ang_vel(IMU_ang_vel_), IMU_ang_acc(IMU_ang_acc_), Lidar_ang_vel(Lidar_ang_vel_), deltaT_LI(deltaT_LI_) {}

    template<typename T>
    bool operator()(const T *q, const T *b_g, const T *t, T *residual) const {
        Eigen::Quaternion<T> q_LI{q[0], q[1], q[2], q[3]};
        Eigen::Matrix<T, 3, 1> bias_g{b_g[0], b_g[1], b_g[2]};
        T td{t[0]};
        Eigen::Matrix<T, 3, 1> resi = q_LI.toRotationMatrix() * Lidar_ang_vel.cast<T>() - IMU_ang_vel.cast<T>() - (T(deltaT_LI) + td) * IMU_ang_acc.cast<T>() + bias_g;
        residual[0] = resi[0]; residual[1] = resi[1]; residual[2] = resi[2];
        return true;
    }

    static ceres::CostFunction *Create(const V3D IMU_ang_vel_, const V3D IMU_ang_acc_, const V3D Lidar_ang_vel_, const double deltaT_LI_) {
        return (new ceres::AutoDiffCostFunction<Angular_Vel_Cost, 3, 4, 3, 1>(new Angular_Vel_Cost(IMU_ang_vel_, IMU_ang_acc_, Lidar_ang_vel_, deltaT_LI_)));
    }
    V3D IMU_ang_vel, IMU_ang_acc, Lidar_ang_vel; double deltaT_LI;
};

struct Linear_acc_Cost {
    Linear_acc_Cost(CalibState LidarState_, M3D R_LI_, V3D IMU_linear_acc_) :
            LidarState(LidarState_), R_LI(R_LI_), IMU_linear_acc(IMU_linear_acc_) {}

    template<typename T>
    bool operator()(const T *q, const T *b_a, const T *trans, T *residual) const {
        Eigen::Matrix<T, 3, 3> R_LL0_T = LidarState.rot_end.cast<T>();
        Eigen::Quaternion<T> q_GL0{q[0], q[1], q[2], q[3]};
        Eigen::Matrix<T, 3, 1> bias_aL{b_a[0], b_a[1], b_a[2]};
        Eigen::Matrix<T, 3, 1> T_IL{trans[0], trans[1], trans[2]};

        M3D Lidar_omg_SKEW, Lidar_angacc_SKEW;
        Lidar_omg_SKEW << SKEW_SYM_MATRX(LidarState.ang_vel);
        Lidar_angacc_SKEW << SKEW_SYM_MATRX(LidarState.ang_acc);
        Eigen::Matrix<T, 3, 3> Jacob_trans_T = (Lidar_omg_SKEW * Lidar_omg_SKEW + Lidar_angacc_SKEW).cast<T>();

        Eigen::Matrix<T, 3, 1> resi = R_LL0_T * R_LI.transpose().cast<T>() * IMU_linear_acc.cast<T>() - R_LL0_T * bias_aL + q_GL0.toRotationMatrix() * STD_GRAV.cast<T>() - LidarState.linear_acc.cast<T>() - R_LL0_T * Jacob_trans_T * T_IL;
        residual[0] = resi[0]; residual[1] = resi[1]; residual[2] = resi[2];
        return true;
    }

    static ceres::CostFunction *Create(const CalibState LidarState_, const M3D R_LI_, const V3D IMU_linear_acc_) {
        return (new ceres::AutoDiffCostFunction<Linear_acc_Cost, 3, 4, 3, 3>(new Linear_acc_Cost(LidarState_, R_LI_, IMU_linear_acc_)));
    }
    CalibState LidarState; M3D R_LI; V3D IMU_linear_acc;
};

class LI_Init {
public:
    double data_accum_length;
    LI_Init();
    ~LI_Init();

    void push_ALL_IMU_CalibState(const sensor_msgs::msg::Imu::SharedPtr msg, const double &mean_acc_norm);
    void push_Lidar_CalibState(const M3D &rot, const V3D &omg, const V3D &linear_vel, const double &timestamp);
    bool data_sufficiency_assess(MatrixXd &Jacobian_rot, int &frame_num, V3D &lidar_omg, int &orig_odom_freq, int &cut_frame_num);
    void LI_Initialization(int &orig_odom_freq, int &cut_frame_num, double &timediff_imu_wrt_lidar, const double &move_start_time);
    
    V3D get_Grav_L0() { return Grav_L0; }
    M3D get_R_LI() { return Rot_Lidar_wrt_IMU; }
    V3D get_T_LI() { return Trans_Lidar_wrt_IMU; }
    V3D get_gyro_bias() { return gyro_bias; }
    V3D get_acc_bias() { return acc_bias; }
    void IMU_buffer_clear() { IMU_state_group_ALL.clear(); }
    double get_total_time_lag() { return time_delay_IMU_wtr_Lidar; }

private:
    deque<CalibState> IMU_state_group, Lidar_state_group, IMU_state_group_ALL;
    M3D Rot_Lidar_wrt_IMU;
    V3D Trans_Lidar_wrt_IMU, gyro_bias, acc_bias, Grav_L0;
    double time_delay_IMU_wtr_Lidar, time_lag_1, time_lag_2;

    void solve_Rotation_only();
    void solve_Rot_bias_gyro(double &timediff_imu_wrt_lidar);
    void solve_trans_biasacc_grav();
    void downsample_interpolate_IMU(const double &move_start_time);
    void xcorr_temporal_init(const double &odom_freq);
    void IMU_time_compensate(const double &lag_time, const bool &is_discard);
};

#endif
