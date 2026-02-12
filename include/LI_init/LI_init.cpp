#include "LI_init.h"

LI_Init::LI_Init() : time_delay_IMU_wtr_Lidar(0.0), time_lag_1(0.0), time_lag_2(0.0) {
    data_accum_length = 500;
    Rot_Lidar_wrt_IMU = Eye3d;
    Trans_Lidar_wrt_IMU = Zero3d;
    gyro_bias = Zero3d;
    acc_bias = Zero3d;
    Grav_L0 = V3D(0, 0, -G_m_s2);
}

LI_Init::~LI_Init() = default;

void LI_Init::push_ALL_IMU_CalibState(const sensor_msgs::msg::Imu::SharedPtr msg, const double &mean_acc_norm) {
    CalibState IMUstate;
    IMUstate.ang_vel = V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    IMUstate.linear_acc = V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) / mean_acc_norm * G_m_s2;
    IMUstate.timeStamp = rclcpp::Time(msg->header.stamp).seconds(); 
    IMU_state_group_ALL.push_back(IMUstate);
}

void LI_Init::push_Lidar_CalibState(const M3D &rot, const V3D &omg, const V3D &linear_vel, const double &timestamp) {
    CalibState Lidarstate;
    Lidarstate.rot_end = rot;
    Lidarstate.ang_vel = omg;
    Lidarstate.linear_vel = linear_vel;
    Lidarstate.timeStamp = timestamp;
    Lidar_state_group.push_back(Lidarstate);
}

bool LI_Init::data_sufficiency_assess(MatrixXd &Jacobian_rot, int &frame_num, V3D &lidar_omg, int &orig_odom_freq, int &cut_frame_num) {
    M3D lidar_omg_skew;
    lidar_omg_skew << SKEW_SYM_MATRX(lidar_omg);
    Jacobian_rot.block<3, 3>(3 * frame_num, 0) = lidar_omg_skew;

    if (frame_num % (orig_odom_freq * cut_frame_num) == 0 && frame_num > 0) {
        M3D Hessian_rot = Jacobian_rot.transpose() * Jacobian_rot;
        EigenSolver<M3D> es(Hessian_rot);
        V3D EigenValue = es.eigenvalues().real();
        V3D Scaled_Eigen = EigenValue / data_accum_length;
        std::cout << "Tentou Otimizar" << std::endl;
        std::cout << "EigenValue: " << EigenValue<< std::endl;
        std::cout << "Scaled: " << Scaled_Eigen.minCoeff() << std::endl;
        if (Scaled_Eigen.minCoeff() > 0.9) return true;
    }
    return false;
}

void LI_Init::downsample_interpolate_IMU(const double &move_start_time) {
    while (!IMU_state_group_ALL.empty() && IMU_state_group_ALL.front().timeStamp < move_start_time - 1.0)
        IMU_state_group_ALL.pop_front();
    
    for (size_t i = 0; i < Lidar_state_group.size(); i++) {
        for (size_t j = 1; j < IMU_state_group_ALL.size(); j++) {
            if (IMU_state_group_ALL[j-1].timeStamp <= Lidar_state_group[i].timeStamp && 
                IMU_state_group_ALL[j].timeStamp > Lidar_state_group[i].timeStamp) {
                double t_diff = IMU_state_group_ALL[j].timeStamp - IMU_state_group_ALL[j-1].timeStamp;
                double ratio = (Lidar_state_group[i].timeStamp - IMU_state_group_ALL[j-1].timeStamp) / t_diff;
                CalibState interp;
                interp.ang_vel = (1-ratio)*IMU_state_group_ALL[j-1].ang_vel + ratio*IMU_state_group_ALL[j].ang_vel;
                interp.linear_acc = (1-ratio)*IMU_state_group_ALL[j-1].linear_acc + ratio*IMU_state_group_ALL[j].linear_acc;
                interp.timeStamp = Lidar_state_group[i].timeStamp;
                IMU_state_group.push_back(interp);
                break;
            }
        }
    }
}

void LI_Init::xcorr_temporal_init(const double &odom_freq) {
    int N = IMU_state_group.size();
    double max_corr = -1e10;
    int best_lag = 0;
    for (int lag = -N/2; lag < N/2; lag++) {
        double corr = 0;
        for (int i = 0; i < N; i++) {
            int j = i + lag;
            if (j >= 0 && j < N) corr += IMU_state_group[i].ang_vel.norm() * Lidar_state_group[j].ang_vel.norm();
        }
        if (corr > max_corr) { max_corr = corr; best_lag = lag; }
    }
    time_lag_1 = best_lag / odom_freq;
}

void LI_Init::IMU_time_compensate(const double &lag_time, const bool &is_discard) {
    for (auto &s : IMU_state_group) s.timeStamp -= lag_time;
}

void LI_Init::solve_Rotation_only() {
    double q[4] = {1, 0, 0, 0};
    ceres::Problem problem;
    #if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1
        problem.AddParameterBlock(q, 4, new ceres::QuaternionManifold());
    #else
        problem.AddParameterBlock(q, 4, new ceres::QuaternionParameterization());
    #endif
    for (size_t i = 0; i < IMU_state_group.size(); i++) {
        problem.AddResidualBlock(Angular_Vel_Cost_only_Rot::Create(IMU_state_group[i].ang_vel, Lidar_state_group[i].ang_vel), nullptr, q);
    }
    ceres::Solver::Options opt; ceres::Solver::Summary sum; ceres::Solve(opt, &problem, &sum);
    Rot_Lidar_wrt_IMU = Eigen::Quaterniond(q[0], q[1], q[2], q[3]).toRotationMatrix();
}

void LI_Init::solve_Rot_bias_gyro(double &timediff) {
    double q[4], bg[3] = {0,0,0}, t2 = 0;
    Eigen::Quaterniond eq(Rot_Lidar_wrt_IMU); q[0]=eq.w(); q[1]=eq.x(); q[2]=eq.y(); q[3]=eq.z();
    ceres::Problem prob;
    #if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1
        prob.AddParameterBlock(q, 4, new ceres::QuaternionManifold());
    #else
        prob.AddParameterBlock(q, 4, new ceres::QuaternionParameterization());
    #endif
    for (size_t i = 0; i < IMU_state_group.size(); i++) {
        prob.AddResidualBlock(Angular_Vel_Cost::Create(IMU_state_group[i].ang_vel, V3D(0,0,0), Lidar_state_group[i].ang_vel, 0.0), nullptr, q, bg, &t2);
    }
    ceres::Solver::Options opt; ceres::Solver::Summary sum; ceres::Solve(opt, &prob, &sum);
    Rot_Lidar_wrt_IMU = Eigen::Quaterniond(q[0], q[1], q[2], q[3]).toRotationMatrix();
    gyro_bias = V3D(bg[0], bg[1], bg[2]); time_lag_2 = t2;
    time_delay_IMU_wtr_Lidar = time_lag_1 + time_lag_2;
}

void LI_Init::solve_trans_biasacc_grav() {
    double qg[4] = {1,0,0,0}, ba[3] = {0,0,0}, til[3] = {0,0,0};
    ceres::Problem prob;
    #if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1
        prob.AddParameterBlock(qg, 4, new ceres::QuaternionManifold());
    #else
        prob.AddParameterBlock(qg, 4, new ceres::QuaternionParameterization());
    #endif
    for (size_t i = 0; i < IMU_state_group.size(); i++) {
        prob.AddResidualBlock(Linear_acc_Cost::Create(Lidar_state_group[i], Rot_Lidar_wrt_IMU, IMU_state_group[i].linear_acc), nullptr, qg, ba, til);
    }
    ceres::Solver::Options opt; ceres::Solver::Summary sum; ceres::Solve(opt, &prob, &sum);
    Grav_L0 = Eigen::Quaterniond(qg[0], qg[1], qg[2], qg[3]).toRotationMatrix() * STD_GRAV;
    acc_bias = V3D(ba[0], ba[1], ba[2]); Trans_Lidar_wrt_IMU = V3D(til[0], til[1], til[2]);
}

void LI_Init::LI_Initialization(int &freq, int &cut, double &timediff, const double &move_t) {
    downsample_interpolate_IMU(move_t);
    xcorr_temporal_init(freq * cut);
    IMU_time_compensate(time_lag_1, false);
    solve_Rotation_only();
    solve_Rot_bias_gyro(timediff);
    solve_trans_biasacc_grav();
}
