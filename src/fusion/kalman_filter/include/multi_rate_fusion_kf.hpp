






















#pragma once

#include <Eigen/Dense>

namespace tdt_radar {

/**
 * @brief 高性能多源异构后融合卡尔曼滤波器 (6状态恒定加速度 CA 模型)
 * @details 状态向量 X = [x, y, vx, vy, ax, ay]^T
 *          观测向量 Z = [x, y]^T
 */
class MultiRateFusionKF {
public:
    // 固定的 6 状态维度: [x, y, vx, vy, ax, ay]
    static constexpr int STATE_DIM = 6;
    // 固定的 2 观测维度: [x, y]
    static constexpr int MEAS_DIM = 2;

    using StateVec = Eigen::Matrix<double, STATE_DIM, 1>;
    using StateMat = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
    using MeasVec  = Eigen::Matrix<double, MEAS_DIM, 1>;
    using MeasMat  = Eigen::Matrix<double, MEAS_DIM, MEAS_DIM>;
    using ObsMat   = Eigen::Matrix<double, MEAS_DIM, STATE_DIM>;

    MultiRateFusionKF() : is_initialized_(false) {}

    /**
     * @brief 初始化滤波器
     * @param init_x 初始状态 (x, y)
     * @param proc_noise_std 过程噪声标准差 (加速度扰动，默认1.5)
     */
    void Initialize(const Eigen::Vector2d& init_x, double proc_noise_std = 1.5)
    {
        x_.setZero();
        x_.head<2>() = init_x;

        P_.setIdentity();
        P_ *= 1.0; 
        P_(2, 2) = 10.0; P_(3, 3) = 10.0; // 速度初始不确定性
        P_(4, 4) = 50.0; P_(5, 5) = 50.0; // 加速度初始不确定性

        q_weight_ = proc_noise_std * proc_noise_std;

        H_.setZero();
        H_(0, 0) = 1.0;
        H_(1, 1) = 1.0;

        is_initialized_ = true;
    }

    /**
     * @brief 异步预测步骤 (支持任意时间差 dt)
     * @param dt 距离上一次预测的时间差 (秒)
     */
    void Predict(double dt)
    {
        if (!is_initialized_ || dt <= 0.0) return;

        // 1. 动态构建状态转移矩阵 F(dt)
        StateMat F = StateMat::Identity();
        double dt2 = dt * dt;
        double half_dt2 = 0.5 * dt2;

        F(0, 2) = dt; F(0, 4) = half_dt2;
        F(1, 3) = dt; F(1, 5) = half_dt2;
        F(2, 4) = dt;
        F(3, 5) = dt;

        // 2. 动态构建连续时间白噪声离散化过程噪声 Q(dt)
        StateMat Q = StateMat::Zero();
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        double dt5 = dt4 * dt;

        // X 方向过程噪声
        Q(0, 0) = dt5 / 20.0; Q(0, 2) = dt4 / 8.0;  Q(0, 4) = dt3 / 6.0;
        Q(2, 0) = dt4 / 8.0;  Q(2, 2) = dt3 / 3.0;  Q(2, 4) = dt2 / 2.0;
        Q(4, 0) = dt3 / 6.0;  Q(4, 2) = dt2 / 2.0;  Q(4, 4) = dt;

        // Y 方向过程噪声
        Q(1, 1) = dt5 / 20.0; Q(1, 3) = dt4 / 8.0;  Q(1, 5) = dt3 / 6.0;
        Q(3, 1) = dt4 / 8.0;  Q(3, 3) = dt3 / 3.0;  Q(3, 5) = dt2 / 2.0;
        Q(5, 1) = dt3 / 6.0;  Q(5, 3) = dt2 / 2.0;  Q(5, 5) = dt;

        Q *= q_weight_;

        // 3. 执行预测公式
        x_ = F * x_;
        P_ = F * P_ * F.transpose() + Q;
    }

    /**
     * @brief 异构多源更新步骤 (后融合核心)
     * @param z 观测到的 2D 位置坐标 [x, y]^T
     * @param R_type 观测源类型：'L' = 纯雷达, 'C' = 纯相机, 'F' = 完美融合
     */
    void Update(const Eigen::Vector2d& z, char R_type)
    {
        if (!is_initialized_) return;

        // 根据异构观测源分配不同的测量噪声协方差 R
        MeasMat R = MeasMat::Zero();
        if (R_type == 'L') {
            // 纯雷达：位置极准，噪声极小
            R(0, 0) = 0.04; // 0.2m 误差的平方
            R(1, 1) = 0.04;
        } 
        else if (R_type == 'C') {
            // 纯相机投影：由于远距离视角倾斜，Y方向（纵深）误差大于X方向（横向）
            R(0, 0) = 0.09; // X方向：0.3m 误差
            R(1, 1) = 0.25; // Y方向：0.5m 误差（纵深抖动较强）
        } 
        else if (R_type == 'F') {
            // 双源完美融合：置信度最高
            R(0, 0) = 0.01; // 0.1m 误差的平方
            R(1, 1) = 0.01;
        }

        // 1. 计算卡尔曼增益 K = P * H^T * (H * P * H^T + R)^-1
        // H * P * H^T 实际上就是 P 矩阵的左上角 2x2 块！
        MeasMat S = P_.topLeftCorner<2>(2) + R;
        Eigen::Matrix<double, STATE_DIM, MEAS_DIM> K = P_.leftCols<2>() * S.inverse();

        // 2. 更新状态
        MeasVec residual = z - x_.head<2>();
        x_ = x_ + K * residual;

        // 3. 更新协方差 P = (I - K * H) * P
        StateMat I = StateMat::Identity();
        P_ = (I - K * H_) * P_;
    }

    bool IsInitialized() const { return is_initialized_; }
    StateVec GetState() const { return x_; }
    Eigen::Vector2d GetPosition() const { return x_.head<2>(); }
    Eigen::Vector2d GetVelocity() const { return x_.segment<2>(2); }
    Eigen::Vector2d GetAcceleration() const { return x_.tail<2>(); }

private:
    StateVec x_;          // 状态向量
    StateMat P_;          // 状态协方差矩阵
    ObsMat   H_;          // 观测矩阵
    double   q_weight_;   // 过程噪声强度
    bool     is_initialized_ = false;
};

} // namespace tdt_radar
