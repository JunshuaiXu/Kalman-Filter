%% 卡尔曼滤波融合角度与角速度仿真 (MATLAB)
clear; clc; close all;

%% 1. 仿真参数设置
dt = 0.01;                 % 采样步长 (100Hz)
t = 0:dt:10;              % 仿真总时间 10 秒
N = length(t);

% 真实运动生成 (假设物体在做正弦摆动)
true_angle = 30 * sin(2 * pi * 0.5 * t);          % 真实角度 (度)
true_gyro = 30 * 2 * pi * 0.5 * cos(2 * pi * 0.5 * t); % 真实角速度 (度/秒)

%% 2. 模拟传感器带噪声的测量值
gyro_bias = 2.0;          % 陀螺仪常值零偏 (度/秒)
gyro_noise = 1.5;         % 陀螺仪高斯白噪声标准差
acc_noise = 4.0;          % 加速度计算出的角度高斯白噪声标准差

% 传感器输出
z_gyro = true_gyro + gyro_bias + gyro_noise * randn(size(true_gyro)); % 陀螺仪测量值
z_acc = true_angle + acc_noise * randn(size(true_angle));             % 加速度计角度值

%% 3. 卡尔曼滤波器初始化
% 状态向量 X = [angle; bias]
X = [0; 0];               % 初始状态估计

% 状态协方差矩阵 P
P = [1  0; 
     0  1];

% 系统过程噪声协方差 Q (影响预测步)
Q = [0.001   0; 
     0       0.003];      % 调大 Q 意为更信任加速度计，调小意为更信任陀螺仪

% 测量噪声协方差 R (影响更新步)
R = acc_noise^2;          % 加速度计角度的方差

% 状态转移矩阵 A 和 控制输入矩阵 B
A = [1  -dt; 
     0   1];
B = [dt; 
     0];

% 观测矩阵 H
H = [1, 0];

%% 4. 卡尔曼滤波循环迭代
est_angle = zeros(1, N);
est_bias = zeros(1, N);

for k = 1:N
    % 【步骤一：预测 Predict】
    u = z_gyro(k);                          % 当前时刻控制输入（陀螺仪角速度）
    X_pred = A * X + B * u;                 % 状态预测
    P_pred = A * P * A' + Q;                % 协方差预测
    
    % 【步骤二：更新 Update】
    Z = z_acc(k);                           % 当前时刻观测值（加速度计角度）
    y = Z - H * X_pred;                     % 创新值（测量残差）
    S = H * P_pred * H' + R;                % 残差协方差
    K = P_pred * H' / S;                    % 计算卡尔曼增益
    
    X = X_pred + K * y;                     % 更新状态估计
    P = (eye(2) - K * H) * P_pred;          % 更新协方差
    
    % 存储滤波结果
    est_angle(k) = X(1);
    est_bias(k) = X(2);
end

%% 5. 结果绘图与对比
figure('Position', [100, 100, 1000, 600]);

% 子图1：角度融合效果对比
subplot(2,1,1);
plot(t, true_angle, 'k-', 'LineWidth', 2); hold on;
plot(t, z_acc, 'r.', 'MarkerSize', 4);
plot(t, est_angle, 'b-', 'LineWidth', 1.5);
grid on;
legend('真实角度 (True)', '加速度计测量值 (Acc Noise)', '卡尔曼融合角度 (Kalman Filter)');
xlabel('时间 (秒)');
ylabel('角度 (度)');
title('卡尔曼滤波角速度与角度融合结果');

% 子图2：陀螺仪零偏估计曲线
subplot(2,1,2);
plot(t, gyro_bias * ones(1,N), 'k--', 'LineWidth', 1.5); hold on;
plot(t, est_bias, 'g-', 'LineWidth', 1.5);
grid on;
legend('真实零偏 (True Bias)', '卡尔曼估计零偏 (Estimated Bias)');
xlabel('时间 (秒)');
ylabel('零偏 (度/秒)');
title('陀螺仪常值零偏 (Bias) 实时估计曲线');
