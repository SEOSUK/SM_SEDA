%% SEDA CSV Plotter
close all; clear; clc;

% CSV 파일 경로
csv_path = '/seda_log.csv';

%% 렌더러 설정 (벡터 PDF용)
set(groot,'defaultFigureRenderer','painters');

% CSV 읽기
data = readmatrix(csv_path);

% 컬럼 정의
t          = data(:,1);   % real_time
q_cmd      = data(:,2);
q_meas     = data(:,3);
theta_cmd  = data(:,4);
theta_meas = data(:,5);

%% Plot
figure('Position',[100 100 800 600]);

% ───────────────────────
% subplot 1: q_cmd vs q_meas
% ───────────────────────
subplot(2,1,1);
plot(t, q_cmd, 'LineWidth', 1.6); hold on;
plot(t, q_meas, 'LineWidth', 1.6);
grid on;
title('q command vs q measured');
xlabel('Time [s]');
ylabel('Angle q [rad]');
legend('q\_cmd','q\_meas');

% ───────────────────────
% subplot 2: theta_cmd vs theta_meas
% ───────────────────────
subplot(2,1,2);
plot(t, theta_cmd, 'LineWidth', 1.6); hold on;
plot(t, theta_meas, 'LineWidth', 1.6);
grid on;
title('\theta command vs \theta measured');
xlabel('Time [s]');
ylabel('\theta [rad]');
legend('\theta\_cmd','\theta\_meas');