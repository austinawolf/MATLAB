
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% Import
addpath('C:\Users\austi\OneDrive\Documents\Processing\QuadSerialDisplay\quaternion_library');
addpath('C:\Users\austi\OneDrive\Documents\Processing\QuadSerialDisplay');

%micros,timer_counts
%CH1,CH2,CH3,CH4,CH5,CH6
%MOTOR1,MOTOR2,MOTOR3,MOTOR4 (10 - 13)
%GYRO_X,GYRO_Y,GYRO_Z,ACC_X,ACC_Y,ACC_Z (14 - 19)
%bad_packets,serial_recieve (20 - 21)
%q0,q1,q2,q3,roll,pitch,yaw,pid_output_roll,pid_output_pitch (22 - 30)
%pid_output_yaw,roll_target,pitch_target (31 - 33)
%roll_error,pitch_error,pid_p_roll_prop,pid_p_pitch_prop (34 - 37)
%pid_i_roll_sum,pid_i_pitch_sum,pid_p_gain,pid_d_gain (38 - 41)

data = importdata('output.csv');
micros = data(:,1);
time = data(:,2);
n = 1:length(time);
channels = data(:,4:9);
motors = data(:,10:13);
Gyroscope = data(:,14:16);
Accelerometer = data(:,17:19);
bad_packets = data(:,21);
quaternion_uc = data(:,22:25);
euler_uc = data(:,26:28);
pid_output = data(:,29:30);
pid_target = data(:,32:33);
pid_error = data(:,34:35);
pid_p_gain = data(:,40);

t1 = 1;
t2 = length(n);

%% Conversions

Accelerometer = (Accelerometer * 2.0) / 32768.0;
Gyroscope = (Gyroscope * 250.0) / 32768.0 * (pi/180);


%% Noise Analysis 
figure('Name','Noise Analysis');
axis(1) = subplot(2,1,1);
hold on;

plot(1:length(Gyroscope(t1:t2,2)), Gyroscope(t1:t2,2), 'r');
plot(1:length(Gyroscope(t1:t2,1)), Gyroscope(t1:t2,1), 'g');
title('Time Domain');

hold off;

y = fft(Gyroscope(t1:t2,2));              % Compute DFT of x
m = abs(y);                               % Magnitude
f = (0:length(y)-1)*100/length(y);        % Frequency vector
axis(2) = subplot(2,1,2);
plot(f,m);
title('Magnitude');

%% End of script
