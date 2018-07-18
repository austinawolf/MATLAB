
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

%% Get Magnitude

Accelerometer = (Accelerometer * 2.0) / 32768.0;
Gyroscope = (Gyroscope * 250.0) / 32768.0 * (pi/180);


Magnitude = zeros(1,length(Accelerometer));
for col_num = 1:length(Accelerometer)
   Magnitude(col_num) = norm(Accelerometer(col_num,:));
end

%% Plot Gyro/Acc

figure('Name', 'Sensor Data');
axis(1) = subplot(4,1,1);
hold on;

plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');

legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(4,1,2);
hold on;

plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
plot(time,Magnitude,'y');

legend('X', 'Y', 'Z','Mag');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;

linkaxes(axis, 'x');

%% Process sensor data through algorithm

AHRS = MadgwickAHRS('SamplePeriod', 1/100, 'Beta', 0.1);
%AHRS = MahonyAHRS('SamplePeriod', 1/100, 'Kp', .1);

quaternion = zeros(length(time), 4);
for t = 1:length(time)
    AHRS.UpdateIMU(Gyroscope(t,:), Accelerometer(t,:));
    quaternion(t, :) = AHRS.Quaternion;
end

%% Plot algorithm output as Euler angles
euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.
axis(3) = subplot(4,1,3);
hold on;
plot(time, euler(:,2), 'r');
plot(time, euler(:,1), 'g');
plot(time, euler(:,3), 'b');
plot(time,zeros(1,length(euler(:,1))),'y');
title('Euler Angles (Matlab)');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

%% Local Euler Angle Plot

axis(4) = subplot(4,1,4);
hold on;
plot(time, euler(:,2), 'r');
plot(time, euler(:,1), 'g');
plot(time, euler(:,3), 'b');
plot(time,zeros(1,length(euler_uc(:,1))),'y');
title('Euler Angles (uC)');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;


%% End of script
