%% Start of script

close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

% ExampleScript.m
%
% This script demonstrates use of the MadgwickAHRS and MahonyAHRS algorithm
% classes with example data. ExampleData.mat contains calibrated gyroscope,
% accelerometer and magnetometer data logged from an AHRS device (x-IMU)
% while it was sequentially rotated from 0 degrees, to +90 degree and then
% to -90 degrees around the X, Y and Z axis.  The script first plots the
% example sensor data, then processes the data through the algorithm and
% plots the output as Euler angles.
%
% Note that the Euler angle plot shows erratic behaviour in phi and psi
% when theta approaches ±90 degrees. This due to a singularity in the Euler
% angle sequence known as 'Gimbal lock'.  This issue does not exist for a
% quaternion or rotation matrix representation.
%
% Date          Author          Notes
% 28/09/2011    SOH Madgwick    Initial release
% 13/04/2012    SOH Madgwick    deg2rad function no longer used
% 06/11/2012    Seb Madgwick    radian to degrees calculation corrected

%% Import
addpath('C:\Users\austi\OneDrive\Documents\Processing\QuadSerialDisplay\quaternion_library');
addpath('C:\Users\austi\OneDrive\Documents\Processing\QuadSerialDisplay');

a = importdata('output.csv');
time = a.data(:,2);
Accelerometer = a.data(:,17:19);
Gyroscope = a.data(:,14:16);
%Gyroscope(:,2) = zeros(1,length(Gyroscope(:,1)));
quaternion_uc = a.data(:,22:25);
micros = a.data(:,1);
bad_packets = a.data(:,21);
euler_uc = a.data(:,26:28);

figure('Name','Interval Plot');
axis(1) = subplot(2,1,1);
hold on;

previous = 0;

for x = 1:length(micros)
    intervals(x) = micros(x) - previous;
    previous = micros(x);
end
plot(micros,intervals);

hold off;
axis(2) = subplot(2,1,2);
hold on;
plot(micros,bad_packets);
linkaxes(axis, 'x');

%temp = Gyroscope(:,1);
%Gyroscope(:,1) = Gyroscope(:,2);
%Gyroscope(:,2) = temp; 

Accelerometer = (Accelerometer * 2.0) / 32768.0;
Gyroscope = (Gyroscope * 250.0) / 32768.0 * (pi/180);

%% Gyro/Accelerometer Offsets



%% Get Magnitude
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

%% Process Gyro data

x_filtered = Gyroscope(1,1);
y_filtered = Gyroscope(1,2);
z_filtered = Gyroscope(1,3);

x_summed = 0;
y_summed = 0;
z_summed = 0;

gyro_filtered  = zeros(length(Gyroscope),3);
gyro_summed  = zeros(length(Gyroscope),3);

freq = 100;
for col = 1:length(Gyroscope)
    
    gyro_x = Gyroscope(col,1);
    gyro_y = Gyroscope(col,2);
    gyro_z = Gyroscope(col,3);

    x_filtered = x_filtered * 0.7 + gyro_x * 0.3;
    y_filtered = y_filtered * 0.7 + gyro_y * 0.3;
    z_filtered = z_filtered * 0.7 + gyro_z * 0.3;
    filtered = [x_filtered y_filtered z_filtered];
    
    x_summed = x_summed + x_filtered * (1/freq);
    y_summed = y_summed + y_filtered * (1/freq);
    z_summed = z_summed + z_filtered * (1/freq);
    summed = [x_summed y_summed z_summed];
    
    gyro_filtered(col,:) = filtered;
    gyro_summed(col,:) =  summed;
end





%% Process sensor data through algorithm

AHRS = MadgwickAHRS('SamplePeriod', 1/100, 'Beta', 0.1);
%AHRS = MahonyAHRS('SamplePeriod', 1/100, 'Kp', .1);

quaternion = zeros(length(time), 4);
for t = 1:length(time)
    AHRS.UpdateIMU(Gyroscope(t,:), Accelerometer(t,:));
    quaternion(t, :) = AHRS.Quaternion;
end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ±90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

%figure('Name', 'Euler Angles');
axis(3) = subplot(4,1,3);
hold on;
plot(time, euler(:,1), 'r');
plot(time, euler(:,2), 'g');
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

%% 

figure('Name', 'Large Local Euler Plot');
hold on;
plot(time, euler_uc(:,1), 'r');
plot(time, euler_uc(:,2), 'g');
plot(time,zeros(1,length(euler(:,1))),'y');
title('Euler Angles (uC)');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta');
hold off;

%% CALC OFFSETS

Acc_x = mean(Accelerometer(:,1))
Acc_y = mean(Accelerometer(:,2))
Acc_z = mean(Accelerometer(:,3))
Gyro_x = mean(Gyroscope(:,1))
Gyro_y = mean(Gyroscope(:,2))
Gyro_z = mean(Gyroscope(:,3))
Mag = mean(Magnitude)
%% Plot Gyro Processing

figure('Name', 'Gyro Processing');
axis(1) = subplot(3,1,1);
hold on;

plot(time, Gyroscope(:,1)*(180/pi), 'r');
plot(time, Gyroscope(:,2)*(180/pi), 'g');
plot(time, Gyroscope(:,3)*(180/pi), 'b');

legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;

axis(2) = subplot(3,1,2);
hold on;

plot(time, gyro_filtered(:,1)*(180/pi), 'r');
plot(time, gyro_filtered(:,2)*(180/pi), 'g');
plot(time, gyro_filtered(:,3)*(180/pi), 'b');

legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Filtered');
hold off;

axis(2) = subplot(3,1,3);
hold on;

plot(time, gyro_summed(:,1)*(180/pi), 'r');
plot(time, gyro_summed(:,2)*(180/pi), 'g');
plot(time, gyro_summed(:,3)*(180/pi), 'b');
plot(time,zeros(1,length(gyro_summed(:,1))),'y');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Summed');
hold off;


%% Noise Analysis 
figure('Name','Noise Analysis');
axis(1) = subplot(2,1,1);
hold on;
t1 = 1;
t2 = 8000;
plot(1:length(Gyroscope(t1:t2,2)), Gyroscope(t1:t2,2), 'r');
plot(1:length(Gyroscope(t1:t2,1)), Gyroscope(t1:t2,1), 'g');
title('Time Domain');

hold off;

y = fft(Gyroscope(t1:t2,2));                               % Compute DFT of x
m = abs(y);                               % Magnitude
f = (0:length(y)-1)*100/length(y);        % Frequency vector
axis(2) = subplot(2,1,2);
plot(f,m);
title('Magnitude');

%% End of script
