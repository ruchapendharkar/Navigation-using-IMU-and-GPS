data = readmatrix("imu.csv"); %Import the CSV file for imu from the driving in circles data
mx = data(:,27);
my = data(:,28);

magx = mx(1200:end-1000);
magy = my(1200:end-1000);

%Plotting the magnetometer data before calibration

fig1 = figure(1);
scatter(magx, magy);
hold on;
ellipse = fit_ellipse(magx, magy,1);
xlabel("MagX (Gauss)");
ylabel("MagY (Gauss)");
title("Magnetometer before calibration");
grid on;

%Hard Iron Calibration

magx_translated = magx - ellipse.X0_in;
magy_translated = magy - ellipse.Y0_in;

%Soft Iron Calibration

mag_data = [magx_translated magy_translated];
Rotation_matrix = [cos(ellipse.phi), -sin(ellipse.phi); sin(ellipse.phi), cos(ellipse.phi)];
magnetometer_data = mag_data';

rotated_magnetometer = mtimes(Rotation_matrix,magnetometer_data);

mag_calibrated = [rotated_magnetometer(1,:)/ ellipse.long_axis; rotated_magnetometer(2,:) / ellipse.short_axis];

%Plotting calibrated magnetometer data

fig2= figure(2);
scatter(mag_calibrated(1,:), mag_calibrated(2,:));
hold on;
scatter(magx, magy);
hold on;
ellipse1 = fit_ellipse(mag_calibrated(1,:), mag_calibrated(2,:),1);
hold on;
ellipse2 = fit_ellipse(magx, magy,1);
title("Magnetometer before and after Calibration");
xlabel("MagX (Gauss)");
ylabel("MagY (Gauss)");
legend('Calibrated','Uncalibrated');
axis equal; grid on;

% Plotting time series of magnetometer graph

fig3 = figure(3); 
plot(mx); 
hold on;
plot(mag_calibrated(1,:))
xlabel("Time"); 
ylabel("MagX (Gauss)");
legend('Uncalibrated','Calibrated')
title("Time series of MagX before and after calibration"); grid on;

fig4 = figure(4); 
plot(my); 
hold on;
plot(mag_calibrated(2,:))
xlabel("Time"); 
ylabel("MagY (Gauss)");
legend('Unalibrated','Calibrated')
title("Time series of MagY before and after calibration"); grid on;
%%
%Analysing the Driving Data

%Yaw from the IMU

drivingimu_data = readmatrix("driving_imu.csv");
drivinggps_data = readmatrix('driving_gps.csv');
qx = drivingimu_data(:,10);
qy = drivingimu_data(:,11);
qz = drivingimu_data(:,12);
qw = drivingimu_data(:,13);

orientation = quat2eul([qx qy qz qw]);
yaw = rad2deg(orientation(:,3));
correction_imu_yaw = yaw(1,1);
yaw_from_imu = rad2deg(unwrap(deg2rad(yaw)));
%%
%Yaw from Magnetometer without calibration

time_imu = drivingimu_data(:,3);
time_imu_nsecs = drivingimu_data(:,4);
driving_magx = drivingimu_data(:,27);
driving_magy = drivingimu_data(:,28);

driving_magnetometer = [driving_magx,driving_magy];

yaw_from_magnetometer = rad2deg(atan2(-driving_magy,driving_magx));

%Yaw from Magnetometer after calibration
mx_driving = driving_magx - ellipse.X0_in;
my_driving = driving_magy - ellipse.Y0_in;

magnetometer_driving = [mx_driving my_driving];

driving_magnetometer_calibrated = mtimes(Rotation_matrix,magnetometer_driving');

mx_imu = driving_magnetometer_calibrated(1,:)/ellipse.long_axis;
my_imu = driving_magnetometer_calibrated(2,:)/ellipse.short_axis;

corrected_yaw_from_magnetometer = rad2deg(unwrap(atan2(-my_imu,mx_imu)));

fig7 = figure(7);
plot(time_imu, yaw_from_magnetometer, LineWidth=1);
hold on; 
plot(time_imu, (corrected_yaw_from_magnetometer), LineWidth=1);
xlabel("Time (s)");
ylabel(" Yaw(degrees)");
legend('Raw Magnetometer Yaw','Corrected Yaw');
title("Comparison of Yaw");
grid on;
%%
% Yaw from Gyroscope 

gyro_x = drivingimu_data(:,15);
gyro_y = drivingimu_data(:,16);
gyro_z = drivingimu_data(:,17);

yaw_from_gyro = rad2deg((cumtrapz(gyro_z)*(1/40)) + deg2rad(correction_imu_yaw));

fig8 = figure(8);
plot(time_imu,(yaw_from_gyro));
hold on;
plot(time_imu,(corrected_yaw_from_magnetometer-270));
xlabel("Time (s)");
ylabel(" Yaw(degrees)");
legend('Yaw from Gyroscope','Yaw from Magnetometer');
title("Comparison of Yaw from Gyroscope and Magnetometer");
grid on;
%%
%Sensor fusion of gyroscope estimate and magnetometer estimate 

Fs = 40;
alpha = 0.95;

%Magnetometer data is filtered using a low pass filter
d_mag = designfilt('lowpassfir', 'FilterOrder', 5 , 'CutoffFrequency', 0.85, 'SampleRate', Fs);
low_pass_mag = filter(d_mag, corrected_yaw_from_magnetometer);
low_pass_mag_t = low_pass_mag';

%Gyroscope data is filtered using a high pass filter 

d_gyro = designfilt('highpassfir', 'FilterOrder', 6 , 'CutoffFrequency', 0.05, 'SampleRate', Fs);
high_pass_gyro = filter(d_gyro, yaw_from_gyro);

yaw_fused = (alpha*high_pass_gyro(:,1))+ ((1-alpha)*low_pass_mag_t(:,1));
heading = rad2deg(wrapToPi(deg2rad(yaw_fused)));

fig9 = figure(9);
plot(time_imu,heading);
hold on;
plot(time_imu,yaw);
xlabel('Time(s)')
ylabel('Yaw(deg)')
legend('Yaw from Fusion','Yaw from IMU')
title('Comparison of Yaw from Sensor Fusion and IMU Yaw')
grid on;

fig10 = figure(10);
plot(time_imu,high_pass_gyro);
hold on;
plot(time_imu,low_pass_mag);
hold on;
plot(time_imu,yaw_fused);
xlabel('Time(s)')
ylabel('Yaw(deg)')
legend('HPF','LPF','CF');
title('Yaw from High Pass , Low Pass and Complementary Filter')
grid on;
%Part 1 ends here 

%%

% Part 2 starts here 
utm_e = drivinggps_data(:,9);
utm_n = drivinggps_data(:,10);
time_gps = drivinggps_data(:,3);

utm_easting = utm_e - min(utm_e);
utm_northing = utm_n - min(utm_n);
% Velocity estimation from GPS

velocity_e = utm_easting;
velocity_n = utm_northing;

gps_velocity = sqrt(diff(velocity_e).^2 + diff(velocity_n).^2);

gps_velocity_corrected = filloutliers(gps_velocity,"nearest"); % to replace outlier points
gps_vel = interp(gps_velocity_corrected(:,1),40); % Upsampling the GPS Velocity 

%%

x_acc = drivingimu_data(:,19);
y_acc = drivingimu_data(:,20);
z_acc = drivingimu_data(:,21);

r_acc = sqrt(x_acc.^2 + y_acc.^2 + z_acc.^2);

time_start = time_imu(1);
time = [];
for k=1:length(x_acc)
    time(k) = time_imu(k) - time_start;
end
velocity_biased = cumsum(x_acc);
fig7 = figure(7);
plot(time(1:length(gps_vel)),gps_vel);
hold on;
plot(time,velocity_biased)
xlabel('Time(s)');
ylabel('Velocity (m/s)')
legend('GPS Velocity','IMU Velocity');
title('GPS Velocity and IMU Velocity before correction')
grid on

%Error Detection

count = 0;
time_1 = [];
time_2 = [];
j = 1;

x_acc = x_acc -mean(x_acc(1:1000));

for i = 1:1:length(r_acc)
    if r_acc(i) < 9.8
        count = count + 1;
        if count <= 250
            time_1(j) = i - 250;
       
        end
   
        if count > 250 && (r_acc(i + 1) > 9.8)
            time_2(j) = i;
            j = j + 1;
        end

    else
        count = 0;
       
    end
end

for k = 1:1:length(time_1) - 1
        bias = mean(x_acc(time_1(k):time_2(k)));
        x_acc(time_1(k):time_2(k)) = 0;
        x_acc(time_2(k):time_1(k+1)) = x_acc(time_2(k): time_1(k+1)) - bias;
   
end
% Velocity from Acceleration

imu_velocity = integrate(x_acc);

fig11 = figure(11);
plot(time(1:length(gps_vel)),gps_vel);
hold on;
plot(time,imu_velocity)
xlabel('Time(s)');
ylabel('Velocity (m/s)')
legend('GPS Velocity','IMU Velocity');
title('GPS Velocity and Linear Velocity after correction')
grid on
% Part 2 ends here
%%

%Part 3 starts here

% Displacement from Acceleration
imu_displacement = integrate(imu_velocity);

gps_displacement = integrate(gps_vel);

ve = cos(deg2rad(yaw_fused)).*imu_velocity';
vn = sin(deg2rad(yaw_fused)).*imu_velocity';

xe = cumsum(ve) * 1/40;
xn = cumsum(vn) * 1/40;

y = gyro_z.*imu_velocity';
fig13 = figure(13);
plot(smooth(y_acc));
hold on;
plot(y);
legend('Obs','Calc')
title('Comparison of Observed Acceleration and Coriolis Acceleration')

offset = utm_easting(1)-xe(1);
scaling_factor = 1.3;

fig12= figure(12);
plot(-scaling_factor*(xe-offset),scaling_factor*xn);
hold on;
plot(utm_easting,utm_northing);
xlabel('UTM-Easting (m)');
ylabel('UTM-Northing (m)');
legend('IMU','GPS')
title('Plot of Route')
grid on;

function integrated_data = integrate(data)
    x = 0;
    integrated_data= [];
    for k=1:length(data)
        tmp = x + data(k) * (1/40);
        if tmp < 0
            x = 0;
            tmp = x + data(k) * (1/40);
        end
        integrated_data(k) = tmp;
        x = tmp;
    end
end