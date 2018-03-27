%% Step.1 reading data from log files
clearvars;
% add the lcm.jar file to the matlabpath - need to only do this once
javaaddpath /usr/local/share/java/lcm.jar
javaaddpath /home/jeoker/Documents/EECE-5698/lab3/lcmtypes%/imu_msg.jar
%javaaddpath /home/jeoker/Documents/EECE-5698/lab3/lcmtypes/gps_msg.jar
% open log file for reading
static_file = lcm.logging.Log('/home/jeoker/Documents/EECE-5698/lab3/log/imu_static', 'r');
loop_file = lcm.logging.Log('/home/jeoker/Documents/EECE-5698/lab3/log/best_log', 'r');
size_stat_imu = 0;
size_imu = 0;
size_gps = 0;

%read static data first
while true
    try
        static_ev = static_file.readNext();
   
        size_stat_imu = size_stat_imu + 1;
        msg_stat_imu = imu_struct.imu_msg(static_ev.data);
        
        imu_static(1, size_stat_imu)  = msg_stat_imu.yaw;
        imu_static(2, size_stat_imu)  = msg_stat_imu.pitch;
        imu_static(3, size_stat_imu)  = msg_stat_imu.roll;
        imu_static(4, size_stat_imu)  = msg_stat_imu.magn_x;
        imu_static(5, size_stat_imu)  = msg_stat_imu.magn_y;
        imu_static(6, size_stat_imu)  = msg_stat_imu.magn_z;
        imu_static(7, size_stat_imu)  = msg_stat_imu.accl_x;
        imu_static(8, size_stat_imu)  = msg_stat_imu.accl_y;
        imu_static(9, size_stat_imu)  = msg_stat_imu.accl_z;
        imu_static(10, size_stat_imu) = msg_stat_imu.gyro_x;
        imu_static(11, size_stat_imu) = msg_stat_imu.gyro_y;
        imu_static(12, size_stat_imu) = msg_stat_imu.gyro_z;
    catch
        break;
    end
end

% then read the driving data (both gps & imu)
while true
    try
        loop_ev = loop_file.readNext();
        
        if strcmp(loop_ev.channel, 'imu_talker')
            size_imu = size_imu + 1;
            time_imu(size_imu) = loop_ev.utime;
            msg_imu = imu_struct.imu_msg(loop_ev.data);
            
            imu(1, size_imu)  = msg_imu.yaw;
            imu(2, size_imu)  = msg_imu.pitch;
            imu(3, size_imu)  = msg_imu.roll;
            imu(4, size_imu)  = msg_imu.magn_x;
            imu(5, size_imu)  = msg_imu.magn_y;
            imu(6, size_imu)  = msg_imu.magn_z;
            imu(7, size_imu)  = msg_imu.accl_x;
            imu(8, size_imu)  = msg_imu.accl_y;
            imu(9, size_imu)  = msg_imu.accl_z;
            imu(10, size_imu) = msg_imu.gyro_x;
            imu(11, size_imu) = msg_imu.gyro_y;
            imu(12, size_imu) = msg_imu.gyro_z;
        elseif strcmp(loop_ev.channel, 'gps_talker')
            size_gps = size_gps + 1;
            msg_gps = gps_struct.gpgga_msg(loop_ev.data);
            
            gps(1, size_gps) = msg_gps.utc_time;
            gps(2, size_gps) = msg_gps.lon;
            gps(3, size_gps) = msg_gps.lat;
            gps(4, size_gps) = msg_gps.alt;
            gps(5, size_gps) = msg_gps.utm_x;
            gps(6, size_gps) = msg_gps.utm_y;
        end
    catch
        break;
    end
end

% clear all irrelavant data
clear static_file static_ev msg_stat_imu
clear loop_file   loop_ev   msg_imu msg_gps

%% Step.2 read data from mat & static calibration
clearvars
load('log_data.mat');
% set right timestamp
timestamp = (time_imu - time_imu(1))./(1e+6);
% calibration index
calib_idx = [2400 7000;50 174];
stat_idx = [1 1560];
% get static imu drift
min_stat_imu = min(imu(:,stat_idx(1):stat_idx(2)),[],2);
max_stat_imu = max(imu(:,stat_idx(1):stat_idx(2)),[],2);
mean_stat_imu = mean(imu(:,stat_idx(1):stat_idx(2)),2);
% mean_stat_imu(10:12,:) = mean(imu_static(10:12,:),2);
% static data calibration
imu(4:6,:) = imu(4:6,:) - mean_stat_imu(4:6);           
imu(10:12,:) = imu(10:12,:) - mean_stat_imu(10:12);

%% Step.3 data processing

% gps calibration
offset_utm = mean(gps(5:6,calib_idx(2,1):calib_idx(2,2)),2);
calib_gps = gps(5:6,:) - offset_utm;

% mag calibration
raw_ypr = imu(1:3,calib_idx(1,1):calib_idx(1,2));
raw_mag = imu(4:6,calib_idx(1,1):calib_idx(1,2));
mag_calibing(1,:) = raw_mag(1,:).*cosd(raw_ypr(2,:))+raw_mag(2,:).*sind(raw_ypr(2,:)).*sind(raw_ypr(3,:))...
    -raw_mag(3,:).*sind(raw_ypr(2,:)).*cosd(raw_ypr(3,:));
mag_calibing(2,:) = raw_mag(2,:).*cosd(raw_ypr(3,:))+raw_mag(3,:).*sind(raw_ypr(3,:));

param = fit_ellipse(mag_calibing(1,:)', mag_calibing(2,:)');
phi = [cos(param.phi) -sin(param.phi);sin(param.phi) cos(param.phi)];
mag_calib = phi * (imu(4:5,:) - [param.X0_in-0.005 param.Y0_in-0.005]');

sf(1) = max(1, param.short_axis/param.long_axis);
sf(2) = max(1, param.long_axis/param.short_axis);
mag_calib(1,:) = mag_calib(1,:) .* sf(1);
mag_calib(2,:) = mag_calib(2,:) .* sf(2);

% get 3 different yaws
% yaw initial offset
dtheta = 105;
yaw_imu_calib = imu(1,:) - imu(1,1) + dtheta; %from IMU (wrapped, deg)
yaw_mag = atan2d(-mag_calib(2,:),mag_calib(1,:)); % from mag (wrapped, deg)
yaw_mag = unwrap(yaw_mag - yaw_mag(1)); %             <--do rad2deg, unwrap
gyro_z = rad2deg(imu(12,:)); %                      <--do rad2deg
[ahigh, bhigh] = butter(1, 0.000002, 'high');
gyro_z_but = filter(ahigh,bhigh,gyro_z);
yaw_gyro = cumtrapz(timestamp, gyro_z_but); % from raw gyro_z (unwrapped, deg)
% do butterworth filter
[alow, blow]   = butter(1,0.005, 'low');
yaw_mag_but  = filter(alow,blow,yaw_mag);
% get yaw from complementary filter
alpha = 1.1;
yaw_fusion = (1.25-alpha)*yaw_gyro + alpha*yaw_mag_but; %   (unwrapped, rad)
yaw_imu_calib = unwrap(yaw_imu_calib); % (unwrapped, deg)

%% estimate xc
% naive velocity
[a, b] = butter(1, 0.01);
x_acc_but = filter(a, b, imu(7,:)-mean(imu(7,1:1600)));
y_acc = filter(a,b,imu(8,:));
x_vel = cumtrapz(timestamp, x_acc_but);
Y_acc1 = deg2rad(yaw_fusion.*x_vel);

xc = 0.35;
gyro_z_filtered = filter(a,b,imu(12,:)); % zeros(1,size_imu);
X_acc =  x_acc_but + gyro_z_filtered.*gyro_z_filtered*xc;
X_vel = cumtrapz(timestamp, X_acc);
gyro_acc_z = zeros(1,size_imu);
gyro_acc_z(2:end) = diff(gyro_z_filtered) ./ diff(timestamp);
Y_acc2 = gyro_z_filtered.*X_vel + gyro_acc_z.*xc;

mean_xacc = mean(imu(7,1:1600));
% X_acc = imu(7,:)-mean_xacc;
for i = 2:size_gps
    if (sqrt((calib_gps(1,i)-calib_gps(1,i-1))^2+(calib_gps(2,i)-calib_gps(2,i-1))^2) < 0.00001)
        X_acc((i-1)*40:end) = X_acc((i-1)*40:end) - X_acc((i-1)*40);
    end
end
% for i = 120:size_gps
%     if (sqrt((calib_gps(1,i)-calib_gps(1,i-1))^2+(calib_gps(2,i)-calib_gps(2,i-1))^2) < 0.1)
%         X_acc((i-1)*40:end) = X_acc((i-1)*40:end) - X_acc((i-1)*40);
%     end
% end
X_vel = cumtrapz(timestamp, X_acc)*0.18;
for i = 40:size_gps
    if (sqrt((calib_gps(1,i)-calib_gps(1,i-1))^2+(calib_gps(2,i)-calib_gps(2,i-1))^2) < 0.000001)
        X_vel((i-1)*40:end) = X_vel((i-1)*40:end) - X_vel((i-1)*40);
    end
end
tot_dist = cumtrapz(timestamp, X_vel);
tot_diff = zeros(1,size_imu);
x_dist = zeros(1,size_imu);
y_dist = zeros(1,size_imu);
tot_diff(2:end) = diff(tot_dist);
for i = 2:size_imu
    x_dist(i) = x_dist(i-1) + cosd(yaw_imu_calib(i)) * tot_diff(i);
    y_dist(i) = y_dist(i-1) + sind(yaw_imu_calib(i)) * tot_diff(i);
end
x_dist = -x_dist;







