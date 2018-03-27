%% Step.3 Static plot
close all
figure('Name', 'Static Hist')
subplot(3,2,1)
histfit(imu_static(7,:),20)
axis tight;title('accl\_x');
subplot(3,2,3)
histfit(imu_static(8,:),20)
axis tight;title('accl\_y');
subplot(3,2,5)
histfit(imu_static(9,:),20)
axis tight;title('accl\_z');
subplot(3,2,2)
histfit(imu_static(10,:),20)
axis tight;title('gyro\_x');
subplot(3,2,4)
histfit(imu_static(11,:),20)
axis tight;title('gyro\_y');
subplot(3,2,6)
histfit(imu_static(12,:),20)
axis tight;title('gyro\_z');

figure('Name', 'Static Accl')
subplot(2,2,1)
scatter3(imu_static(7,:),imu_static(8,:),imu_static(9,:),6,'filled')
axis tight;title('accl\_xyz');
subplot(2,2,2)
scatter(imu_static(7,:),imu_static(8,:),6,'filled')
axis tight;title('accl\_xy');
subplot(2,2,3)
scatter(imu_static(8,:),imu_static(9,:),6,'filled')
axis tight;title('accl\_yz');
subplot(2,2,4)
scatter(imu_static(7,:),imu_static(9,:),6,'filled')
axis tight;title('accl\_xz');

figure('Name','Static Gyro')
subplot(2,2,1)
scatter3(imu_static(10,:),imu_static(11,:),imu_static(12,:),6,'filled')
axis tight;title('gyro\_xyz');
subplot(2,2,2)
scatter(imu_static(10,:),imu_static(11,:),6,'filled')
axis tight;title('gyro\_xy')
subplot(2,2,3)
scatter(imu_static(11,:),imu_static(12,:),6,'filled')
axis tight;title('gyro\_yz');
subplot(2,2,4)
scatter(imu_static(10,:),imu_static(12,:),6,'filled')
axis tight;title('gyro\_xz');

figure('Name','Static Magn')
subplot(2,2,1)
scatter3(imu_static(4,:),imu_static(5,:),imu_static(6,:),6,'filled')
axis tight;title('magn\_xyz');
subplot(2,2,2)
scatter(imu_static(4,:),imu_static(5,:),6,'filled')
axis tight;title('magn\_xy');
subplot(2,2,3)
scatter(imu_static(5,:),imu_static(6,:),6,'filled')
axis tight;title('magn\_yz');
subplot(2,2,4)
scatter(imu_static(4,:),imu_static(6,:),6,'filled')
axis tight;title('magn\_xz');

%% Step.4 Driving Plot
% close all
figure('Name', 'Trajectory')
plot(calib_gps(1,:),calib_gps(2,:),'.')
hold on
plot(x_dist,y_dist,'.');
hold off
axis tight;title('Driving Trajectory: GPS');
%% Mag Calibration
figure('Name', 'Mag Calibration')
subplot(2,2,1)
plot(imu(4,:),imu(5,:),'.') 
grid on; axis tight equal; title('Raw Mag From IMU')
subplot(2,2,2)
plot(raw_mag(1,:),raw_mag(2,:),'.')
grid on; axis tight equal; title('Before Calib')
subplot(2,2,3)
plot(mag_calib(1,calib_idx(1,1):calib_idx(1,2)),mag_calib(2,calib_idx(1,1):calib_idx(1,2)),'.')
grid minor; axis tight equal; title('After Calib')
subplot(2,2,4)
plot(mag_calib(1,:),mag_calib(2,:),'.')
grid on; axis tight equal;title('Full Calibrated Mag')
%% Yaws
close all;
figure('Name','Yaw Values')
prng = [1,size_imu];
plot(timestamp(prng(1):prng(2)),yaw_mag_but(prng(1):prng(2)))
hold on
plot(timestamp(prng(1):prng(2)),yaw_gyro(prng(1):prng(2)))
plot(timestamp(prng(1):prng(2)),yaw_imu_calib(prng(1):prng(2)))
plot(timestamp(prng(1):prng(2)),yaw_fusion(prng(1):prng(2)))
% plot(timestamp,gyro_z_but)
legend('Mag Calculated Yaw',...
    'Gryo Yaw with butterworth',...
    'Raw Yaw from IMU',...
    'Complimentary filtered Yaw')
% plot(timestamp(prng(1):prng(2)),yaw_mag(prng(1):prng(2)))
axis tight;
hold off
%% integrate traj
% close all
% figure('Name','Y_acc comparation')
subplot(3,2,1)
plot(timestamp, Y_acc1)
title('Assume xc = 0');xlabel('calculated Y acceleration')
subplot(3,2,3)
plot(timestamp, imu(8,:))
xlabel('IMU Y acceleration')
subplot(3,2,5)
plot(timestamp, Y_acc1)
hold on
plot(timestamp, imu(8,:))
hold off

subplot(3,2,2)
plot(timestamp, Y_acc2)
title('Assume xc = -0.45');xlabel('calculated Y acceleration')
subplot(3,2,4) 
plot(timestamp, imu(8,:))
xlabel('IMU Y acceleration')
subplot(3,2,6)
plot(timestamp, Y_acc2)
hold on
plot(timestamp, imu(8,:))
hold off
%% test
figure()









