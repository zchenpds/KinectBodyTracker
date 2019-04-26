%%
close all;
clear;
[filename2, pathname2] = uigetfile('data*Robot?.csv');
if isequal(filename2,0)
   disp('Cannot open data file. User selected Cancel');
   return
else
    dataRobot = importRobotData([pathname2,filename2]);
end


[filename3, pathname3] = uigetfile([pathname2,filename2(1:22),'*desiredPath.m']);
if isequal(filename3,0)
   disp('Cannot open desiredPath file. User selected Cancel');
else
    run([pathname3,filename3]);
end


[filename4, pathname4] = uigetfile([pathname2,filename2(1:22),'*Vicon.csv']);
if isequal(filename4,0)
   disp('Cannot open Vicon file. User selected Cancel');
else
    dataVicon = importViconData([pathname4,filename4]);
end

%% Draw trajectory as recorded by odometry
close all;
figure;

if exist('desired_path', 'var')
    plot(dataRobot.xVm, dataRobot.yVm)
    hold on
    plot(desired_path(:,1), desired_path(:, 2), 'o');
    legend({'Actual path', 'Desired path'});
else
    plot(dataRobot.x, dataRobot.y)
end
axis equal
xlabel('x(m)')
ylabel('y(m)')
title('Trajectory as recorded by odometry');
%% Plot x, y, and th vs time
figure;
ax1 = subplot(3, 1, 1);
t = (dataRobot.tRW - dataRobot.tRW(1))/1e3;
plot(t, dataRobot.x)
xlabel('t(s)')
ylabel('x(m)')
title('x position');

ax2 = subplot(3, 1, 2);
plot(t, dataRobot.y)
xlabel('t(s)')
ylabel('y(m)')
title('y position');

ax3 = subplot(3, 1, 3);
plot(t, dataRobot.th)
xlabel('t(s)')
ylabel('th(rad)')
title('orientation');

linkaxes([ax1, ax2, ax3], 'x');

%% Plot speed vs time
figure;
ax1 = subplot(2, 1, 1);
t = (dataRobot.tRW - dataRobot.tRW(1))/1e3;
plot(t, dataRobot.v)
hold on
plot(t, dataRobot.vD)
xlabel('t(s)')
ylabel('v(m/s)')
legend({'Actual', 'Desired'});
title('Linear speed');

ax2 = subplot(2, 1, 2);
plot(t, dataRobot.w)
hold on
plot(t, dataRobot.wD)
xlabel('t(s)')
ylabel('w(rad/s)')
legend({'Actual', 'Desired'});
title('Angular speed');

linkaxes([ax1, ax2], 'x');
%% Vicon
if ~exist('dataVicon', 'var')
    return;
end

xR_V = (dataVicon.X1 + dataVicon.X2 + dataVicon.X3 + dataVicon.X4 + dataVicon.X5) / 5 / 1000;
yR_V = (dataVicon.Y1 + dataVicon.Y2 + dataVicon.Y3 + dataVicon.Y4 + dataVicon.Y5) / 5 / 1000;

% Register the odometry frame at t=0 with the Vicon frame
TO_V.x = -xR_V(1);
TO_V.y = -yR_V(1);
TO_V.th = pi/2;
[xR_V, yR_V] = my_transform(xR_V, yR_V, TO_V);

% Trim the trajectory prior to the start
i0 = find(xR_V > 0.005, 1, 'first');
xR_V = xR_V(i0:end);
yR_V = yR_V(i0:end);
figure; range = 1:1500; plot(range, xR_V(range));

% Start drawing
figure;
plot(xR_V, yR_V);
hold on
plot(desired_path(:,1), desired_path(:, 2), 'o');
axis equal
xlabel('x(m)')
ylabel('y(m)')
legend({'Actual path', 'Desired path'});
title('Trajectory recorded by Vicon');