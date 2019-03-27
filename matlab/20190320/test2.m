%% Initialize variables.

filename2 = uigetfile('data*Robot?.csv');
if isequal(filename2,0)
   disp('Cannot open data file. User selected Cancel');
   return
end
dataRobot = importRobotData(filename2);

%%
close all;
figure;
plot(dataRobot.xVm, dataRobot.yVm)
hold on
run('desiredPath.m')
plot(desired_path(:,1), desired_path(:, 2), 'o');
axis equal
xlabel('x(m)')
ylabel('y(m)')
legend({'Actual path', 'Desired path'});
%%
figure;
subplot(2, 1, 1)
t = (dataRobot.tRW - dataRobot.tRW(1))/1e3;
plot(t, dataRobot.v)
hold on
plot(t, dataRobot.vD)
xlabel('t(s)')
ylabel('v(m/s)')
legend({'Actual', 'Desired'});

subplot(2, 1, 2)
plot(t, dataRobot.w)
hold on
plot(t, dataRobot.wD)
xlabel('t(s)')
ylabel('w(rad/s)')
legend({'Actual', 'Desired'});