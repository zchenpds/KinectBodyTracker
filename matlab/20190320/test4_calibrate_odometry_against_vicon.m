close all;
clear;
run('importData');
if importDataSuccess == false
    return;
end
%% Vicon data preprocessing
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

% Verify trimming
figure; 
plot(1:length(xR_V), xR_V);
title('See if trimming is successful');
xlabel('Index');
ylabel('x (m)');


%% Robot odometry data
xR_O = dataRobot.x;
yR_O = dataRobot.y;
tR = dataRobot.tR - dataRobot.tR(1);

v = dataRobot.v;
w = dataRobot.wD;
th = dataRobot.th;


%% Find the timestamps of Vicon data
fV = 300;
iO = find(xR_O > 1.5, 1);
iVKey = find(xR_V > xR_O(iO), 1);
tVKey = tR(iO) + (xR_V(iVKey) - xR_O(iO)) / v(iO);
tV = ((1:length(xR_V))' - iVKey) ./ fV + tVKey;


%% integrate speed to get x and y

th_Int = cumsum(w * 0.1 * 1.338 * 0.98);
xR_Int = cumsum(v * 0.1 .* cos(th_Int));
yR_Int = cumsum(v * 0.1 .* sin(th_Int));

figure;
plot(xR_V, yR_V);
hold on
%plot(xR_O, yR_O);
plot(xR_Int, yR_Int);











