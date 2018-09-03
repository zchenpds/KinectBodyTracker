%% Initialize variables.
filename1 = uigetfile('data*Kinect?.csv');
if isequal(filename1,0)
   disp('Cannot open data file. User selected Cancel');
   return
end
dataKinect = importKinectData(filename1);

filename2 = uigetfile('data*Robot?.csv');
if isequal(filename2,0)
   disp('Cannot open data file. User selected Cancel');
   return
end
dataRobot = importRobotData(filename2);

%% Subtract the base time from timestamps
t0 = dataRobot.tRW(1);
dataKinect.tKW = (dataKinect.tKW - t0) / 1000;
dataRobot.tRW = (dataRobot.tRW - t0) / 1000;

%% Exclude data where following is disabled, if these are not calib data
if ~contains(filename1, 'calib')
    dataRobot = dataRobot(dataRobot.isFollowing == 1, :);
    dataKinect = dataKinect( dataKinect.tKW > dataRobot.tRW(1) ...
        & dataKinect.tKW < dataRobot.tRW(end), :);
end

%%

xRo = interp1(dataRobot.tRW, dataRobot.x, dataKinect.tKW);
yRo = interp1(dataRobot.tRW, dataRobot.y, dataKinect.tKW);
dRo = (xRo.^2 + yRo.^2).^0.5;


%% z, Left foot and ankle
figure;
subplot(2, 1, 1);
h11 = plot(dataKinect.tKW, dataKinect.ankleLZ, '.-');
hold on;
h12 = plot(dataKinect.tKW, dataKinect.ankleRZ, '.-');
h13 = plot(dataKinect.tKW, dataKinect.footLZ, '.-');
h14 = plot(dataKinect.tKW, dataKinect.footRZ, '.-');
grid on;
legend([h11, h12, h13, h14], "ankleLZ", "ankleLZ", "footLZ", "footRZ")
ylabel("Coordinates (m)")
xlabel("Time (s)");

subplot(2, 1, 2);
h21 = plot(dataKinect.tKW, xRo, '.-');
hold on
h22 = plot(dataRobot.tRW, dataRobot.x, 'x-');
legend([h21, h22], "Interpolated x_R_o", "Raw x_R_o")
grid on;
ylabel("Coordinates (m)")
xlabel("Time (s)");




