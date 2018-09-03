%% Initialize variables.
close all

% Configuration
global CalibModel
CalibModel = CalibModels.Symmetrical;


[filename1, path1] = uigetfile('*data*Kinect?.csv');
if isequal(filename1,0)
   disp('Cannot open data file. User selected Cancel');
   return
end
file1 = fullfile(path1, filename1);
dataKinect = importKinectData3(file1);

filename2 = strrep(filename1, 'Kinect', 'Robot');
file2 = fullfile(path1, filename2);
dataRobot = importRobotData2(file2);


%%
params = [3.1520, 0.9888, -0.0541];
[xW_LA, yW_LA] = composite_tf(dataKinect, dataRobot, params, 'ankleL');
[xW_RA, yW_RA] = composite_tf(dataKinect, dataRobot, params, 'ankleR');
ts = (dataKinect.tKW - dataKinect.tKW(1))/1000;

figure;
subplot(2, 1, 1);
h1 = plot(ts, xW_LA);
%h21 = plot(dataKinect.tKW, xRo - dataKinect.ankleLZ);
hold on
h2 = plot(ts, xW_RA);
legend([h1, h2], "^wx_L_A", "^wx_R_A")
grid on;
ylabel("X Coordinates (m)")
xlabel("Time (s)");
xlim([ts(1), ts(end)]);


subplot(2, 1, 2);
h1 = plot(dataKinect.tKW, yW_LA);
%h21 = plot(dataKinect.tKW, xRo - dataKinect.ankleLZ);
hold on
h2 = plot(ts, yW_RA);
legend([h1, h2], "^wy_L_A", "^wy_R_A")
grid on;
ylabel("Y Coordinates (m)")
xlabel("Time (s)");
xlim([ts(1), ts(end)]);
















