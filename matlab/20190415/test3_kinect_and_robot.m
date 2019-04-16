%% Initialize variables.
clear;
clc;
[filename1, pathname1] = uigetfile('data*Kinect?.csv');
if isequal(filename1,0)
   disp('Cannot open data file. User selected Cancel');
   return
end
dataKinect = importKinectData([pathname1,filename1]);

[filename2, pathname2] = uigetfile([pathname1,filename1(1:22),'*Robot?.csv']);
if isequal(filename2,0)
   disp('Cannot open data file. User selected Cancel');
   return
end
dataRobot = importRobotData([pathname2,filename2]);

%%
t0 = dataKinect.tK(1);
ts = (dataKinect.tK - t0) / 1000;

%%
close all;
figure;
ax1 = subplot(3, 1, 1);
h11 = plot(ts, dataKinect.WLAnkleX, '.-');
hold on;
h12 = plot(ts, dataKinect.WRAnkleX, '.-');
grid on;
legend([h11, h12], "WLAnkleX", "WRAnkleX")
ylabel("Coordinates (m)")
xlabel("Time (s)");

ax2 = subplot(3, 1, 2);
h11 = plot(ts, dataKinect.WLAnkleY, '.-');
hold on;
h12 = plot(ts, dataKinect.WRAnkleY, '.-');
grid on;
legend([h11, h12], "WLAnkleY", "WRAnkleY")
ylabel("Coordinates (m)")
xlabel("Time (s)");


ax3 = subplot(3, 1, 3);
h11 = plot(ts, dataKinect.WLAnkleZ, '.-');
hold on;
h12 = plot(ts, dataKinect.WRAnkleZ, '.-');
grid on;
legend([h11, h12], "WLAnkleZ", "WRAnkleZ")
ylabel("Coordinates (m)")
xlabel("Time (s)");

linkaxes([ax1, ax2, ax3], 'x');
%%
figure;
h11 = plot3(dataKinect.WLAnkleX, dataKinect.WLAnkleY, dataKinect.WLAnkleZ, 'o');
hold on;
h12 = plot3(dataKinect.WRAnkleX, dataKinect.WRAnkleY, dataKinect.WRAnkleZ, '*');
legend([h11, h12], "Left ankle", "Right ankle")
view(0,90);


