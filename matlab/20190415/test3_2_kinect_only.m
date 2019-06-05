%% Initialize variables.
clear;
clc;
[filename1, pathname1] = uigetfile('data*Kinect?.csv');
if isequal(filename1,0)
   disp('Cannot open data file. User selected Cancel');
   return
end
dataKinect = importKinectData([pathname1,filename1]);

[filename2, pathname2] = uigetfile([pathname1,filename1(1:22),'*Insole.mat']);
if isequal(filename2,0)
   disp('Neglecting insole data.');
else
    load([pathname2,filename2]);
end

clear pathname1 filename1 pathname2 filename2 

%%
t0 = dataKinect.tK(1);
ts = (dataKinect.tK - t0) / 1000;

%% Find heel strike and toe off indices
if exist('StanceL', 'var')
    Stances = {StanceL, StanceR};
    iStances = {nan(size(StanceL, 1), 2), nan(size(StanceR, 1), 2)};
    for n = 1:length(Stances) % loop over {L, R}
        Stance = Stances{n};
        for i = 1:size(Stance, 1) % loop over rows
            if isnan(Stance(i, 3))
                continue;
            end
            iStances{n}(i, 1) = find(Stance(i, 3) == Kinect.t);
            iStances{n}(i, 2) = find(Stance(i, 4) == Kinect.t);
        end
        iNan = find( isnan(iStances{n}(:, 1)) );
        iStances{n}(iNan, :) = [];
    end
    clear Stances Stance
end
%% Ankles
xL = dataKinect.WLAnkleX;
yL = dataKinect.WLAnkleY;
zL = dataKinect.WLAnkleZ;
xR = dataKinect.WRAnkleX;
yR = dataKinect.WRAnkleY;
zR = dataKinect.WRAnkleZ;
close all;
figure;
ax1 = subplot(3, 1, 1);
h11 = plot(ts, xL, '.-r');
hold on;
h12 = plot(ts, xR, '.-b');
if exist('StanceL', 'var')
    plot(ts(iStances{1}(:,1)), xL(iStances{1}(:,1)), 'ro');
    plot(ts(iStances{1}(:,2)), xL(iStances{1}(:,2)), 'rx');
    plot(ts(iStances{2}(:,1)), xR(iStances{2}(:,1)), 'bo');
    plot(ts(iStances{2}(:,2)), xR(iStances{2}(:,2)), 'bx');
end
grid on;
legend([h11, h12], "Left", "Right")
ylabel("X Coordinates (m)")
xlabel("Time (s)");
title(sprintf("Ankle, %.2f %%", length(ts)/30/(ts(end) - ts(1)) * 100));

ax2 = subplot(3, 1, 2);
h11 = plot(ts, yL, '.-r');
hold on;
h12 = plot(ts, yR, '.-b');
if exist('StanceL', 'var')
    plot(ts(iStances{1}(:,1)), yL(iStances{1}(:,1)), 'ro');
    plot(ts(iStances{1}(:,2)), yL(iStances{1}(:,2)), 'rx');
    plot(ts(iStances{2}(:,1)), yR(iStances{2}(:,1)), 'bo');
    plot(ts(iStances{2}(:,2)), yR(iStances{2}(:,2)), 'bx');
end
grid on;
legend([h11, h12], "Left", "Right")
ylabel("Y Coordinates (m)")
xlabel("Time (s)");


ax3 = subplot(3, 1, 3);
h11 = plot(ts, zL, '.-');
hold on;
h12 = plot(ts, zR, '.-');
grid on;
legend([h11, h12], "Left", "Right")
ylabel("Z Coordinates (m)")
xlabel("Time (s)");

linkaxes([ax1, ax2, ax3], 'x');
return
%% Feet
xL = dataKinect.WLFootX;
yL = dataKinect.WLFootY;
zL = dataKinect.WLFootZ;
xR = dataKinect.WRFootX;
yR = dataKinect.WRFootY;
zR = dataKinect.WRFootZ;
figure;
ax1 = subplot(3, 1, 1);
h11 = plot(ts, xL, '.-r');
hold on;
h12 = plot(ts, xR, '.-b');
if exist('StanceL', 'var')
    plot(ts(iStances{1}(:,1)), xL(iStances{1}(:,1)), 'ro');
    plot(ts(iStances{1}(:,2)), xL(iStances{1}(:,2)), 'rx');
    plot(ts(iStances{2}(:,1)), xR(iStances{2}(:,1)), 'bo');
    plot(ts(iStances{2}(:,2)), xR(iStances{2}(:,2)), 'bx');
end
grid on;
legend([h11, h12], "Left", "Right")
ylabel("X Coordinates (m)")
xlabel("Time (s)");
title("Foot");

ax2 = subplot(3, 1, 2);
h11 = plot(ts, yL, '.-r');
hold on;
h12 = plot(ts, yR, '.-b');
if exist('StanceL', 'var')
    plot(ts(iStances{1}(:,1)), yL(iStances{1}(:,1)), 'ro');
    plot(ts(iStances{1}(:,2)), yL(iStances{1}(:,2)), 'rx');
    plot(ts(iStances{2}(:,1)), yR(iStances{2}(:,1)), 'bo');
    plot(ts(iStances{2}(:,2)), yR(iStances{2}(:,2)), 'bx');
end
grid on;
legend([h11, h12], "Left", "Right")
ylabel("Y Coordinates (m)")
xlabel("Time (s)");


ax3 = subplot(3, 1, 3);
h11 = plot(ts, zL, '.-');
hold on;
h12 = plot(ts, zR, '.-');
grid on;
legend([h11, h12], "Left", "Right")
ylabel("Z Coordinates (m)")
xlabel("Time (s)");

linkaxes([ax1, ax2, ax3], 'x');
return
%%
figure;
h11 = plot3(dataKinect.WLAnkleX, dataKinect.WLAnkleY, dataKinect.WLAnkleZ, 'o');
hold on;
h12 = plot3(dataKinect.WRAnkleX, dataKinect.WRAnkleY, dataKinect.WRAnkleZ, '*');
legend([h11, h12], "Left ankle", "Right ankle")
view(0,90);


