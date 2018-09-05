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
params = [3.1420, 0.9888, -0.0541];
[xW_LA, yW_LA] = composite_tf(dataKinect, dataRobot, params, 'ankleL');
[xW_RA, yW_RA] = composite_tf(dataKinect, dataRobot, params, 'ankleR');
%[xW_LF, yW_LF] = composite_tf(dataKinect, dataRobot, params, 'footL');
%[xW_RF, yW_RF] = composite_tf(dataKinect, dataRobot, params, 'footR');
ts = (dataKinect.tKW - dataKinect.tKW(1))/1000;

vW_LA = get_speed(ts, xW_LA, yW_LA);
vW_RA = get_speed(ts, xW_RA, yW_RA);

%% Segmentation
lapIndices = segment_lap(dataKinect, dataRobot);


%%
figure('units','normalized', ...
       'outerposition',[0 0 1 1]);
ax1 = subplot(3, 1, 1);
h1 = plot(ts, xW_LA, 'r-.');
hold on
h2 = plot(ts, xW_RA, 'g-.');
h3 = plot(ts(lapIndices(:, 1)), xW_RA(lapIndices(:, 1)), 'ro');
h4 = plot(ts(lapIndices(:, 2)), xW_RA(lapIndices(:, 2)), 'rx');
legend([h1, h2, h3, h4], "^wx_L_A", "^wx_R_A", 'lap Start', 'Lap end')
grid on;
ylabel("X Position (m)")
xlabel("Time (s)");
xlim([ts(1), ts(end)]);


ax2 = subplot(3, 1, 2);
h1 = plot(ts, yW_LA);
hold on
h2 = plot(ts, yW_RA);
legend([h1, h2], "^wy_L_A", "^wy_R_A")
grid on;
ylabel("Y Position (m)")
xlabel("Time (s)");
xlim([ts(1), ts(end)]);

ax3 = subplot(3, 1, 3);
h1 = plot(ts, vW_LA);
hold on
h2 = plot(ts, vW_RA);
legend([h1, h2], "^wv_L_A", "^wv_R_A")
grid on;
ylabel("joint speed (m)")
xlabel("Time (s)");
xlim([ts(1), ts(end)]);

linkaxes([ax1, ax2, ax3], 'x');


%%
numSubplots = 4;
vThresholdStancePhase = 0.5;
%lapIndices = lapIndices(1:9, :);
iRangeLS = find(vW_LA < vThresholdStancePhase);
iRangeRS = find(vW_RA < vThresholdStancePhase);

for lap = 1:size(lapIndices, 1)
    iSubplot = mod(lap - 1, numSubplots) + 1;
    if iSubplot == 1
        figure('units','normalized', ...
            'outerposition',[0 0 1 1], ...
            'Name', ['Lap #', num2str(lap), ...
            '-', num2str(lap + numSubplots - 1)]);
    end
    
    iStart = lapIndices(lap, 1);
    iEnd = lapIndices(lap, 2);
    iLapRange = iStart:iEnd;
    iLapRangeLS = intersect(iRangeLS, iLapRange);
    iLapRangeRS = intersect(iRangeRS, iLapRange);
    subplot(numSubplots, 1, iSubplot);
    h1 = plot(xW_LA(iLapRange), yW_LA(iLapRange), 'r-');
    hold on;
    h2 = plot(xW_RA(iLapRange), yW_RA(iLapRange), 'b-');
    h3 = plot(xW_LA(iLapRangeLS), yW_LA(iLapRangeLS), 'rx');
    h4 = plot(xW_RA(iLapRangeRS), yW_RA(iLapRangeRS), 'bx');
    ylabel("Y (m)");
    xlabel("X (m)");
    legend([h1, h2, h3, h4], ...
        "Left ankle", "Right ankle", ...
        "Left ankle stance phase", "Right ankle stance phase", ...
        'Location', 'Best')
end















