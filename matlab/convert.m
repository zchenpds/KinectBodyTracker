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
iStrSubj = strfind(filename1, '-S'); 
iSubj = str2num(filename1((iStrSubj+2):(iStrSubj+3)));
if any(iSubj == [1, 2])
    params = [3.1454    1.0000   -0.0462];
elseif any(iSubj == [3, 4, 5])
    params = [3.1640    1.0052   -0.0541];
elseif any(iSubj == [6, 7, 8, 9, 10])
    params = [3.1489    1.0092   -0.0475];
else
    error(['No params have been found for subject ', num2str(iSubj)]);
end
[xW_LA, yW_LA] = composite_tf(dataKinect, dataRobot, params, 'ankleL');
[xW_RA, yW_RA] = composite_tf(dataKinect, dataRobot, params, 'ankleR');
[xW_LF, yW_LF] = composite_tf(dataKinect, dataRobot, params, 'footL');
[xW_RF, yW_RF] = composite_tf(dataKinect, dataRobot, params, 'footR');
ts = (dataKinect.tKW - dataKinect.tKW(1))/1000;
zW_LA = dataKinect.ankleLY;
zW_RA = dataKinect.ankleRY;
%hist(zW_LA, 50);

vW_LA = get_speed(ts, xW_LA, yW_LA);
vW_RA = get_speed(ts, xW_RA, yW_RA);

fc = 2;
fs = 30;
[b,a] = butter(1,fc/(fs/2));
vW_LA_f = filter(b,a,vW_LA);
vW_RA_f = filter(b,a,vW_RA);
% %% Segmentation
% lapIndices = segment_lap(dataKinect, dataRobot);
% stepIndicesCell = cluster_step( xW_LA, yW_LA, vW_LA,...
%     xW_RA, yW_RA, vW_RA, lapIndices );
% 
% 
% %% Save
% save(['mat_filtered/kinect_S', num2str(iSubj, '%02d')], ...
%     'xW_LA', 'yW_LA', 'vW_LA', ...
%     'xW_RA', 'yW_RA', 'vW_RA', ...
%     'xW_LF', 'yW_LF', 'xW_RF', 'yW_RF', ...
%     'dataKinect', 'dataRobot', 'stepIndicesCell');
% return;

%%
allLapRange = [];
for i = 1:size(lapIndices, 1)
    allLapRange = [allLapRange;
        (lapIndices(i, 1):lapIndices(i, 2))'];
end

figure('units','normalized', ...
       'outerposition',[0 0 1 1]);
ax1 = subplot(4, 1, 1);
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


ax2 = subplot(4, 1, 2);
h1 = plot(ts, yW_LA);
hold on
h2 = plot(ts, yW_RA);
legend([h1, h2], "^wy_L_A", "^wy_R_A")
grid on;
ylabel("Y Position (m)")
xlabel("Time (s)");
xlim([ts(1), ts(end)]);

% ax3 = subplot(4, 1, 3);
% h1 = plot(ts, zW_LA);
% hold on
% h2 = plot(ts, zW_RA);
% legend([h1, h2], "^wz_L_A", "^wz_R_A")
% grid on;
% ylabel("Z Position (m)")
% xlabel("Time (s)");
% xlim([ts(1), ts(end)]);

ax3 = subplot(4, 1, 3);
h1 = plot(ts(allLapRange), vW_LA(allLapRange));
hold on
h2 = plot(ts(allLapRange), vW_LA_f(allLapRange));
legend([h1, h2], "^wv_L_A", "^wv_L_A_f");
grid on;
ylabel("joint speed (m)")
xlabel("Time (s)");
xlim([ts(1), ts(end)]);


ax4 = subplot(4, 1, 4);
h1 = plot(ts(allLapRange), vW_RA(allLapRange));
hold on
h2 = plot(ts(allLapRange), vW_RA_f(allLapRange));
legend([h1, h2], "^wv_R_A", "^wv_R_A_f");
grid on;
ylabel("joint speed (m)")
xlabel("Time (s)");
xlim([ts(1), ts(end)]);

linkaxes([ax1, ax2, ax3, ax4], 'x');

return;
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















