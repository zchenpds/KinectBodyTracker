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

%% Preprocess data
% Subtract the base time from timestamps
t0 = dataRobot.tRW(1);
dataKinect.tKW = (dataKinect.tKW - t0) / 1000;
dataRobot.tRW = (dataRobot.tRW - t0) / 1000;

% Exclude data where following is disabled, if these are not calib data
if ~contains(filename1, 'calib')
    dataRobot = dataRobot(dataRobot.isFollowing == 1, :);
    dataKinect = dataKinect( dataKinect.tKW > dataRobot.tRW(1) ...
        & dataKinect.tKW < dataRobot.tRW(end), :);
end

%% Find transformation
% fun = @(x)nanvar(composite_tf(dataKinect, dataRobot, x(1), x(2), x(3), 'ankleL'));


if 1
    fun = @(x) mean(obj_fun(dataKinect, dataRobot, x));

    x0 = getInitParams(CalibModel);
    options = optimset('PlotFcns',@optimplotfval);
    params = fminsearch(fun,x0, options)

    tau = getTau(CalibModel, params);

else
    % params = [pi, 1.0, 0];
    %params = [3.1315, 0.9831, -0.04];
    params = [pi, 1, 0];
end

%% Interpolation

xRW = interp1(dataRobot.tRW, dataRobot.x, dataKinect.tKW + tau);
yRW = interp1(dataRobot.tRW, dataRobot.y, dataKinect.tKW + tau);
thRW = interp1(dataRobot.tRW, dataRobot.th, dataKinect.tKW + tau);
%dRW = (xRW.^2 + yRW.^2).^0.5;

%% zK, World frame, Left foot and ankle

figure;
subplot(2, 1, 1);
h11 = plot(dataKinect.tKW, dataKinect.ankleLZ, '.-');
hold on;
h12 = plot(dataKinect.tKW, dataKinect.footLZ, '.-');
h13 = plot(dataRobot.tRW, dataRobot.x, '.-');
%h14 = plot(dataKinect.tKW, xRW, '.-');
%h14 = plot(dataKinect.tKW, thRW, '.-');
legend([h11, h12, h13], "^Kz_L_A", "^Kz_L_F", "^Wx_R_0")
grid on;
ylabel("Coordinates (m)")
xlabel("Time (s)");
title("Raw, O_KZ_K, Left foot and ankle")
xlim([dataKinect.tKW(1), dataKinect.tKW(end)]);

subplot(2, 1, 2);
[xW_LA, yW_LA] = composite_tf(dataKinect, dataRobot, params, 'ankleL');
h21 = plot(dataKinect.tKW, xW_LA);
%h21 = plot(dataKinect.tKW, xRo - dataKinect.ankleLZ);
hold on
[xW_LF, yW_LF] = composite_tf(dataKinect, dataRobot, params, 'footL');
h22 = plot(dataKinect.tKW, xW_LF);
%h22 = plot(dataKinect.tKW, xRW - dataKinect.footLZ);
legend([h21, h22], "^Wx_L_A", "^Wx_L_F")
grid on;
ylabel("Coordinates (m)")
xlabel("Time (s)");
title("Transformed, O_KZ_K, Left foot and ankle")
xlim([dataKinect.tKW(1), dataKinect.tKW(end)]);

%% zK, World frame, Right foot and ankle
figure;
subplot(2, 1, 1);
h11 = plot(dataKinect.tKW, dataKinect.ankleRZ, '.-');
hold on;
h12 = plot(dataKinect.tKW, dataKinect.footRZ, '.-');
h13 = plot(dataRobot.tRW, dataRobot.x, '.-');
legend([h11, h12, h13], "^Kz_R_A", "^Kz_R_F", "^Wx_R_o")
grid on;
ylabel("Coordinates (m)")
xlabel("Time (s)");
title("Raw, O_KZ_K, Right foot and ankle")
xlim([dataKinect.tKW(1), dataKinect.tKW(end)]);

subplot(2, 1, 2);
[xW_RA, yW_RA] = composite_tf(dataKinect, dataRobot, params, 'ankleR');
h21 = plot(dataKinect.tKW, xW_RA);
%h21 = plot(dataKinect.tKW, xRW - dataKinect.ankleRZ);
hold on
[xW_RF, yW_RF] = composite_tf(dataKinect, dataRobot, params, 'footR');
h22 = plot(dataKinect.tKW, xW_RF);
%h22 = plot(dataKinect.tKW, xRW - dataKinect.footRZ);
legend([h21, h22], "^Wx_R_A", "^Wx_R_F")
grid on;
ylabel("Coordinates (m)")
xlabel("Time (s)");
title("Transformed, O_KZ_K, Right foot and ankle")
xlim([dataKinect.tKW(1), dataKinect.tKW(end)]);

%% xK, Left foot and ankle
% Raw
figure;
subplot(2, 1, 1);
h11 = plot(dataKinect.tKW, dataKinect.ankleLX, '.-');
hold on;
h12 = plot(dataKinect.tKW, dataKinect.footLX, '.-');
legend([h11, h12], "^Kx_L_A", "^Kx_L_F")
grid on;
ylabel("Coordinates (m)")
xlabel("Time (s)");
title("Raw, O_KX_K, Left")

% Transformed
subplot(2, 1, 2);
h11 = plot(dataKinect.tKW, yW_LA, '.-');
hold on;
h12 = plot(dataKinect.tKW, yW_LF, '.-');
legend([h11, h12], "^Wy_L_A", "^Wy_L_F")
grid on;
ylabel("Coordinates (m)")
xlabel("Time (s)");
title("Transformed, O_KX_K, Left")

%% xK, Right foot and ankle

% Raw
figure;
subplot(2, 1, 1);
h11 = plot(dataKinect.tKW, dataKinect.ankleRX, '.-');
hold on;
h12 = plot(dataKinect.tKW, dataKinect.footRX, '.-');
legend([h11, h12], "^Kx_R_A", "^Kx_R_F")
grid on;
ylabel("Coordinates (m)")
xlabel("Time (s)");
title("Raw, O_KX_K, Right")


% Transformed
subplot(2, 1, 2);
h11 = plot(dataKinect.tKW, yW_RA, '.-');
hold on;
h12 = plot(dataKinect.tKW, yW_RF, '.-');
legend([h11, h12], "^Wy_R_A", "^Wy_R_F")
grid on;
ylabel("Coordinates (m)")
xlabel("Time (s)");
title("Transformed, O_KX_K, Right")

%% yK, Kinect frame
% Left foot and ankle
figure;
subplot(2, 1, 1);
h11 = plot(dataKinect.tKW, dataKinect.ankleLY, '.-');
hold on;
h12 = plot(dataKinect.tKW, dataKinect.footLY, '.-');
legend([h11, h12], "^Ky_L_A", "^Ky_L_F")
grid on;
ylabel("Coordinates (m)")
xlabel("Time (s)");
title("O_KY_K, Left")

% Right foot and ankle
subplot(2, 1, 2);
h11 = plot(dataKinect.tKW, dataKinect.ankleLY, '.-');
hold on;
h12 = plot(dataKinect.tKW, dataKinect.footLY, '.-');
legend([h11, h12], "^Ky_R_A", "^Ky_R_F")
grid on;
ylabel("Coordinates (m)")
xlabel("Time (s)");
title("O_KY_K, Right")



