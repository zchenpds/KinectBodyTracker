%% Initialize variables.
clear;
clc;
[filename1, pathname1] = uigetfile('data*Kinect?.csv');
if isequal(filename1,0)
   disp('Cannot open data file. User selected Cancel');
   return
end
dataKinect = importKinectData([pathname1,filename1]);

[filename2, pathname2] = uigetfile([pathname1,filename1(1:22),'*Vicon.mat']);
if isequal(filename2,0)
   disp('Cannot open data file. User selected Cancel');
   return
end
load([pathname2,filename2]);

[filename2, pathname2] = uigetfile([pathname1,filename1(1:22),'*Insole.mat']);
if isequal(filename2,0)
   disp('Neglecting insole data.');
else
    load([pathname2,filename2]);
end

clear pathname1 filename1 pathname2 filename2 

%%
paramK.t = -0.28;
paramK.x = 0.2;
paramK.y = 0.2;
paramK.zL = 0;
paramK.xR = 0;
paramK.yR = 0;
paramK.zR = 0;

t0 = dataKinect.tK(1);
ts = (dataKinect.tK - t0) / 1000 + paramK.t;
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
%% Get Kinect data
xL_K = dataKinect.WLAnkleX;
yL_K = dataKinect.WLAnkleY;
zL_K = dataKinect.WLAnkleZ;
xR_K = dataKinect.WRAnkleX;
yR_K = dataKinect.WRAnkleY;
zR_K = dataKinect.WRAnkleZ;


%% Get Vicon data
tV = VICON.t;
xL_V = VICON.LHeelCent(:, 1) / 1000;
yL_V = VICON.LHeelCent(:, 2) / 1000;
zL_V = VICON.LHeelCent(:, 3) / 1000;
xR_V = VICON.RHeelCent(:, 1) / 1000;
yR_V = VICON.RHeelCent(:, 2) / 1000;
zR_V = VICON.RHeelCent(:, 3) / 1000;

iV0 = find(abs(xR_V - xR_V(1)) > 0.02, 1);
tV = tV - tV(iV0);

% Register the odometry frame at t=0 with the Vicon frame
T_VW.x = yR_K(1) - xR_V(1) + paramK.y;
T_VW.y = xR_K(1) - yR_V(1) - paramK.x;
T_VW.th = pi/2;
[xL_V, yL_V] = my_transform(xL_V, yL_V, T_VW);
[xR_V, yR_V] = my_transform(xR_V, yR_V, T_VW);



%% Plot left ankle data

for caseNum = 1:2
    if caseNum == 1
        t1 = ts;
        x1 = xL_K;
        y1 = yL_K;
        z1 = zL_K;
        t2 = tV;
        x2 = xL_V;
        y2 = yL_V;
        z2 = zL_V;
    elseif caseNum == 2
        t1 = ts;
        x1 = xR_K;
        y1 = yR_K;
        z1 = zR_K;
        t2 = tV;
        x2 = xR_V;
        y2 = yR_V;
        z2 = zR_V;
    end
    
    figure;
    ax1 = subplot(3, 1, 1);
    h11 = plot(t1, x1, '.-r');
    hold on;
    h12 = plot(t2, x2, '-b');
    if exist('StanceL', 'var')
        plot(t1(iStances{caseNum}(:,1)), x1(iStances{caseNum}(:,1)), 'ro');
        plot(t1(iStances{caseNum}(:,2)), x1(iStances{caseNum}(:,2)), 'rx');
    end
    grid on;
    legend([h11, h12], "Kinect", "Vicon")
    ylabel("X Coordinates (m)")
    xlabel("Time (s)");
    
    if caseNum == 1
        title("Left ankle");
    elseif caseNum == 2
        title("Right ankle");
    end
    

    ax2 = subplot(3, 1, 2);
    h11 = plot(t1, y1, '.-r');
    hold on;
    h12 = plot(t2, y2, '-b');
    if exist('StanceL', 'var')
        plot(t1(iStances{caseNum}(:,1)), y1(iStances{caseNum}(:,1)), 'ro');
        plot(t1(iStances{caseNum}(:,2)), y1(iStances{caseNum}(:,2)), 'rx');
    end
    grid on;
    legend([h11, h12], "Kinect", "Vicon")
    ylabel("Y Coordinates (m)")
    xlabel("Time (s)");


    ax3 = subplot(3, 1, 3);
    h11 = plot(t1, z1, '.-r');
    hold on;
    h12 = plot(t2, z2, '-b');
    grid on;
    legend([h11, h12], "Kinect", "Vicon")
    ylabel("Z Coordinates (m)")
    xlabel("Time (s)");
    ylim([-0.1, 0.5]);

    linkaxes([ax1, ax2, ax3], 'x');

end


return
%%
figure;
h11 = plot3(dataKinect.WLAnkleX, dataKinect.WLAnkleY, dataKinect.WLAnkleZ, 'o');
hold on;
h12 = plot3(dataKinect.WRAnkleX, dataKinect.WRAnkleY, dataKinect.WRAnkleZ, '*');
legend([h11, h12], "Left ankle", "Right ankle")
view(0,90);


