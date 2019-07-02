%% Initialize variables.
if ~exist('plot_option', 'var')
    plot_option = 'heatmap';
end
% Open Kinect File
if exist('strKinectPathName', 'var')
    pathname1 = [strKinectPathName, '\\'];
    filename1 = strKinectFileName;
    fileandpath1 = fullfile(strKinectPathName, strKinectFileName);
else
    [filename1, pathname1] = uigetfile('data20190517p3dx\data*Kinect?.csv');
    if isequal(filename1,0)
       disp('Cannot open data file. User selected Cancel');
       return
    end
    fileandpath1 = [pathname1,filename1];
end

try
    dataKinect = importKinectData(fileandpath1);
catch
    fileID = fopen(fileandpath1,'r');
    dataArray = textscan(fileID, '%s','delimiter','\r\n');
    fclose(fileID);
    dataKinect = importKinectData(fileandpath1, 2, size(dataArray{1}, 1) -1);
end

% Open Robot File
fileandpath2 = [pathname1,filename1(1:23),'Robot0.csv'];
try
    dataRobot = importRobotData(fileandpath2);
catch
    fileID = fopen(fileandpath2,'r');
    dataArray = textscan(fileID, '%s','delimiter','\r\n');
    fclose(fileID);
    dataRobot = importRobotData(fileandpath2, 2, size(dataArray{1}, 1) -1);
end

clear filename1 pathname1 fileandpath1 fileandpath2

%%
t0 = dataKinect.tKW(1);
ts = (dataKinect.tKW - t0) / 1000;

[t_loss_start, t_loss_end] = find_loss_of_track_intervals(dataKinect.tKW, 300);

%% Plot robot trajectory
if strcmp(plot_option, 'robot')
    xR = dataRobot.x;
    yR = dataRobot.y;

    if exist('is_first_figure', 'var') && is_first_figure == true
        plot(xR, yR, 'DisplayName', 'path');hold on; axis equal;
        title(['Loss of Track of Subject ' dataFile.folder((end-2):end)]);
        xlabel('x(m)'); ylabel('y(m)');
        %xlim([0, 5]); 
        ylim(y_lims);
    end

    if ~exist('is_first_figure', 'var')
        figure;
        plot(xR, yR, 'DisplayName', 'path');hold on; axis equal;
    end

    for k = 1:length(t_loss_start)
        i = find(dataRobot.tRW > t_loss_start(k), 1);
        duration_of_loss = t_loss_end(k) - t_loss_start(k);
        plot(xR(i), yR(i), 'o', 'MarkerSize', duration_of_loss / 100, ...
            'DisplayName', [num2str(duration_of_loss), ' ms']);
    end
end


%% plot the position of the human relative to the robot
xH = (dataKinect.KLAnkleZ + dataKinect.KRAnkleZ) / 2;
yH = (dataKinect.KLAnkleX + dataKinect.KRAnkleX) / 2;
if strcmp(plot_option, 'human')
    x_lim = 3;
    if ~exist('is_first_figure', 'var')
        figure;
        is_first_figure = true;
        %plot(xH, yH, '.', 'DisplayName', 'position');hold on; axis equal;
    end

    for k = 1:length(t_loss_start)
        i = find(dataKinect.tKW >= t_loss_start(k), 1);
        duration_of_loss = t_loss_end(k) - t_loss_start(k);
        plot(xH(i), yH(i), 'o', 'MarkerSize', duration_of_loss / 100, ...
            'DisplayName', [num2str(duration_of_loss), ' ms']);
        if exist('is_first_figure', 'var') && is_first_figure == true
            %fill([0, x_lim, x_lim], [0, x_lim * tan(1.22), x_lim * tan(-1.22)], 'yellow');
            if exist('dataFile', 'var')
                title(['Loss of Track of Subject ' dataFile.folder((end-2):end)]);
            end
            xlabel('x(m)'); ylabel('y(m)');
            xlim([0, 3]);  hold on; axis equal;
            is_first_figure = false;
        end
    end
    
end

if strcmp(plot_option, 'heatmap')
    hist3([xH, yH], 'edgecolor', 'interp', 'CDataMode', 'auto');colorbar;view(2)
    %plot(xH, yH, '.', 'DisplayName', 'position');hold on; axis equal;
end