rootdir = 'data20190517p3dx';
scriptSel = 'test6_loss_of_track';
dataSel = input(['Please enter the type of data you want to run the script on\n',...
    'For example, CW\n'],'s');
if isempty(dataSel)
    dataSel = 'CW';
end
if contains('straightline', dataSel, 'IgnoreCase', true)
    strType = '?STR';
    figureSize = [200,100,750,250];
    y_lims = [-0.5, 1.5];
elseif strcmpi(dataSel, 'CCW')
    strType = '?CCW';
    figureSize = [200,100,1150,450];
    y_lims = [-1.8, 1.0];
else
    strType = '?CW';
    figureSize = [200,100,1150,450];
    y_lims = [-2.0, 2.0];
end
dataFiles = dir(fullfile(rootdir, '*', strType));

close all;
for k = 1:length(dataFiles) 
    dataFile = dataFiles(k);
    dataFiles_Kinect = dir(fullfile(dataFile.folder, dataFile.name, '*Kinect*'));
    figure; set(gcf,'position',figureSize)
    is_first_figure = true;
    for n = 1:length(dataFiles_Kinect)
        strKinectPathName = dataFiles_Kinect(n).folder;
        strKinectFileName = dataFiles_Kinect(n).name;
        subplot(1, 2, 1); plot_option = 'human'; run(scriptSel); 
        subplot(1, 2, 2); plot_option = 'heatmap'; run(scriptSel);
        is_first_figure = false;
    end
    if strcmp(plot_option, 'robot')
        legend('NumColumns',5, 'location', 'north');
    elseif strcmp(plot_option, 'human')
        legend('NumColumns', 2, 'location', 'west');
    end
end

    