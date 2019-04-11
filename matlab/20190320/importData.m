%% Initialize variables.
importDataFailedAt = 0;
[filename2, pathname2] = uigetfile('data*Robot?.csv');
if isequal(filename2,0)
   disp('Cannot open data file. User selected Cancel');
   importDataFailedAt = 1;
   return
end
dataRobot = importRobotData([pathname2,filename2]);

[filename3, pathname3] = uigetfile([pathname2,filename2(1:22),'*.m']);
if isequal(filename3,0)
   disp('Cannot open data file. User selected Cancel');
   importDataFailedAt = 2;
   return
end
run([pathname3,filename3]);

[filename4, pathname4] = uigetfile([pathname2,filename2(1:22),'*Vicon.csv']);
if isequal(filename4,0)
   disp('Cannot open data file. User selected Cancel');
   importDataFailedAt = 3;
   return
end
dataVicon = importViconData([pathname4,filename4]);
