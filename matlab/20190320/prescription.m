[filename, pathname] = uigetfile('data*desiredPath.m');
if isequal(filename,0)
   disp('Cannot open data file. User selected Cancel');
   return
end
run([pathname,filename]);
%%
close all;
%plot(desired_path(1:140,1), desired_path(1:140, 2), '.')
plot(desired_path(:,1), desired_path(:, 2), '.')
%plot(1:153, atan2(diff(desired_path(:, 2)), diff(desired_path(:, 1)))); hold on; plot(1:153, desired_path(2:end, 3))