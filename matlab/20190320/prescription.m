%%
% The imported m file should contain a 

[filename, pathname] = uigetfile('data*desiredPath.m');
if isequal(filename,0)
   disp('Cannot open data file. User selected Cancel');
   return
end
run([pathname,filename]);

dxdy = diff(desired_path(:,1:2));
ds = (dxdy(:, 1).^2 + dxdy(:, 2).^2).^0.5;
s = [0; cumsum(ds)];
%%
close all;
subplot(2,2,1)
%plot(desired_path(1:140,1), desired_path(1:140, 2), '.')
plot(desired_path(:,1), desired_path(:, 2), '.')

xlabel('x (m)');
ylabel('y (m)');

%%
subplot(2,2,2)
plot(1:length(s), s);
xlabel('Array Index');
ylabel('Arc Length (m)');


%%
subplot(2,2,3)
plot(s, desired_path(:,3));
yticks([-2*pi -pi 0 pi 2*pi])
yticklabels({'-2\pi','-\pi','0','\pi','2\pi'})

xlabel('s (m)');
ylabel('orientation (rad)');

%%
subplot(2,2,4)
plot(s, desired_path(:,4));

xlabel('s (m)');
ylabel('Max V (m/s)');
