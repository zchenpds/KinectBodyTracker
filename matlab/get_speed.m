function [ v ] = get_speed( ts, x, y )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

vx = diff(x)./diff(ts);
vy = diff(y)./diff(ts);
v = (vx.^2 + vy.^2).^0.5;
v = [0; v];
v = movmean(v,2);

% iOutliers = find(v > 5);
% 
% for i=iOutliers
%     v(i) = v(i - 1);
% end