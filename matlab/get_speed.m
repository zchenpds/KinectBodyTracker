function [ v ] = get_speed( ts, x, y )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

vx = diff(x)./diff(ts);
vy = diff(y)./diff(ts);
v = (vx.^2 + vy.^2).^0.5;
v = [0; v];
%v = movmean(v,2);

% fc = 2;
% fs = 30;
% [b,a] = butter(4,fc/(fs/2));
% v = filter(b,a,v);

% iOutliers = find(v > 5);
% 
% for i=iOutliers
%     v(i) = v(i - 1);
% end