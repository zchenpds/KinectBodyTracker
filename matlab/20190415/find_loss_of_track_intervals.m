function [t_start, t_end] = find_loss_of_track_intervals(t,threshold)
%FIND_LOSS_OF_TRACK_TW
if nargin == 1
    threshold = 0.2;
end
dt = diff(t);
t_start = t(dt > threshold);
t_end = t([false; dt > threshold]);
end

