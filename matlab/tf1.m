function [ xR, yR ] = tf1( zK, xK, gamma )
%tf1: SO(2) coordinate transformation.
% Input args:
%   [zK, xK]:   Joint positions in Kinect frame {OZX}_K
%   gamma:      The angle from {OXY}_R to {OZX}_K
% Returns:
%   [xR, xR]:   Joint positions in Robot frame {OXY}_R

xR = zK .* cos(gamma) - xK .* sin(gamma);
yR = zK .* sin(gamma) + xK .* cos(gamma);

end

