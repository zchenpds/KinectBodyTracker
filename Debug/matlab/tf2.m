function [ xW, yW ] = tf2( xR, yR, xRW, yRW, theta, scale )
%tf1: Coordinate transformation, with SE(2) and scaling.
% Input args:
%   [xR, yR]:   Joint positions in Robot frame {OXY}_R
%   [xRW, yRW]: Robot position in World frame {OXY}_W
%   theta:      Heading of the robot. Namely, the angle 
%               from {OXY}_W to {OXY}_R
%   scale:      The scale coefficient. 
% Returns:
%   [xW, yW]:   Joint positions in W0orld frame {OXY}_W
%   

xW = scale * xRW + xR .* cos(theta) - yR .* sin(theta);
yW = scale * yRW + xR .* sin(theta) + yR .* cos(theta);
end

