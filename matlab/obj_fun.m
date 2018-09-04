function [ variances ] = obj_fun( dataKinect, dataRobot, params)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    

    % list = [1, 2, 3, 4];
    list = [1, 2];
    varsX = zeros(length(list),1);
    varsY = zeros(length(list),1);
    for i=list
        [xW, yW] = composite_tf(dataKinect, dataRobot, params, i);
        varsX(i) = nanvar(xW);
        varsY(i) = nanvar(yW);
    end
    variances = [varsX; varsY];
    %variances = varsX(i);
end

