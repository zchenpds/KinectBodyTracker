function [ stepIndices ] = cluster_step_per_lap( x, y , iLapRangeS)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

% cluster
MaxNumSteps = 15;
if isempty(iLapRangeS)
    stepIndices = [];
    return;
end
Z = linkage([x(iLapRangeS), y(iLapRangeS)]);
T = cluster(Z, 'Maxclust',10);
scatter(x(iLapRangeS), y(iLapRangeS), 10, T);

iClusters = unique(T);
stepIndices = nan(MaxNumSteps, 2);

step = 1;
for i = iClusters'
    
    % Only i's that are populous enough in T can qualify as steps
    if (length(x(T==i)) < 4)
        % Too few points in this cluster
        continue;
    end
    
    stepIndices(step, 1) = iLapRangeS(find(T==i, 1));
    stepIndices(step, 2) = iLapRangeS(find(T==i, 1, 'last'));
    step = step + 1;
end

stepIndices(any(isnan(stepIndices), 2), :) = [];
stepIndices = sortrows(stepIndices);



end

