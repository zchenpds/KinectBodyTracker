function [ stepIndicesCell ] = cluster_step( ...
    xW_LA, yW_LA, vW_LA, ...
    xW_RA, yW_RA, vW_RA, lapIndices )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


numSubplots = 4;
vThresholdStancePhase = 0.3;
%lapIndices = lapIndices(1:4, :);
if 1
    % Use speed to determine stance phase
    iRangeLS = find(vW_LA < vThresholdStancePhase);
    iRangeRS = find(vW_RA < vThresholdStancePhase);
end

stepIndicesCell = cell(size(lapIndices, 1), 2);
for lap = 1:size(lapIndices, 1)
    % plot initialization
    iSubplot = mod(lap - 1, numSubplots) + 1;
    if iSubplot == 1
        figure('units','normalized', ...
            'outerposition',[0 0 1 1], ...
            'Name', ['Lap #', num2str(lap), ...
            '-', num2str(lap + numSubplots - 1)]);
    end
    
    iStart = lapIndices(lap, 1);
    iEnd = lapIndices(lap, 2);
    iLapRange = iStart:iEnd;
    iLapRangeLS = intersect(iRangeLS, iLapRange);
    iLapRangeRS = intersect(iRangeRS, iLapRange);
        
    % plot
    subplot(numSubplots, 1, iSubplot);
    h1 = plot(xW_LA(iLapRange), yW_LA(iLapRange), 'r-');
    hold on;
    h2 = plot(xW_RA(iLapRange), yW_RA(iLapRange), 'b-');
    h3 = plot(xW_LA(iLapRangeLS), yW_LA(iLapRangeLS), 'rx');
    h4 = plot(xW_RA(iLapRangeRS), yW_RA(iLapRangeRS), 'bx');
    ylabel("Y (m)");
    xlabel("X (m)");
%     legend([h1, h2, h3, h4], ...
%         "Left ankle", "Right ankle", ...
%         "Left ankle stance phase", "Right ankle stance phase", ...
%         'Location', 'Best')
%     
    stepIndicesCell{lap, 1} = ...
        cluster_step_per_lap(xW_LA, yW_LA, iLapRangeLS);
    stepIndicesCell{lap, 2} = ...
        cluster_step_per_lap(xW_RA, yW_RA, iLapRangeRS);
end

end









