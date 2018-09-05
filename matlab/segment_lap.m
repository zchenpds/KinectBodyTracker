function [ lapIndices ] = segment_lap( dataKinect, dataRobot )
    %UNTITLED2 Summary of this function goes here
    %   Detailed explanation goes here
    vThreshold = 0.1;
    maxLapNum = 50;
    lapIndices = nan(maxLapNum, 2);
    vRobot = dataRobot.v;
    xRobot = dataRobot.x;
    lapDurationMax = 15;
    lapDurationMin = 5;
    %lapGapMax = 100;
    lapGapMin = 10;
    tRW = (dataRobot.tRW - dataRobot.tRW(1))/1000;
    tKW = (dataKinect.tKW - dataRobot.tRW(1))/1000;
    iVOverThreshold = find(vRobot > vThreshold);

    lap = 1;
    iLapStartRange = 1:length(tRW);
    while lap < maxLapNum

        ind = intersect(iLapStartRange, iVOverThreshold);
        if isempty(ind)
           break;
        else
            iLapStart = ind(1);
        end

        %iLapStart = find(vRobot(iLapStartRange) > vThreshold, 1);
        tLapStart = tRW(iLapStart);
        iLapEndRange = find(tRW > tLapStart & tRW < tLapStart + lapDurationMax);
        ind = intersect(iLapEndRange, iVOverThreshold);
        if isempty(ind)
           iLapStartRange = find(tRW > iLapStart + lapDurationMax);
           continue;
        else
            iLapEnd = ind(end);
            tLapEnd = tRW(iLapEnd);
            if tLapEnd - tLapStart < lapDurationMin
                iLapStartRange = find(tRW > tLapEnd);
                continue;
            elseif mean(vRobot(iLapStart:iLapEnd)) < 0.4
                iLapStartRange = find(tRW > tLapStart + 0.5);
                continue;
            end
            
        end

        iLapStartRange = find(tRW > tLapEnd + lapGapMin);

        if isempty(iLapStartRange)
            break;
        end
        iKLapStart = find(tKW > tLapStart, 1);
        iKLapEnd = find(tKW < tLapEnd, 1, 'last');
        if ~isempty(iKLapStart) && ~isempty(iKLapEnd)
            lapIndices(lap, 1) = iKLapStart;
            lapIndices(lap, 2) = iKLapEnd;
            lap = lap + 1;
        end
    end
    
    lapIndices(any(isnan(lapIndices), 2), :) = [];

end

