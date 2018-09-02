function [ xW, yW ] = composite_tf(dataKinect, dataRobot, params, cmd )
    if strcmp(cmd, 'ankleL') || (isnumeric(cmd) && cmd == 1)
        zK = dataKinect.ankleLZ;
        xK = dataKinect.ankleLX;
    elseif strcmp(cmd, 'ankleR') || (isnumeric(cmd) && cmd == 2)
        zK = dataKinect.ankleRZ;
        xK = dataKinect.ankleRX;
    elseif strcmp(cmd, 'footL') || (isnumeric(cmd) && cmd == 3)
        zK = dataKinect.footLZ;
        xK = dataKinect.footLX;
    elseif strcmp(cmd, 'footR') || (isnumeric(cmd) && cmd == 4)
        zK = dataKinect.footRZ;
        xK = dataKinect.footRX;
    end

    % params
    global CalibModel
    if CalibModel == CalibModels.Symmetrical
        gamma = params(1);
        scale = params(2);
        tau = params(3);
    else
        error('Asymmetrical model has not been implemented yet.');
        gamma = params(1);
        scale1 = params(2);
        scale2 = params(3);
        tau = params(4);
    end

    xRW = interp1(dataRobot.tRW, dataRobot.x, dataKinect.tKW + tau);
    yRW = interp1(dataRobot.tRW, dataRobot.y, dataKinect.tKW + tau);
    thRW = interp1(dataRobot.tRW, dataRobot.th, dataKinect.tKW + tau);
    thRW0 = zeros(size(thRW));
    [xR, yR] = tf1(zK, xK, gamma);
    [xW, yW] = tf2(xR, yR, xRW, yRW, thRW, scale);

end

