function [i_start, i_end] = get_footstep_cluster_indices(x, y, t)

    % The distance threshold the points within which are considered
    % inliners of a footstep (foot-flat phase).
    dist_threshold = 0.05; 
    % The maximum number of outliers allowed to appear in one footstep
    max_num_outliers = 3;
    % The minimum time difference between the startings of two foot-flat
    % phases
    min_peak_sep = 0.8;
    % The maximum duration that one foot-flat phase can last
    t_threashold = 1;
    % The minimum number of points required for one foot-flat phase
    min_num_points = 10;
    
    len = length(x);
    i_start_cand = (1:len)';
    i_end_cand = i_start_cand + 5;

    for k = 1:len
        num_outliers = 0;
        while i_end_cand(k) < len && i_end_cand(k) - i_start_cand(k) < 30 && ...
                num_outliers < max_num_outliers && ...
                t(i_end_cand(k)) - t(i_start_cand(k)) < t_threashold
            dist = norm( [x(i_start_cand(k)) - x(i_end_cand(k)), ...
                y(i_start_cand(k)) - y(i_end_cand(k))] );
            i_end_cand(k) = i_end_cand(k) + 1;
            if dist > dist_threshold
                num_outliers = num_outliers + 1;
            end
        end
        i_end_cand(k) = i_end_cand(k) - 1;
    end
    sep_cand = i_end_cand - i_start_cand;
    % Debug: show the peaks found
    figure; findpeaks(sep_cand, t, 'MinPeakHeight', min_num_points, 'MinPeakDistance', min_peak_sep);
    
    [~, locs_t] = findpeaks(sep_cand, t, 'MinPeakHeight', min_num_points, 'MinPeakDistance', min_peak_sep);
    [~, locs] = ismember(locs_t, t);
    i_start = i_start_cand(locs);
    i_end = i_end_cand(locs);
    
    % Get rid of the footsteps when the subject was actually standing still
    while length(i_start) >= 2 && ....
            norm([x(i_start(1)) - x(i_start(2)), y(i_start(1)) - y(i_start(2))]) < dist_threshold
        i_start = i_start(2:end);
        i_end = i_end(2:end);
    end
end
