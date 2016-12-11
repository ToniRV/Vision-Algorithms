function [best_guess_history, max_num_inliers_history, iterations] = ...
    parabolaRansac(data, max_noise)
% data is 2xN with the data points given column-wise,
% best_guess_history is 3xnum_iterations with the polynome coefficients
%   from polyfit of the BEST GUESS at each iteration columnwise and
% max_num_inliers_history is 1xnum_iterations, with the inlier count of the
%   BEST GUESS at each iteration.
  
% Unomment to have deterministic output, just to debug.
%   rng(2);

    s = 3;
    max_iterations = 100;

    max_num_inliers_history = zeros(1, max_iterations);
    best_guess_history = zeros (3, max_iterations);
    i=1;
    RMS = 0;
    while (i<max_iterations) 
        i = i +1;
        samples = datasample(data, s, 2, 'Replace', false);
        P = polyfit(samples(1,:),samples(2,:),2);
        evaluated_points = polyval(P, data(1,:));
        noise_values = abs(data(2,:)-evaluated_points);
        num_inliers = sum(noise_values(:)<max_noise);
        if num_inliers > max_num_inliers_history(i-1)
            max_num_inliers_history(i) = num_inliers;
            best_guess_history(:, i) = P;
            inliers_indices = noise_values(:)<max_noise;
            iterations = i;
        else
            max_num_inliers_history(i) = max_num_inliers_history(i-1);
            best_guess_history(:, i) = best_guess_history(:, i-1);
        end
    end
    best_guess_history(:, end) = polyfit(data(1, inliers_indices), data(2, inliers_indices) , 2);
    
end

