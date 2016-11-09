function M_dlt = estimatePoseDLT(pts2d, p_W_corners, K)
%ESTIMATEPOSEDLT Summary of this function goes here
%   Detailed explanation goes here
    if (size(pts2d,2)==2)
        pts2d = pts2d';
    end
    num_pts = size(pts2d,2);
    num_corners = size(p_W_corners,1);
    
    norm_coords= K\[pts2d; ones(1, num_pts)];
    norm_coords = reshape(norm_coords(1:2, :), 2*num_pts, 1);
    norm_coords = repmat(norm_coords, 1, 4);
    
    hom_p_W_corners = [p_W_corners, ones(num_corners, 1)];
    
    q_3 = -1*norm_coords .* repmat(hom_p_W_corners, 2, 1);
    q_1 = cat(3, hom_p_W_corners, zeros(num_corners, 4));
    q_1 = permute(q_1, [2 3 1]);
    q_1 = reshape(q_1, 4, 2*num_corners);
    q_1 = q_1';
    q_2 = circshift(q_1, 1);
    
    Q = [q_1, q_2, q_3];
    [U,S,V] = svd(Q);
    
end
