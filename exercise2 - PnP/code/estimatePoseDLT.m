function M = estimatePoseDLT(pts2d, p_W_corners, K)
%ESTIMATEPOSEDLT Summary of this function goes here
%   Detailed explanation goes here
% p_W_corners is given weirdly... I think it is x y z in columns and N
% rows...
    if (size(pts2d,2)==2)
        pts2d = pts2d';
    end
    if (size(p_W_corners,2)~=3)
        p_W_corners = p_W_corners';
    end
    num_pts = size(pts2d,2);
    
    norm_coords= K\[pts2d; ones(1, num_pts)];
    norm_coords = reshape(norm_coords(1:2, :), 2*num_pts, 1);
    norm_coords = repmat(norm_coords, 1, 4);
    
    hom_p_W_corners = [p_W_corners, ones(num_pts, 1)];
    
    q_1 = cat(3, hom_p_W_corners, zeros(num_pts, 4));
    q_1 = permute(q_1, [2 3 1]);
    q_1 = reshape(q_1, 4, 2*num_pts);
    q_1 = q_1';
    
    q_2 = circshift(q_1, 1);
    
    q_3 = -1*norm_coords .* (q_1+q_2);
    
    Q = [q_1, q_2, q_3];
    [~,~,V] = svd(Q);
    M = reshape(V(:,12), 4, 3)';
    
    if M(3,4) < 0
        M = -M;
    end
    
    R = M(:,1:3);
    
    [U,~,V] = svd(R);
    R_tilde = U*V';    

    inv_alpha = norm(R_tilde, 'fro')/norm(R, 'fro');    
    
    M = [R_tilde inv_alpha * M(:,4)];
end
