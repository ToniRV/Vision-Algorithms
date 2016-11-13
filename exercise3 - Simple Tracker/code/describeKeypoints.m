function descriptors = describeKeypoints(img, keypoints, r)
% Returns a (2r+1)^2xN matrix of image patch vectors based on image
% img and a 2xN matrix containing the keypoint coordinates.
% r is the patch "radius".
    rows = keypoints(1,:)+r;
    cols = keypoints (2,:)+r;
    N_features = length(rows);
    patch_size = 2*r+1;
    descriptors = repmat(uint8(zeros(patch_size)), 1, 1, N_features);
    tmp_img = padarray(img, [r, r]);
    for i=1:N_features
        descriptors(:,:,i) = tmp_img(rows(i)-r:rows(i)+r, cols(i)-r:cols(i)+r);
    end
    descriptors = reshape(descriptors, patch_size^2, N_features);
end
