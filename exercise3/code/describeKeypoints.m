function descriptors = describeKeypoints(img, keypoints, r)
% Returns a (2r+1)^2xN matrix of image patch vectors based on image
% img and a 2xN matrix containing the keypoint coordinates.
% r is the patch "radius".
tic
    rows = keypoints(1,:)+r;
    cols = keypoints (2,:)+r;
    patch_size = (2*r+1) ^ 2;
    descriptors = uint8(zeros(patch_size, length(rows)));
    tmp_img = padarray(img, [r, r]);
    for i=1:length(rows)
        descriptors(:,i) = reshape(tmp_img(rows(i)-r:rows(i)+r, cols(i)-r:cols(i)+r), patch_size, 1);
    end
    disp(['Mine descript ' num2str(toc)]);
end
