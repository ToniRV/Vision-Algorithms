function scores = harris(img, patch_size, kappa)
% Slower code
% hor_sobel = fspecial('sobel');
% ver_sobel = hor_sobel';
% 
% Especially this function here...
% tmp_Ix = imfilter(double(img), hor_sobel, 'conv', 'same');
% tmp_Iy = imfilter(double(img), ver_sobel, 'conv', 'same');
% 
% Ix = tmp_Ix(2:size(tmp_Ix,1)-1,2:size(tmp_Ix,2)-1);
% Iy = tmp_Iy(2:size(tmp_Iy,1)-1,2:size(tmp_Iy,2)-1);

sobel_para = [-1 0 1];
sobel_orth = [1 2 1];

Ix = conv2(sobel_orth', sobel_para, img, 'valid');
Iy = conv2(sobel_para', sobel_orth, img, 'valid');

lxx = double(Ix .^ 2);
lyy = double(Iy .^ 2);
Ixy = double(Ix .* Iy);

patch = ones(patch_size, patch_size) / (patch_size ^ 2);
pr = floor(patch_size / 2);  % patch radius
sIxx = conv2(lxx, patch, 'valid');
sIyy = conv2(lyy, patch, 'valid');
sIxy = conv2(Ixy, patch, 'valid');

scores = (sIxx .* sIyy - sIxy .^ 2) ... determinant
    - kappa * (sIxx + sIyy) .^ 2;  % square trace

scores(scores<0) = 0;

scores = padarray(scores, [1+pr 1+pr]);

end
