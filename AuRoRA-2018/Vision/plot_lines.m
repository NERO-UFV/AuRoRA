function plot_lines(path)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
img1 = imread(path);

med = uint8(mean(img1,3));
img = img1-med;

I = impyramid(img, 'reduce');
I = impyramid(I, 'reduce');
I = impyramid(I, 'reduce');

img = impyramid(img, 'reduce');
img = impyramid(img, 'reduce');
img = impyramid(img, 'reduce');

img1 = impyramid(img1, 'reduce');
img1 = impyramid(img1, 'reduce');
img1 = impyramid(img1, 'reduce');

BW1 = imbinarize(img,'global');
BW = edge(BW1(:,:,2),'canny'); %canny, sobel, log

[H,T,R] = hough(BW);
P  = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));
x = T(P(:,2)); y = R(P(:,1));


lines = houghlines(BW,T,R,P,'FillGap',5,'MinLength',7);
figure(1)
imshow(img1)
hold on
max_len = 0;
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end
plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','cyan');
end

