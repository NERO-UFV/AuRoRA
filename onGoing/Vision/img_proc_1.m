close all
img1 = imread('C:\Users\alexa\Dropbox\AuRoRA 2018\Vision\Screenshot_1.png');
% I = impyramid(I, 'reduce');

med = uint8(mean(img1,3));
% moda = uint(mode(In,3));
img = img1-med;
[featureVector,hogVisualization] = extractHOGFeatures(I);
% n=0;
% [featureVector,hugVisualization] = extractHOGFeatures(img,'CellSize',[32 32]);

I = impyramid(img, 'reduce');
I = impyramid(I, 'reduce');
I = impyramid(I, 'reduce');

img = impyramid(img, 'reduce');
img = impyramid(img, 'reduce');
img = impyramid(img, 'reduce');

img1 = impyramid(img1, 'reduce');
img1 = impyramid(img1, 'reduce');
img1 = impyramid(img1, 'reduce');

% figure(1)
% subplot(121)
% imshow(I)
% hold on
% plot(hogVisualization)
% subplot(222)
% mean
BW1 = imbinarize(img,'global');
BW = edge(BW1(:,:,2),'canny');
% BW = edge(im2bw(img),'canny');
% BW = impyramid(BW, 'reduce');
% BW = impyramid(BW, 'reduce');
% figure(2)
% imshow(BW)

% figure(3)
[H,T,R] = hough(BW);
% imshow(H,[],'XData',T,'YData',R,...
%             'InitialMagnification','fit');
% xlabel('\theta'), ylabel('\rho');
% axis on, axis normal, hold on;

P  = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));
x = T(P(:,2)); y = R(P(:,1));
% plot(x,y,'s','color','white');

lines = houghlines(BW,T,R,P,'FillGap',5,'MinLength',7);
% figure(4)
% imshow(img1)
hold on
max_len = 0;
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   
%    plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
%    plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%    plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end
% plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','cyan');