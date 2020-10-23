% Reproduz sequencialmente os frames salvos

clear all 
close all
clc

% figure;
k = 1;

while 1
    % for k = 1:234
    try
        tic
        str = sprintf('pic%d.jpg',k);
        f = imread(str);
        imshow(f);
%         pause(.05)
%         delete pic234.jpg 
        k = k+1;
        toc
    catch
    disp('Frame não disponível!')
    break
    end

end

close all