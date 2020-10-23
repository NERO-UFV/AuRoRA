clear all
close all
clc

% Parametros

tm = 10;
ta = 0.01;

figure
axis equal
hold on
grid on

t2 = tic;
pause
t = tic;

while toc(t) < tm
    if toc(t2) > ta
        t2 = tic;
        w = 2*pi/tm;
        x1 = sin(w*toc(t)/2);
        y = toc(t)/10;
        z1 = cos(w*toc(t)/2);
        
        x2 = -sin(w*toc(t)/2);
        z2 = -cos(w*toc(t)/2);
        
        plot3(x1,y,z1,'.k')
        plot3(x2,y,z2,'.b')
        drawnow
    end

end