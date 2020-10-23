clear all
close all
clc

K = diag([.2 .2]);
a = 0.2;
ta = 0.1;
tmax = 60;
t = tic;
tc = tic;

X = [-1 0 0]';
Xc = [X(1) + a*cos(X(3)); X(2) + a*sin(X(3)); pi];

c = 0:0.1:2*pi;
Corpo = [X(1) + a*cos(c); X(2) + a*sin(c)];

T = 30;
w = 2*pi/T;
raio_a = 1.5;
raio_b = 1.5;
rastro = [];
rastroC = [];
rastroT = [];

n = 0;
filename = 'TestePioneer.gif';

h = figure;
while toc(t) < tmax
    if toc(tc) > ta
        tc = tic;
        
        Xd = [raio_a*cos(w*toc(t)); raio_b*sin(w*toc(t))];
        dXd = [-raio_a*w*sin(w*toc(t));raio_b*w*cos(w*toc(t))];
        %         Xd = [1.5;1.5];
        %         dXd = [0;0];
        
        %         Xtil = Xd - X(1:2);
        Xtil = Xd - Xc(1:2);
        
        Xref = dXd + K*Xtil;
        
        F = [cos(X(3)) -a*sin(X(3));
            sin(X(3))  a*cos(X(3))];
        
        U = F\Xref;
        
        %         X(1:2) = X(1:2) + Xref*ta;
        %         X(3) = X(3) + U(2)*ta;
        %         Xc = [X(1) + a*cos(X(3)); X(2) + a*sin(X(3))];
        Xc(1:2) = Xc(1:2) + Xref*ta;
        Xc(3) = Xc(3) + U(2)*ta;
        X = [Xc(1) - a*cos(X(3)); Xc(2) - a*sin(X(3)); Xc(3)];
        Corpo = [X(1) + a*cos(c); X(2) + a*sin(c)];
        rastro = [rastro X];
        rastroC = [rastroC Xc];
        rastroT = [rastroT Xd];
        
        plot(X(1),X(2),'*')
        hold on
        plot(Xc(1),Xc(2),'r*')
        plot(Xd(1),Xd(2),'k*')
        plot(Corpo(1,:),Corpo(2,:),'k')
        plot(rastro(1,:),rastro(2,:),'k')
        plot(rastroC(1,:),rastroC(2,:),'r')
        plot(rastroT(1,:),rastroT(2,:),'k--')
        axis(2*[-1 1 -1 1]);
        hold off
        grid on
        %         drawnow
        
        frame = getframe(h);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        % Write to the GIF File
        if n == 0
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',ta);
        else
            imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',ta);
        end
        n = 1;
        drawnow
    end
end