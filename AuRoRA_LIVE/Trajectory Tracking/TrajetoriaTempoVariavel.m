% Referência com tempo variável
clearvars
close all
clc
tf = 60;
t = tic;

w = 2*pi/60;

figure
hold
X1 = [];
X2 = [];

while  toc(t) < tf

    tt = toc(t); 
    
    a = 3*(tt/tf)^2 - 2*(tt/tf)^3;
    
    tp = a*tf;
    
    X1 = [X1,[sin(w*tt); sin(2*w*tt)]];
    X2 = [X2,[sin(w*tp); sin(2*w*tp)]];
    
    try
        delete(h1)
        delete(h2)
    end
    
    h1 = plot(X1(1,:),X1(2,:),'-r','linewidth',2);
    h2 = plot(X2(1,:),X2(2,:)+0.1,'-k','linewidth',2);
    
    axis([-2 2 -2 2])
%     h = plot(tt,tp,'bo');
%     axis([0 tf 0 tf])
    grid on
    drawnow
    
end

