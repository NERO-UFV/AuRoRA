clear all
close all
clc

%%
P = [1 2;
    5 6];
plot(P(1,1),P(1,2),'ob')
hold on
plot(P(2,1),P(2,2),'ob')
grid on
axis([0 7, 0 7])
axis equal

tmax = 5;
ta = 0.1;
ts = tic;
pause
t = tic;
while toc(t) < tmax
if toc(ts) > ta    
    x = P(1,1) + toc(t)/tmax*(P(2,1) - P(1,1));
    y = P(1,2) + toc(t)/tmax*(P(2,2) - P(1,2));
    plot(x,y,'.r')
    drawnow
    ts = tic;
end
end

%%
f=figure(1);
set(f,'WindowButtonDownFcn',@clickFunction)
iptPointerManager(f, 'enable');
iptSetPointerBehavior(axis, @(f, currentPoint)set(f, 'Pointer', 'cross'));
plot(1,1,'ok')
hold on
plot(5,5,'ok')
plot(10,10,'ok')
pause
while true
    try
        plot(clickPoint(1,1),clickPoint(1,2),'xk');
        clickPoint = [];
    end
    disp('oi')
    drawnow
end

%%

pos = find(4==Nomes')
Nomes = Nomes';
Nomes(pos) = 0;
for i=pos:(size(Nomes,1)*size(Nomes,2))
    if Nomes(i) ~= 0
        Nomes(i) = Nomes(i) - 1;
    end
end
Nomes = Nomes'


%%

temp = FECHADO;
temp_teste = FECHADO;
valor = 3;
while size(valor,1) ~= 0
%     t = find(valor(end) == temp_teste(:,1));
    valor_t = temp_teste(find(valor(end) == temp_teste(:,4)),1);
    disp(valor(end))
    temp_teste(find(valor(end) == temp_teste(:,1)),:) = [];
    valor = valor(1:(end-1),:);
    valor((end+1):(end+size(valor_t,1)),1) = valor_t;
end
% temp(t,:) = [];











