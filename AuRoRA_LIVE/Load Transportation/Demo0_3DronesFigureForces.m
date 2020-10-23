% Inicialização
close all
clear
clc

try
    fclose(instrfindall);
catch
end

% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

gains = [1 2 1 2 2 15;
    2 13 2 15 1 5];
%
% Robot initialization
A{1} = ArDrone;
A{2} = ArDrone;
A{3} = ArDrone;

mCADcolor(A{1},[1 0 0])
mCADcolor(A{2},[0 1 0])
mCADcolor(A{2},[0 0 1])
L{1} = Load;
L{2} = Load;
L{3} = Load;

c3 = cos(30*pi/180); c4 = cos(45*pi/180); c6 = cos(60*pi/180);
s3 = sin(30*pi/180); s4 = sin(45*pi/180); s6 = sin(60*pi/180);
%% Figure
ObjF.Fig(1) = figure(1);
set(ObjF.Fig(1),'units','pix','pos',[300 150 1200/1.3 800/1.3],'PaperPositionMode','auto')

axis image
ObjF.xlim = [-1 1.0]*1.0;
ObjF.ylim = [-1 1.0]*1.0;
ObjF.zlim = [ 0 1.5];
xlabel('x'); ylabel('y'); zlabel('z');
axis([ObjF.xlim ObjF.ylim ObjF.zlim])

grid on

L{1}.mCADCreate; L{2}.mCADCreate; L{3}.mCADCreate
hold on
A{1}.mCADplot; A{2}.mCADplot; A{3}.mCADplot;
L{1}.mCADPlot(A{1}); L{1}.mCADPlot(A{2}); L{1}.mCADPlot(A{3});
% Linhas entre os VANTs
pd(1) = plot3([A{1}.pPos.X(1) A{2}.pPos.X(1)],[A{1}.pPos.X(2) A{2}.pPos.X(2)],[A{1}.pPos.X(3) A{2}.pPos.X(3)],'--');
pd(2) = plot3([A{1}.pPos.X(1) A{3}.pPos.X(1)],[A{1}.pPos.X(2) A{3}.pPos.X(2)],[A{1}.pPos.X(3) A{3}.pPos.X(3)],'--');
pd(3) = plot3([A{2}.pPos.X(1) A{3}.pPos.X(1)],[A{2}.pPos.X(2) A{3}.pPos.X(2)],[A{2}.pPos.X(3) A{3}.pPos.X(3)],'--');

% tx(1,1) = text(A{1}.pPos.X(1)+.3,A{1}.pPos.X(2),A{1}.pPos.X(3),'A_1');
% tx(2,1) = text(A{2}.pPos.X(1)+.3,A{2}.pPos.X(2),A{2}.pPos.X(3),'A_2');
% tx(3,1) = text(A{3}.pPos.X(1)-.3,A{3}.pPos.X(2),A{3}.pPos.X(3),'A_3');
%%
ph(1) = plot3([0 0],[0 0],[0 0]);
ph(2) = plot3([0 0],[0 0],[0 0]);
ph(3) = plot3([0 0],[0 0],[0 0]);

%%
% Posição da carga
L{1}.pPos.X(1:3) = [0 0 .5]; L{2}.pPos.X = L{1}.pPos.X; L{3}.pPos.X = L{1}.pPos.X;

% Posição da carga
l = L{1}.pPar.l;
x0 = L{1}.pPos.X(1);
y0 = L{1}.pPos.X(2);
z0 = L{1}.pPos.X(3);

p0 = L{1}.pPos.X(1:3);
p1 = A{1}.pPos.X(1:3);
p2 = A{2}.pPos.X(1:3);
p3 = A{3}.pPos.X(1:3);

A{1}.pPos.X(1:3) = [x0+l*c4*s3 y0+l*c4*s6 z0+l*c4];
A{2}.pPos.X(1:3) = [x0+l*c4*s3 y0-l*c4*s6 z0+l*c4];
A{3}.pPos.X(1:3) = [x0-l*c4 y0 z0+l*c4];

% A{3}.pPos.X(1) = -.6;

A{1}.mCADplot; A{2}.mCADplot; A{3}.mCADplot;
L{1}.mCADPlot(A{1}); L{2}.mCADPlot(A{2}); L{3}.mCADPlot(A{3});
% Plota as linhas entre os drones
pd(1).XData = [A{1}.pPos.X(1) A{2}.pPos.X(1)]; pd(1).YData = [A{1}.pPos.X(2) A{2}.pPos.X(2)]; pd(1).ZData = [A{1}.pPos.X(3) A{2}.pPos.X(3)];
pd(2).XData = [A{1}.pPos.X(1) A{3}.pPos.X(1)]; pd(2).YData = [A{1}.pPos.X(2) A{3}.pPos.X(2)]; pd(2).ZData = [A{1}.pPos.X(3) A{3}.pPos.X(3)];
pd(3).XData = [A{2}.pPos.X(1) A{3}.pPos.X(1)]; pd(3).YData = [A{2}.pPos.X(2) A{3}.pPos.X(2)]; pd(3).ZData = [A{2}.pPos.X(3) A{3}.pPos.X(3)];

% Distância entre os drones
d(1) = norm(A{1}.pPos.X(1:3) - A{2}.pPos.X(1:3));
d(2) = norm(A{1}.pPos.X(1:3) - A{3}.pPos.X(1:3));
d(3) = norm(A{2}.pPos.X(1:3) - A{3}.pPos.X(1:3));
disp(['d = ' num2str(d)])

%Distancia entre Drone e caga
dl(1) = norm(A{1}.pPos.X(1:3) - L{1}.pPos.X(1:3));
dl(2) = norm(A{2}.pPos.X(1:3) - L{1}.pPos.X(1:3));
dl(3) = norm(A{3}.pPos.X(1:3) - L{1}.pPos.X(1:3));
disp(['dl = ' num2str(dl)])

%Ponto xy da carga
pf = [(A{1}.pPos.X(1)+A{2}.pPos.X(1)+A{3}.pPos.X(1))/3
    (A{1}.pPos.X(2)+A{2}.pPos.X(2)+A{3}.pPos.X(2))/3];
disp(['Pf = ' num2str(pf')])

%Ângulos entre os veículos
beta = [acos((d(1)^2+d(2)^2-d(3)^2)/(2*d(1)*d(2)))
    acos((d(1)^2-d(2)^2+d(3)^2)/(2*d(1)*d(3)))
    acos((-d(1)^2+d(2)^2+d(3)^2)/(2*d(2)*d(3)))]';
disp(['Beta = ' num2str(beta*180/pi)])



% distância entre drone e o plano formado pelos outros dois
h = [sqrt(1/2*(d(1)^2+d(2)^2-(d(3)^2)/2))
    sqrt(1/2*(d(1)^2+d(3)^2-(d(2)^2)/2))
    sqrt(1/2*(d(2)^2+d(3)^2-(d(1)^2)/2))
    ];
disp(['h = ' num2str(h')])

% distância entre a carga e o plano formado pelos drones dois a dois
dp(1) = abs((p2(2)-p3(2))*p0(1) - (p2(1)-p3(1))*p0(2) + p2(1)*p3(2) - p2(2)*p3(1))/(sqrt((p2(2)-p3(2))^2+(p2(1)-p3(1))^2));
dp(2) = abs((p1(2)-p3(2))*p0(1) - (p1(1)-p3(1))*p0(2) + p1(1)*p3(2) - p1(2)*p3(1))/(sqrt((p1(2)-p3(2))^2+(p1(1)-p3(1))^2));
dp(3) = abs((p2(2)-p1(2))*p0(1) - (p2(1)-p1(1))*p0(2) + p2(1)*p1(2) - p2(2)*p1(1))/(sqrt((p2(2)-p1(2))^2+(p2(1)-p1(1))^2));
disp(['dp = ' num2str(dp)])

%
dp(1) = abs((p2(2)-p3(2))*p0(1) - (p2(1)-p3(1))*p0(2) + p2(1)*p3(2) - p2(2)*p3(1))/(sqrt((p2(2)-p3(2))^2+(p2(1)-p3(1))^2));
dp(2) = abs((p1(2)-p3(2))*p0(1) - (p1(1)-p3(1))*p0(2) + p1(1)*p3(2) - p1(2)*p3(1))/(sqrt((p1(2)-p3(2))^2+(p1(1)-p3(1))^2));
dp(3) = abs((p2(2)-p1(2))*p0(1) - (p2(1)-p1(1))*p0(2) + p2(1)*p1(2) - p2(2)*p1(1))/(sqrt((p2(2)-p1(2))^2+(p2(1)-p1(1))^2));
disp(['dp = ' num2str(dp)])

dh(1) = abs((p2(2)-p3(2))*p1(1) - (p2(1)-p3(1))*p1(2) + p2(1)*p3(2) - p2(2)*p3(1))/(sqrt((p2(2)-p3(2))^2+(p2(1)-p3(1))^2));
dh(2) = abs((p1(2)-p3(2))*p2(1) - (p1(1)-p3(1))*p2(2) + p1(1)*p3(2) - p1(2)*p3(1))/(sqrt((p1(2)-p3(2))^2+(p1(1)-p3(1))^2));
dh(3) = abs((p2(2)-p1(2))*p3(1) - (p2(1)-p1(1))*p3(2) + p2(1)*p1(2) - p2(2)*p1(1))/(sqrt((p2(2)-p1(2))^2+(p2(1)-p1(1))^2));
disp(['dh = ' num2str(dh)])

%Ângulos entre os veículos e a carga no plano xy
alpha = [acos((d(1)^2+d(2)^2-(d(3)^2)/4)/(2*d(1)*d(2)))
    acos((d(1)^2-(d(2)^2)/4+d(3)^2)/(2*d(1)*d(3)))
    acos((-(d(1)^2)/4+d(2)^2+d(3)^2)/(2*d(2)*d(3)))]';
disp(['Alpha = ' num2str(alpha*180/pi)])

alpha = [acos((4*(h(1)^2+h(2)^2)-9*d(3)^2)/(8*h(1)*h(2)))
    acos((4*(h(1)^2+h(3)^2)-9*d(2)^2)/(8*h(1)*h(3)))
    acos((4*(h(2)^2+h(3)^2)-9*d(1)^2)/(8*h(2)*h(3)))]';
disp(['Alpha = ' num2str(alpha*180/pi)])

%Ângulos entre os veículos e a carga no plano xy
alpha = [pi/2-beta(2)
    pi/2-beta(3)
    pi/2-beta(1)];
disp(['Alpha = ' num2str(alpha'*180/pi)])

%%Ângulos entre os veículos e a carga no plano vertical
gamma = [atan2(L{1}.pPar.l,h(1))
    atan2(L{1}.pPar.l,h(2))
    atan2(L{1}.pPar.l,h(3))
    ]';
disp(['Gamma = ' num2str(gamma*180/pi)])


%Ângulos entre os veículos e a carga no plano vertical
gamma = [asin(norm(A{1}.pPos.X(1:2) - L{1}.pPos.X(1:2))/dl(1))
    asin(norm(A{2}.pPos.X(1:2) - L{1}.pPos.X(1:2))/dl(2))
    asin(norm(A{3}.pPos.X(1:2) - L{1}.pPos.X(1:2))/dl(3))
    ]';
disp(['Gamma = ' num2str(gamma*180/pi)])


ll = [L{1}.pPar.l*sin(gamma(1))
    L{1}.pPar.l*sin(gamma(2))
    L{1}.pPar.l*sin(gamma(3))
    ];

%Ângulos entre os veículos
alpha = [acos((d(1)^2+ll(1)^2-ll(2)^2)/(2*d(1)*ll(1)))
    acos((d(3)^2+ll(2)^2-ll(3)^2)/(2*d(3)*ll(2)))
    acos((d(2)^2+ll(3)^2-ll(1)^2)/(2*d(2)*ll(1)))]';
disp(['Alpha = ' num2str(alpha*180/pi)])

lll = [L{1}.pPar.l*sin(alpha(1))
    L{1}.pPar.l*sin(alpha(2))
    L{1}.pPar.l*sin(alpha(3))
    ];

%Ângulos entre os veículos
gamma = [acos((dh(1)^2+l^2-lll(2)^2)/(2*dh(1)*l))
    acos((dh(3)^2+l^2-lll(3)^2)/(2*dh(3)*l))
    acos((dh(2)^2+l^2-lll(1)^2)/(2*dh(2)*l))]';
disp(['Gamma = ' num2str(gamma*180/pi)])

ll = [L{1}.pPar.l*sin(gamma(1))
    L{1}.pPar.l*sin(gamma(2))
    L{1}.pPar.l*sin(gamma(3))
    ];

%Ângulos entre os veículos
alpha = [acos((d(1)^2+ll(1)^2-ll(2)^2)/(2*d(1)*ll(1)))
    acos((d(3)^2+ll(2)^2-ll(3)^2)/(2*d(3)*ll(2)))
    acos((d(2)^2+ll(3)^2-ll(1)^2)/(2*d(2)*ll(1)))]';
disp(['Alpha = ' num2str(alpha*180/pi)])

% hf = [d(1)*cos(alpha(1))
%       d(3)*cos(alpha(2))
%       d(2)*cos(alpha(3))
%       ];
% disp(['hf = ' num2str(hf')])

disp(' ');disp(' ');
%%
if false
    ph(1).XData = [A{1}.pPos.X(1) A{1}.pPos.X(1) + h(1)*cos(deg2rad(-120))];
    ph(1).YData = [A{1}.pPos.X(2) A{1}.pPos.X(2) + h(1)*sin(deg2rad(-120))];
    ph(1).ZData = [A{1}.pPos.X(3) A{1}.pPos.X(3)];
    
    ph(2).XData = [A{2}.pPos.X(1) A{2}.pPos.X(1) + h(2)*cos(deg2rad(120))];
    ph(2).YData = [A{2}.pPos.X(2) A{2}.pPos.X(2) + h(2)*sin(deg2rad(120))];
    ph(2).ZData = [A{2}.pPos.X(3) A{2}.pPos.X(3)];
    
    ph(3).XData = [A{3}.pPos.X(1) A{3}.pPos.X(1) + h(3)*cos(deg2rad(0))];
    ph(3).YData = [A{3}.pPos.X(2) A{3}.pPos.X(2) + h(3)*sin(deg2rad(0))];
    ph(3).ZData = [A{3}.pPos.X(3) A{3}.pPos.X(3)];
end
% view(0,0)
%% Agora o inverso
if false
    % A{1}.pPos.X(1) = .3;
    % A{2}.pPos.X(1) = .3;
    
    A{3}.pPos.X(1) = -.3;
    
    % A{1}.pPos.X(2) = .3;
    % Distância entre os drones
    d(1) = norm(A{1}.pPos.X(1:3) - A{2}.pPos.X(1:3));
    d(2) = norm(A{1}.pPos.X(1:3) - A{3}.pPos.X(1:3));
    d(3) = norm(A{2}.pPos.X(1:3) - A{3}.pPos.X(1:3));
    disp(['d = ' num2str(d)])
    
    %Ângulos entre os drones
    beta = [acos((d(1)^2+d(2)^2-d(3)^2)/(2*d(1)*d(2)))
        acos((d(1)^2-d(2)^2+d(3)^2)/(2*d(1)*d(3)))
        acos((-d(1)^2+d(2)^2+d(3)^2)/(2*d(2)*d(3)))]';
    disp(['Beta = ' num2str(beta*180/pi)])
    
    %Ângulos entre os veículos e a carga no plano xy
    alpha = [pi/2-beta(2)
        pi/2-beta(3)
        pi/2-beta(1)];
    disp(['Alpha = ' num2str(alpha'*180/pi)])
    
    %Ângulos entre os veículos e a carga no plano xy
    alpha = [acos((d(1)^2+d(2)^2-(d(3)^2)/4)/(2*d(1)*d(2)))
        acos((d(1)^2-(d(2)^2)/4+d(3)^2)/(2*d(1)*d(3)))
        acos((-(d(1)^2)/4+d(2)^2+d(3)^2)/(2*d(2)*d(3)))]';
    disp(['Alpha = ' num2str(alpha*180/pi)])
    
    h = [sqrt(1/2*(d(1)^2+d(2)^2-(d(3)^2)/2))
        sqrt(1/2*(d(1)^2+d(3)^2-(d(2)^2)/2))
        sqrt(1/2*(d(2)^2+d(3)^2-(d(1)^2)/2))
        ];
    disp(['hf = ' num2str(h')])
    
    h = [d(1)*cos(alpha(1))
        d(3)*cos(alpha(2))
        d(2)*cos(alpha(3))
        ];
    disp(['hf = ' num2str(h')])
    
    %Distancia entre Drone e caga
    dl(1) = norm(A{1}.pPos.X(1:3) - L{1}.pPos.X(1:3));
    dl(2) = norm(A{2}.pPos.X(1:3) - L{1}.pPos.X(1:3));
    dl(3) = norm(A{3}.pPos.X(1:3) - L{1}.pPos.X(1:3));
    disp(['dl = ' num2str(dl)])
    
    %%%Ponto xy da carga
    pf = [(A{1}.pPos.X(1)+A{2}.pPos.X(1)+A{3}.pPos.X(1))/3
        (A{1}.pPos.X(2)+A{2}.pPos.X(2)+A{3}.pPos.X(2))/3];
    disp(['Pf = ' num2str(pf')])
    
    disp(' ');disp(' ');disp(' ');
    
    L{1}.pPos.X(1:3) = [pf(1) pf(2) .40]; L{2}.pPos.X = L{1}.pPos.X; L{3}.pPos.X = L{1}.pPos.X;
    
    A{1}.mCADplot; A{2}.mCADplot; A{3}.mCADplot;
    L{1}.mCADPlot(A{1}); L{2}.mCADPlot(A{2}); L{3}.mCADPlot(A{3});
    % Plota as linhas entre os drones
    pd(1).XData = [A{1}.pPos.X(1) A{2}.pPos.X(1)]; pd(1).YData = [A{1}.pPos.X(2) A{2}.pPos.X(2)]; pd(1).ZData = [A{1}.pPos.X(3) A{2}.pPos.X(3)];
    pd(2).XData = [A{1}.pPos.X(1) A{3}.pPos.X(1)]; pd(2).YData = [A{1}.pPos.X(2) A{3}.pPos.X(2)]; pd(2).ZData = [A{1}.pPos.X(3) A{3}.pPos.X(3)];
    pd(3).XData = [A{2}.pPos.X(1) A{3}.pPos.X(1)]; pd(3).YData = [A{2}.pPos.X(2) A{3}.pPos.X(2)]; pd(3).ZData = [A{2}.pPos.X(3) A{3}.pPos.X(3)];
    %
    % %%%Plota as distâncias entre drone e os planos formados pelos outros dois
    % ph(1).XData = [A{1}.pPos.X(1) A{1}.pPos.X(1) + h(1)*cos(deg2rad(270)-alpha(1))];
    % ph(1).YData = [A{1}.pPos.X(2) A{1}.pPos.X(2) + h(1)*sin(deg2rad(270)-alpha(1))];
    % ph(1).ZData = [A{1}.pPos.X(3) A{1}.pPos.X(3)];
    %
    % ph(2).XData = [A{2}.pPos.X(1) A{2}.pPos.X(1) + h(2)*cos(deg2rad(90)+alpha(2))];
    % ph(2).YData = [A{2}.pPos.X(2) A{2}.pPos.X(2) + h(2)*sin(deg2rad(90)+alpha(3))];
    % ph(2).ZData = [A{2}.pPos.X(3) A{2}.pPos.X(3)];
    %
    % ph(3).XData = [A{3}.pPos.X(1) A{3}.pPos.X(1) + h(3)*cos(deg2rad(30)-alpha(3))];
    % ph(3).YData = [A{3}.pPos.X(2) A{3}.pPos.X(2) + h(3)*sin(deg2rad(30)-alpha(3))];
    % ph(3).ZData = [A{3}.pPos.X(3) A{3}.pPos.X(3)];
    view(0,0)
    
end
%%
if false
    %%
    figure(1)
    saveas(gcf,'TripleX','epsc')
%     disp(['File ',strin{idx},'.eps Saved']);
    
    
   
    
    
end