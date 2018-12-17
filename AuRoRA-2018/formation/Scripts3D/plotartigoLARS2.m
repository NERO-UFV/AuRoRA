% Rotina para plotar gráficos de Formacaolinha3D
clear all
close all
clc

% Busca pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

% Declara os robos
P = Pioneer3DX;
A = ArDrone;

% Parameters
scale  = 1;     % robot model plot scale
Pcolor = 'k';   % robot A color

% Configura janela do gráfico
figure;
axis equal
grid off
hold on
% Retira os números dos eixos
set(gca,'xticklabel',[])
set(gca,'yticklabel',[])
set(gca,'zticklabel',[])
set(gca,'Box','off')
% Define posição dos robôs
P.pPos.Xc(1:3) = [0 0 0];      % Pioneer
A.pPos.X(1:3) = [0 1 .2];   % AR.Drone

% Plota robos
% P.mCADplot(scale,Pcolor);
A.mCADplot;

drawnow
view(45,30)

% Plota  Eixos
% %% Pioneer
% h = .2; 
% c = .5;     % axis length
% 
% 
% % Eixo x
% x1 = [P.pPos.Xc(1)   P.pPos.Xc(1)+c];
% y1 = [P.pPos.Xc(2)   P.pPos.Xc(2)];
% z1 = [P.pPos.Xc(3)+h   P.pPos.Xc(3)+h];
% 
% % Eixo y
% x2 = [P.pPos.Xc(1)    P.pPos.Xc(1)];
% y2 = [P.pPos.Xc(2)    P.pPos.Xc(2)+c];
% z2 = [P.pPos.Xc(3)+h   P.pPos.Xc(3)+h];
% 
% % Eixo z
% x3 = [P.pPos.Xc(1)   P.pPos.Xc(1)];
% y3 = [P.pPos.Xc(2)   P.pPos.Xc(2)];
% z3 = [P.pPos.Xc(3)+h   P.pPos.Xc(3)+c+h];
% 
% 
% p1 = line(x1,y1,z1);
% p1.Color = 'b';
% p1.LineStyle = '-';
% p1.LineWidth = 1.5;
% 
% p2 = line(x2,y2,z2);
% p2.Color = 'g';
% p2.LineStyle = '-';
% p2.LineWidth = 1.5;
% 
% p3 = line(x3,y3,z3);
% p3.Color = 'r';
% p3.LineStyle = '-';
% p3.LineWidth = 1.5;
% 
% % Nomeia dos eixos
% tx = text(P.pPos.Xc(1)+c+.03,P.pPos.Xc(2),P.pPos.Xc(3)+h,'X'); 
% ty = text(P.pPos.Xc(1),P.pPos.Xc(2)+c,P.pPos.Xc(3)+h,'Y'); 
% tz = text(P.pPos.Xc(1),P.pPos.Xc(2)+.05,P.pPos.Xc(3)+h+c,'Z'); 

%% ArDrone
% Eixo x
c2 = .4;   % axis length

Ax1 = [A.pPos.X(1)   A.pPos.X(1)+c2];
Ay1 = [A.pPos.X(2)   A.pPos.X(2)];
Az1 = [A.pPos.X(3)   A.pPos.X(3)];

% Eixo y
Ax2 = [A.pPos.X(1)   A.pPos.X(1)];
Ay2 = [A.pPos.X(2)   A.pPos.X(2)+c2];
Az2 = [A.pPos.X(3)   A.pPos.X(3)];

% Eixo z
Ax3 = [A.pPos.X(1)   A.pPos.X(1)];
Ay3 = [A.pPos.X(2)   A.pPos.X(2)];
Az3 = [A.pPos.X(3)   A.pPos.X(3)+c2];


Ap1 = line(Ax1,Ay1,Az1);
Ap1.Color = 'k'%'b';
Ap1.LineStyle = '-';
Ap1.LineWidth = 1.5;

Ap2 = line(Ax2,Ay2,Az2);
Ap2.Color = 'k'%'g';
Ap2.LineStyle = '-';
Ap2.LineWidth = 1.5;

Ap3 = line(Ax3,Ay3,Az3);
Ap3.Color = 'k'%'r';
Ap3.LineStyle = '-';
Ap3.LineWidth = 1.5;

% % Nomeia dos eixos
% tx = text(A.pPos.X(1)+c2+.03,A.pPos.X(2),A.pPos.X(3),'X'); 
% ty = text(A.pPos.X(1),A.pPos.X(2)+c2,A.pPos.X(3),'Y'); 
% tz = text(A.pPos.X(1),A.pPos.X(2)+.05,A.pPos.X(3)+c2,'Z'); 
