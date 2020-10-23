%% Rotina para plotar dois pioneers em formação

clear 
close all
clc

% cria robos
PA = Pioneer3DX;
PB = Pioneer3DX;

% seta pose
PA.rSetPose([0 0 0 0])
PB.rSetPose([5 1.5 0 0])

% Configura janela de plot
figure;
hold on
axis([-1 5 -1 5])
axis equal 
axis off             % retira eixos

%% Plota robôs
PA.mCADplot(2,'b')
PB.mCADplot(2,'r')

%% Plota  linhas
% Parâmetros gerais
sizeLine   = 2;
sizeSymbol = 20;
h          = 0.4;    % altura das linhas 

% rho line
xl1 = [PA.pPos.X(1)   PB.pPos.X(1) ];
yl1 = [PA.pPos.X(2)   PB.pPos.X(2) ];
zl1 =  [h h ];

pl = line(xl1,yl1,zl1);
pl.Color = 'k';
pl.LineStyle = '-';
pl.LineWidth = 2;
hold on

% Centro de referência
P = [(PA.pPos.X(1)+PB.pPos.X(1))/2 (PA.pPos.X(2)+PB.pPos.X(2))/2 ];
plot3(P(1),P(2),h,'k.','MarkerSize',sizeSymbol,'LineWidth',sizeLine)

% Pontos extremos da linha 
plot3(PA.pPos.X(1),PA.pPos.X(2),h,'k.','MarkerSize',sizeSymbol,'LineWidth',sizeLine);
plot3(PB.pPos.X(1),PB.pPos.X(2),h,'k.','MarkerSize',sizeSymbol,'LineWidth',sizeLine);

% Linha do ângulo alpha
xl2 = [PA.pPos.X(1)   P(1) ];
yl2 = [PA.pPos.X(2)   PA.pPos.X(2) ];
zl2 =  [h  h];

pl = line(xl2,yl2,zl2);
pl.Color = 'k';
pl.LineStyle = '--';
pl.LineWidth = 2;
hold on

