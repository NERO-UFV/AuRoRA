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
%%
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

%% Figure
ObjF.Fig(1) = figure(1);
set(ObjF.Fig(1),'units','pix','pos',[300 150 1200/1.3 800/1.3],'PaperPositionMode','auto')

axis image
ObjF.xlim = [-1 1.0]*2;
ObjF.ylim = [-1 1.0]*2;
ObjF.zlim = [ 0 1.5];
xlabel('x'); ylabel('y'); zlabel('z');
axis([ObjF.xlim ObjF.ylim ObjF.zlim])

grid on

L{1}.mCADCreate
L{2}.mCADCreate
L{3}.mCADCreate
hold on
A{1}.mCADplot;
A{2}.mCADplot;
A{3}.mCADplot;

L{1}.mCADPlot(A{1});
L{2}.mCADPlot(A{2});
L{3}.mCADPlot(A{3});
%%
d(1) = plot3([A{1}.pPos.X(1) A{2}.pPos.X(1)],[A{1}.pPos.X(2) A{2}.pPos.X(2)],[A{1}.pPos.X(3) A{2}.pPos.X(3)],'--');
d(2) = plot3([A{1}.pPos.X(1) A{3}.pPos.X(1)],[A{1}.pPos.X(2) A{3}.pPos.X(2)],[A{1}.pPos.X(3) A{3}.pPos.X(3)],'--');
d(3) = plot3([A{2}.pPos.X(1) A{3}.pPos.X(1)],[A{2}.pPos.X(2) A{3}.pPos.X(2)],[A{2}.pPos.X(3) A{3}.pPos.X(3)],'--');

%%
c3 = cos(30*pi/180);
c4 = cos(45*pi/180);
c6 = cos(60*pi/180);

s3 = sin(30*pi/180);
s4 = sin(45*pi/180);
s6 = sin(60*pi/180);

L{1}.pPos.X(1:3) = [0 0 .5]; L{2}.pPos.X = L{1}.pPos.X; L{3}.pPos.X = L{1}.pPos.X;
A{1}.pPos.X(1:3) = [L{1}.pPos.X(1)+L{1}.pPar.l*c4*s3 L{1}.pPos.X(2)+L{1}.pPar.l*c4*s6 L{1}.pPos.X(3)+L{1}.pPar.l*c4];
A{2}.pPos.X(1:3) = [L{1}.pPos.X(1)+L{1}.pPar.l*c4*s3 L{1}.pPos.X(2)-L{1}.pPar.l*c4*s6 L{1}.pPos.X(3)+L{1}.pPar.l*c4];
A{3}.pPos.X(1:3) = [L{1}.pPos.X(1)-L{1}.pPar.l*c4 L{1}.pPos.X(2) L{1}.pPos.X(3)+L{1}.pPar.l*c4];


A{1}.mCADplot;
A{2}.mCADplot;
A{3}.mCADplot;

L{1}.mCADPlot(A{1});
L{2}.mCADPlot(A{2});
L{3}.mCADPlot(A{3});

%
dist(1) = norm(A{1}.pPos.X(1:3) - A{2}.pPos.X(1:3));
dist(2) = norm(A{1}.pPos.X(1:3) - A{3}.pPos.X(1:3));
dist(3) = norm(A{2}.pPos.X(1:3) - A{3}.pPos.X(1:3));

dist(4) = norm(A{1}.pPos.X(1:3) - L{1}.pPos.X(1:3));
dist(5) = norm(A{2}.pPos.X(1:3) - L{1}.pPos.X(1:3));
dist(6) = norm(A{3}.pPos.X(1:3) - L{1}.pPos.X(1:3));

d(1).XData = [A{1}.pPos.X(1) A{2}.pPos.X(1)]; d(1).YData = [A{1}.pPos.X(2) A{2}.pPos.X(2)]; d(1).ZData = [A{1}.pPos.X(3) A{2}.pPos.X(3)];
d(2).XData = [A{1}.pPos.X(1) A{3}.pPos.X(1)]; d(2).YData = [A{1}.pPos.X(2) A{3}.pPos.X(2)]; d(2).ZData = [A{1}.pPos.X(3) A{3}.pPos.X(3)];
d(3).XData = [A{2}.pPos.X(1) A{3}.pPos.X(1)]; d(3).YData = [A{2}.pPos.X(2) A{3}.pPos.X(2)]; d(3).ZData = [A{2}.pPos.X(3) A{3}.pPos.X(3)];

disp(dist)
%%
tx(1,1) = text(A{1}.pPos.X(1)+.3,A{1}.pPos.X(2),A{1}.pPos.X(3),'A_1');
tx(2,1) = text(A{2}.pPos.X(1)+.3,A{2}.pPos.X(2),A{2}.pPos.X(3),'A_2');
tx(3,1) = text(A{3}.pPos.X(1)-.3,A{3}.pPos.X(2),A{3}.pPos.X(3),'A_3');

tx(1,2) = text(A{1}.pPos.X(1)-A{2}.pPos.X(1)+.3,A{1}.pPos.X(1)-A{1}.pPos.X(1),A{1}.pPos.X(3),num2str(dist(1),'%3.2f'));
tx(2,2) = text(norm(A{1}.pPos.X(1)+A{3}.pPos.X(1))/2-A{1}.pPos.X(1),norm(A{1}.pPos.X(2)+A{3}.pPos.X(2))/2-A{1}.pPos.X(2),A{1}.pPos.X(3),num2str(dist(2),'%3.2f'));
tx(3,2) = text(norm(A{2}.pPos.X(1)+A{3}.pPos.X(1))/2-A{2}.pPos.X(1),-norm(A{2}.pPos.X(2)+A{3}.pPos.X(2))/2-A{2}.pPos.X(2),A{1}.pPos.X(3),num2str(dist(3),'%3.2f'));

%%


