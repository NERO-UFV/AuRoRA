clear all
close all
clc

% CARREGANDO A CLASSE
TF{1} = TriangularFormationBaju(1);

% POSIÇÃO DO CENTRO DA FORMAÇÃO
Xv = 1;
Yv = 1;
Zv = 0;

% POSIÇÃO INICIAL DA FORMAÇÃO
Xfi = Xv + 1;
Yfi = Yv - sqrt(3)/3;
Zfi = Zv;

% ORIENTAÇÃO INICIAL DA FORMAÇÃO
THETAfi = 0;
PHIfi   = -pi/2;
PSIfi   = pi/2;

% POSTURA INICIAL DA FORMAÇÃO
Pfi = 2;
Qfi = 2;
BETAfi = pi/3;

% ATRIBUINDO PARA FORMAÇÃO
TF{1}.pPos.Q = [Xfi; Yfi; Zfi;
                THETAfi; PHIfi; PSIfi;
                Pfi; Qfi; BETAfi];

TF{1}.pPos.Qd = TF{1}.pPos.Q; 

% TRANSFORMADA INVERSA
TF{1}.tInvTrans;
TF{1}.pPos.X = TF{1}.pPos.Xd;
TF{1}.tDirTrans;

X = TF{1}.pPos.Xd;
% TF{1}.pPos.X(1:3) = X(7:9);
% TF{1}.pPos.X(4:9) = X(1:6);

% PLOT
figure
H(1) = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'b','LineWidth',2);
hold on
H(2) = plot3([X(1) X(7)],[X(2) X(8)],[X(3) X(9)],'b','LineWidth',2);
H(3) = plot3([X(7) X(4)],[X(8) X(5)],[X(9) X(6)],'r','LineWidth',2);
grid on
Point(1) = plot3(X(1),X(2),X(3),'^k','MarkerSize',10,'LineWidth',2);
Point(2) = plot3(X(4),X(5),X(6),'^k','MarkerSize',10,'LineWidth',2);
Point(3) = plot3(X(7),X(8),X(9),'^k','MarkerSize',10,'LineWidth',2);
% P = plot3(Xv,Yv,Zv,'xk','MarkerSize',20);
TPlot(1) = text(X(1)+.1,X(2)+.1,X(3)+.1,'ArDrone1','FontWeight','bold');
TPlot(2) = text(X(4)+.1,X(5)+.1,X(6)+.1,'ArDrone2','FontWeight','bold');
TPlot(3) = text(X(7)+.1,X(8)+.1,X(9)+.1,'ArDrone3','FontWeight','bold');
axis equal
axis([-2 2 -2 2 -2 2])
view(0,90)
pause

% VARIAVEIS SIMULAÇÃO
T_MAX = 5;
T_AMOSTRAGEM = 1/30;
T_PLOT = 1/30;

T = tic;
Ta = tic;
Tp = tic;
TF{1}.pPar.ti = tic;

rx = 2*sqrt(3)/3;
ry = 2*sqrt(3)/3;
W = 2*pi/T_MAX;
Xd = [rx*cos(W*toc(T)); ry*sin(W*toc(T)); 1];


% LAÇO
while toc(T) < 2*T_MAX
   if toc(Ta) > T_AMOSTRAGEM 
       Ta = tic;
       
%        Vda = Vd;
%        Vd = [rx*cos(W*toc(T)); ry*sin(W*toc(T)); 1];
%        dVd = (Vd - Vda)/T_AMOSTRAGEM;
       
       TF{1}.pPos.QdA = TF{1}.pPos.Qd;
       TF{1}.pPos.Qd = [Xv+rx*cos(W*toc(T)-pi/6); Yv+ry*sin(W*toc(T)-pi/6); Zv;
                        THETAfi+(-2*pi-THETAfi)*toc(T)/T_MAX; PHIfi; PSIfi; 
                        Pfi; Qfi; BETAfi];
       TF{1}.pPos.dQd = (TF{1}.pPos.Qd - TF{1}.pPos.QdA)/T_AMOSTRAGEM;
       
       
       TF{1}.pPos.XdA = TF{1}.pPos.Xd;
       TF{1}.tInvTrans;
       
       TF{1}.tFormationControl;
       
       TF{1}.pPos.Xa = TF{1}.pPos.X;
       TF{1}.pPos.X = TF{1}.pPos.X + T_AMOSTRAGEM*TF{1}.pPos.dXr;
       
       X = TF{1}.pPos.X;
       XA = TF{1}.pPos.Xa;
       
%        X(1:3) = Xd;    
   end
   if toc(Tp) > T_PLOT
       Tp = tic;
       
       try
           delete(H)
           delete(Point)
           delete(TPlot)
       catch
       end
       
       H(1) = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'b','LineWidth',2);
       H(2) = plot3([X(1) X(7)],[X(2) X(8)],[X(3) X(9)],'b','LineWidth',2);
       H(3) = plot3([X(7) X(4)],[X(8) X(5)],[X(9) X(6)],'r','LineWidth',2);
       Point(1) = plot3(X(1),X(2),X(3),'^k','MarkerSize',10,'LineWidth',2);
       Point(2) = plot3(X(4),X(5),X(6),'^k','MarkerSize',10,'LineWidth',2);
       Point(3) = plot3(X(7),X(8),X(9),'^k','MarkerSize',10,'LineWidth',2);
       TPlot(1) = text(X(1)+.1,X(2)+.1,X(3)+.1,'ArDrone1','FontWeight','bold');
       TPlot(2) = text(X(4)+.1,X(5)+.1,X(6)+.1,'ArDrone2','FontWeight','bold');
       TPlot(3) = text(X(7)+.1,X(8)+.1,X(9)+.1,'ArDrone3','FontWeight','bold');
       Rastro(1) = plot3([XA(1) X(1)],[XA(2) X(2)],[XA(3) X(3)],'-r','LineWidth',2);
       Rastro(2) = plot3([XA(4) X(4)],[XA(5) X(5)],[XA(6) X(6)],'-g','LineWidth',2);
       Rastro(3) = plot3([XA(7) X(7)],[XA(8) X(8)],[XA(9) X(9)],'-b','LineWidth',2);
      
       drawnow
   end
end

Q = [0; 0; 0;
     pi/2; pi/6; pi/3; %phi theta psi
     1; 1; pi/3];

X = [0.25; 0.433; 1.7113;
     0; 0;  2.5774;
     -0.25; -0.433; 1.7113];
 
A1A2 = [X(4)-X(1) X(5)-X(2) X(6)-X(3)]';
H1A1 = -[(X(7)+X(4))/2-X(1) (X(8)+X(5))/2-X(2) (X(9)+X(6))/2-X(3)]';
 
% N = cross(A1H1,A1A2);
    
% thetaf = atan2(norm(N(1:2)),N(3));
% psif = pi - atan2(N(2),N(1));
% psif = atan2(N(2),N(1));

psif = atan2(H1A1(2),H1A1(1));
thetaf = -atan2(H1A1(3),(norm(H1A1(1:2))));

% Matrizes de rotação
  
% Rotação em Y
Ry = [cos(thetaf)  0 sin(thetaf);
      0            1 0;
      -sin(thetaf) 0 cos(thetaf)];
  
% Rotação em Z
Rz = [cos(psif) -sin(psif) 0;
      sin(psif) cos(psif)  0;
      0         0          1];
  
% Rotação em Y,X e Z nesta ordem
A1A2 = Rz\A1A2';
A1A2 = Ry\A1A2;

H1A1 = Rz\H1A1;
H1A1 = Ry\H1A1;

N = cross(A1A2,H1A1);

phif = atan2(N(3),N(2))+pi/2;

% Rotação em X
Rx = [1 0         0;
      0 cos(phif) -sin(phif);
      0 sin(phif) cos(phif)];

Xc = [(X(1)+X(4)+X(7))/3;
      (X(2)+X(5)+X(8))/3;
      (X(3)+X(6)+X(9))/3];
      
Xc = [Xc;Xc;Xc];
  
X = X-Xc;
  
X(1:3) = Rz\X(1:3);
X(4:6) = Rz\X(4:6);
X(7:9) = Rz\X(7:9);

X(1:3) = Ry\X(1:3);
X(4:6) = Ry\X(4:6);
X(7:9) = Ry\X(7:9);

X(1:3) = Rx\X(1:3);
X(4:6) = Rx\X(4:6);
X(7:9) = Rx\X(7:9);





















