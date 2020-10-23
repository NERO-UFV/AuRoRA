clear all
close all
clc

% CARREGANDO A CLASSE
TF{1} = TriangularFormationBaju(1);

% RAIO DO CILINDRO E ESFERA DE PROTEÇÃO
RAIOe = 1.5;
RAIOc = RAIOe*sin(pi/4);

% POSIÇÃO DO CENTRO DA FORMAÇÃO
Xv = 0;
Yv = 0;
Zv = 0;

% POSIÇÃO INICIAL DA FORMAÇÃO
Xfi = Xv;
Yfi = Yv;
Zfi = Zv;

% ORIENTAÇÃO INICIAL DA FORMAÇÃO
THETAfi = 0;
PHIfi   = 0;
PSIfi   = pi/2;

% POSTURA INICIAL DA FORMAÇÃO
Pfi = RAIOe;
BETAfi = atan2(RAIOc,Pfi);
Qfi = RAIOc/sin(BETAfi);

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
Point(1) = plot3(X(1),X(2),X(3),'ok','MarkerSize',10,'LineWidth',2);
Point(2) = plot3(X(4),X(5),X(6),'^k','MarkerSize',10,'LineWidth',2);
Point(3) = plot3(X(7),X(8),X(9),'^k','MarkerSize',10,'LineWidth',2);
% P = plot3(Xv,Yv,Zv,'xk','MarkerSize',20);
TPlot(1) = text(X(1)+.1,X(2)+.1,X(3)+.1,'Pioneer','FontWeight','bold');
TPlot(2) = text(X(4)+.1,X(5)+.1,X(6)+.1,'ArDrone1','FontWeight','bold');
TPlot(3) = text(X(7)+.1,X(8)+.1,X(9)+.1,'ArDrone2','FontWeight','bold');
axis equal
axis([-2 2 -2 2 0 2])
view(65,30)
pause

% VARIAVEIS SIMULAÇÃO
T_MAX = 5;
T_AMOSTRAGEM = 1/30;
T_PLOT = 1/30;

T = tic;
Ta = tic;
Tp = tic;
TF{1}.pPar.ti = tic;

% LAÇO
while toc(T) < T_MAX
   if toc(Ta) > T_AMOSTRAGEM 
       Ta = tic;
              
       TF{1}.pPos.QdA = TF{1}.pPos.Qd;
       
%        TF{1}.pPos.Qd = [Xv; Yv; Zv;
%                         THETAfi+(-BETAfi-THETAfi)*toc(T)/T_MAX; PHIfi; PSIfi; 
%                         Pfi+(RAIOc/sin(BETAfi)-Pfi)*toc(T)/T_MAX; Qfi+(RAIOc/tan(BETAfi)-Qfi)*toc(T)/T_MAX; BETAfi];
%        
       R2d = RAIOc-RAIOc*toc(T)/T_MAX; 
       THETAfd = -atan2((RAIOc-R2d),RAIOe);
%        Pfd = RAIOe/cos(THETAfd);
       Pfd = sqrt((RAIOc-R2d)^2+RAIOe^2);
       Qfd = sqrt(R2d^2+RAIOe^2);
       TF{1}.pPos.Qd = [Xv; Yv; Zv;
                        THETAfd; PHIfi; PSIfi; 
                        Pfd; Qfd; acos((Pfd^2+Qfd^2-RAIOc^2)/(2*Pfd*Qfd))];            
       
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





















