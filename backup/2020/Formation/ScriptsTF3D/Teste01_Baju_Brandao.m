% clear all
% close all
% clc


X = [1; 0; 1;
     -1; 0; 1;
     0; 0; 0];
 
figure
H(1) = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'--b','LineWidth',1);
hold on
H(2) = plot3([X(1) X(7)],[X(2) X(8)],[X(3) X(9)],'--b','LineWidth',1);
H(3) = plot3([X(7) X(4)],[X(8) X(5)],[X(9) X(6)],'--r','LineWidth',1);
grid on
Point(1) = plot3(X(1),X(2),X(3),'^k','MarkerSize',10,'LineWidth',1);
Point(2) = plot3(X(4),X(5),X(6),'^k','MarkerSize',10,'LineWidth',1);
Point(3) = plot3(X(7),X(8),X(9),'^k','MarkerSize',10,'LineWidth',1);
TPlot(1) = text(X(1)+.1,X(2)+.1,X(3)+.1,'1','FontWeight','bold');
TPlot(2) = text(X(4)+.1,X(5)+.1,X(6)+.1,'2','FontWeight','bold');
TPlot(3) = text(X(7)+.1,X(8)+.1,X(9)+.1,'3','FontWeight','bold');
axis([-3 3 -3 3 0 3])
view(50,30)

XA = X;

x1 = X(1);
y1 = X(2);
z1 = X(3);
x2 = X(4);
y2 = X(5);
z2 = X(6);
x3 = X(7);
y3 = X(8);
z3 = X(9);

Pf = [(x1+x2+x3)/3; 
      (y1+y2+y3)/3; 
      (z1+z2+z3)/3;
      atan2((2*z1-z2-z3),(2*y1-y2-y3));
      -atan((2*z1-z2-z3)/(2*x1-x2-x3));
      atan2((2*y1-y2-y3),(2*x1-x2-x3))];

pf = sqrt((x1-x2)^2+(y1-y2)^2+(z1-z2)^2);
qf = sqrt((x1-x3)^2+(y1-y3)^2+(z1-z3)^2);
rf = sqrt((x2-x3)^2+(y2-y3)^2+(z2-z3)^2);

Sf = [pf;
      qf;
      acos((pf^2+qf^2-rf^2)/(2*pf*qf))];
  
Q = [Pf; Sf];

xf = Q(1);
yf = Q(2);
zf = Q(3);
phif = Q(4);
thetaf = Q(5);
psif = Q(6);
pf = Q(7);
qf = Q(8);
betaf = Q(9);

% Matrizes de rotação
% Rotação em X
Rx = [1 0         0;
      0 cos(phif) -sin(phif);
      0 sin(phif) cos(phif)];
% Rotação em Y
Ry = [cos(thetaf)  0 sin(thetaf);
      0            1 0;
      -sin(thetaf) 0 cos(thetaf)];
% Rotação em Z
Rz = [cos(psif) -sin(psif) 0;
      sin(psif) cos(psif)  0;
      0         0          1];
% Rotação em Y,X e Z nesta ordem
R = Rz*Ry*Rx;

rf = sqrt(pf^2+qf^2-2*pf*qf*cos(betaf));
h1 = sqrt((pf^2+qf^2-rf^2/2)/2);
h2 = sqrt((rf^2+pf^2-qf^2/2)/2);
h3 = sqrt((qf^2+rf^2-pf^2/2)/2);
alfa1 = acos((4*(h1^2+h2^2)-9*pf^2)/(8*h1*h2));
alfa2 = acos((4*(h1^2+h3^2)-9*qf^2)/(8*h1*h3));

XF = [xf; yf; zf];
X1 = [2*h1/3; 0; 0];
X2 = [2*h2*cos(alfa1)/3; 2*h2*sin(alfa1)/3; 0];
X3 = [2*h3*cos(alfa2)/3; -2*h3*sin(alfa2)/3; 0]; 

X = [R*X1+XF;
     R*X2+XF;
     R*X3+XF];


% figure
H(1) = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'b','LineWidth',2);
H(2) = plot3([X(1) X(7)],[X(2) X(8)],[X(3) X(9)],'b','LineWidth',2);
H(3) = plot3([X(7) X(4)],[X(8) X(5)],[X(9) X(6)],'r','LineWidth',2);
Point(1) = plot3(X(1),X(2),X(3),'^k','MarkerSize',10,'LineWidth',2);
Point(2) = plot3(X(4),X(5),X(6),'^k','MarkerSize',10,'LineWidth',2);
Point(3) = plot3(X(7),X(8),X(9),'^k','MarkerSize',10,'LineWidth',2);
TPlot(1) = text(X(1)+.1,X(2)+.1,X(3)+.1,'1','FontWeight','bold');
TPlot(2) = text(X(4)+.1,X(5)+.1,X(6)+.1,'2','FontWeight','bold');
TPlot(3) = text(X(7)+.1,X(8)+.1,X(9)+.1,'3','FontWeight','bold');
% axis([-3 3 -3 3 0 3])
% view(50,30)


