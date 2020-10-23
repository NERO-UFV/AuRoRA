clearvars
close all
clc

A = linspace(0,2*pi,100);
Ox = cos(A);
Oy = sin(A);
Bx = 2/1.5*[0; -1.5; -1.5; -1.5; 0];
By = 2/1.5*[-1.5; -1.5; 0; 1.5; 1.5];
Cx = 1.5*cos(A(26:75));
Cy = 1.5*sin(A(26:75));

figure
plot(Ox,Oy,'-r','LineWidth',1.6);
hold on
plot(Cx,Cy,'--b','LineWidth',1.6);
plot(Bx,By,':k');
plot(Bx,By,'ok','LineWidth',1.6,'MarkerSize',10);
grid on
axis(2*[-2 2 -2 2])
axis equal

Qx = [1; 1; -1; -1; 1];
Qy = [-1; 1; 1; -1; -1];
Bqx = [0; -2; -1.5; -2; 0];
Bqy = [-1.5; -2; 0; 2; 1.5];
Bp = [Bqx(1:3) Bqy(1:3)];
syms t
B = bernsteinMatrix(2,t);
bezierCurve = simplify(B*Bp);
bezierCurve2 = simplify(B*[Bqx(3:5) Bqy(3:5)]);
bezierCurve3 = simplify(B*[Bx(1:3) By(1:3)]);

figure
plot(Qx,Qy,'-r','LineWidth',1.6);
hold on
fplot(bezierCurve(1),bezierCurve(2),[0,1],'--b','LineWidth',1.6);
fplot(bezierCurve2(1),bezierCurve2(2),[0,1],'--b','LineWidth',1.6);
fplot(bezierCurve3(1),bezierCurve3(2),[0,1],'--g','LineWidth',1.6);
plot(Bqx(1:5),Bqy(1:5),':k');
plot(Bx(1:3),By(1:3),'-k');
plot(Bx(1:3),By(1:3),'ok','LineWidth',1.6,'MarkerSize',10);
plot(Bqx,Bqy,'ok','LineWidth',1.6,'MarkerSize',10);
grid on
axis(2*[-2 2 -2 2])
axis equal

