function mCADCreate(obstacle)


hold on
% Create the cilinder
[X,Y,Z] = cylinder(obstacle.pPar.r);
X = X+obstacle.pPos.X(1)*ones(size(X));
Y = Y+obstacle.pPos.X(2)*ones(size(Y));
Z = obstacle.pPos.X(3)*Z;
obstacle.pCAD.ObjImag{1} = surf(X,Y,Z,'facecolor',[0.5020 0.1647 0.1647],'edgecolor',[0.5020 0.1647 0.1647]/1.5);

% Create the circle
t = linspace(0, 2*pi);
x = obstacle.pPar.r*cos(t)+obstacle.pPos.X(1);
y = obstacle.pPar.r*sin(t)+obstacle.pPos.X(2);
z = ones(size(x))*obstacle.pPos.X(3);

obstacle.pCAD.ObjImag{2} = fill3(x, y, z,[0.5020 0.1647 0.1647]);
hold off

end