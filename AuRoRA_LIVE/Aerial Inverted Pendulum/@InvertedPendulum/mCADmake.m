function mCADmake(pendulum)

[X,Y,Z] = sphere(8);
X = X*pendulum.pPar.R;
Y = Y*pendulum.pPar.R;
Z = Z*pendulum.pPar.R + pendulum.pPar.r;
pendulum.pCAD.ball = patch(surf2patch(X,Y,Z,Z));
pendulum.pCAD.ball.FaceAlpha = 0.9;
pendulum.pCAD.ball.FaceColor = [1 0 0];
pendulum.pCAD.ball.EdgeColor = [1 0.5 0.5];

[X,Y,Z] = cylinder(0.005,6);
Z = Z*pendulum.pPar.r;
pendulum.pCAD.bar  = patch(surf2patch(X,Y,Z,Z));
pendulum.pCAD.bar.FaceAlpha = 0.5;

end