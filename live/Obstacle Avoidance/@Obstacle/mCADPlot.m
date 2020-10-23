function mCADPlot(obstacle)
% Update the Osbstacle Position

obstacle.pCAD.ObjImag.XData = obstacle.pPos.X(1);
obstacle.pCAD.ObjImag.YData = obstacle.pPos.X(2);
obstacle.pCAD.ObjImag.ZData = obstacle.pPos.X(3);
end