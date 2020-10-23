function mCADPlot(load,drone)
 
load.pCAD.ObjImag.XData = load.pPos.X(1);
load.pCAD.ObjImag.YData = load.pPos.X(2);
load.pCAD.ObjImag.ZData = load.pPos.X(3);
 
load.pCAD.CableImag.XData = [load.pPos.X(1) drone.pPos.X(1)];
load.pCAD.CableImag.YData = [load.pPos.X(2) drone.pPos.X(2)];
load.pCAD.CableImag.ZData = [load.pPos.X(3) drone.pPos.X(3)];
 
end