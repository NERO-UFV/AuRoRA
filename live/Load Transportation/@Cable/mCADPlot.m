function mCADPlot(cable,load,drone)
  
cable.pCAD.CableImag.XData = [load.pPos.X(1) drone.pPos.X(1)];
cable.pCAD.CableImag.YData = [load.pPos.X(2) drone.pPos.X(2)];
cable.pCAD.CableImag.ZData = [load.pPos.X(3) drone.pPos.X(3)];
 
end