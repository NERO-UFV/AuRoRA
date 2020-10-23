function mCADCreate(cable,load,drone)
 
hold on
cable.pCAD.CableImag = plot3([drone.pPos.X(1) load.pPos.X(1)],[drone.pPos.X(2) load.pPos.X(2)],[drone.pPos.X(3) load.pPos.X(3)],'Color','k');
hold off
 
end