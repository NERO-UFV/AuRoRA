function mCADCreate(load)
 
hold on
load.pCAD.ObjImag = plot3(load.pPos.X(1),load.pPos.X(2),load.pPos.X(3),'.','Color','k','MarkerSize',load.pPar.m*150);
load.pCAD.CableImag = plot3([load.pPos.X(1) load.pPos.X(1)],[load.pPos.X(2) load.pPos.X(2)],[load.pPos.X(3) load.pPos.X(1)],'Color','k');
hold off
 
end