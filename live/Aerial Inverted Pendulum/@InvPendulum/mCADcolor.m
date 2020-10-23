function mCADcolor(drone,color)
% Modify drone color

if nargin > 1
    drone.pCAD.mtl(1).Kd = color';
end

for ii = 1:length(drone.pCAD.obj.umat3)
    mtlnum = drone.pCAD.obj.umat3(ii);
    for jj=1:length(drone.pCAD.mtl)
        if strcmp(drone.pCAD.mtl(jj).name,drone.pCAD.obj.usemtl(mtlnum-1))
            break;
        end
    end
    fvcd3(ii,:) = drone.pCAD.mtl(jj).Kd';
end

drone.pCAD.i3D.FaceVertexCData  = fvcd3;
end