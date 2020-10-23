function mCADcolor(obj,color)
% Modify drone color

if nargin > 1
    obj.pCAD.mtl(1).Kd = color';
end

for ii = 1:length(obj.pCAD.obj.umat3)
    mtlnum = obj.pCAD.obj.umat3(ii);
    for jj=1:length(obj.pCAD.mtl)
        if strcmp(obj.pCAD.mtl(jj).name,obj.pCAD.obj.usemtl(mtlnum-1))
            break;
        end
    end
    fvcd3(ii,:) = obj.pCAD.mtl(jj).Kd';
end

obj.pCAD.i3D.FaceVertexCData  = fvcd3;
end