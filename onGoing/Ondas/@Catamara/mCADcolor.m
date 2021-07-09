function mCADcolor(boat,color)
% Modify drone color

if nargin > 1
    %boat.pCAD.mtl.Kd = color';
    boat.pCAD.i3D.FaceColor = color';
end

% for ii = 1:length(boat.pCAD.obj.umat3)
%     mtlnum = boat.pCAD.obj.umat3(ii);
%     for jj=1:length(boat.pCAD.mtl)
%         if strcmp(boat.pCAD.mtl(jj).name,boat.pCAD.obj.usemtl(mtlnum-1))
%             break;
%         end
%     end
%     fvcd3(ii,:) = boat.pCAD.mtl(jj).Kd';
% end
% 
% boat.pCAD.i3D.FaceVertexCData  = fvcd3;
end