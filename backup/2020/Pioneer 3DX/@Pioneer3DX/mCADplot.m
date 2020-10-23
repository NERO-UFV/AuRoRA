% Plot Pioneer in 3D space according to his posture defined by
% X = [x y z psi theta phi dx dy dz dpsi dtheta dphi]^T
% obj.pCAD.Face define transparence level

function mCADplot(p3dx,scale,color)

% 
%     scale = .4;   % model size scale
%     color = 'k';
tr = 1;  % transparency

% p3dx.pCAD.CockpitColor = color;

%%% Rotation Matrix
RotX = [1 0 0; 0 cos(p3dx.pPos.Xc(4)) -sin(p3dx.pPos.Xc(4)); 0 sin(p3dx.pPos.Xc(4)) cos(p3dx.pPos.Xc(4))];
RotY = [cos(p3dx.pPos.Xc(5)) 0 sin(p3dx.pPos.Xc(5)); 0 1 0; -sin(p3dx.pPos.Xc(5)) 0 cos(p3dx.pPos.Xc(5))];
RotZ = [cos(p3dx.pPos.Xc(6)) -sin(p3dx.pPos.Xc(6)) 0; sin(p3dx.pPos.Xc(6)) cos(p3dx.pPos.Xc(6)) 0; 0 0 1];

Rot = RotZ*RotY*RotX;

% ObjImag Robot parts

% Vertices
for k = 1:size(p3dx.pCAD.vertices,2)
    r1 = Rot * p3dx.pCAD.vertices{k}*scale;
%     p3dx.pCAD.ObjImag(k) = patch(r1(1,:)+p3dx.pPos.Xc(1), r1(2,:)+p3dx.pPos.Xc(2), r1(3,:)+p3dx.pPos.Xc(3),'k','facealpha',.5);
    p3dx.pCAD.ObjImag(k) = patch(r1(1,:)+p3dx.pPos.Xc(1), r1(2,:)+p3dx.pPos.Xc(2), r1(3,:)+p3dx.pPos.Xc(3),color,'facealpha',tr);
    
end
indice = k;

% Wheels
for k=1:size(p3dx.pCAD.rodas,2)
    indice = indice + 1;
    r2 = Rot * p3dx.pCAD.rodas{k}*scale;
    p3dx.pCAD.ObjImag(indice) = patch(r2(1,:)+p3dx.pPos.Xc(1), r2(2,:)+p3dx.pPos.Xc(2), r2(3,:)+p3dx.pPos.Xc(3),'k','facealpha',tr);
end

% Body
for k=1:size(p3dx.pCAD.corpo,2)
    indice = indice +1;
    r3 = Rot * p3dx.pCAD.corpo{k}*scale;
    p3dx.pCAD.ObjImag(indice) = patch(r3(1,:)+p3dx.pPos.Xc(1), r3(2,:)+p3dx.pPos.Xc(2), r3(3,:)+p3dx.pPos.Xc(3),'r','facealpha',tr);
%     p3dx.pCAD.ObjImag(indice) = patch(r3(1,:)+p3dx.pPos.Xc(1), r3(2,:)+p3dx.pPos.Xc(2), r3(3,:)+p3dx.pPos.Xc(3),color,'facealpha',tr);

end

end