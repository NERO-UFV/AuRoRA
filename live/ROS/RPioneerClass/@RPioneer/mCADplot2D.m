function mCADplot2D(obj,cor)

% Exibe o ArDrone no espaco 3D de acordo com sua postura definida por
% X = [x y z psi theta phi dx dy dz dpsi dtheta dphi]^T
% obj.pCAD.Face indica o grau de transparencia de cada parte do veiculo

%%% Matriz de Rotacao
RotX = [1 0 0; 0 cos(obj.pPos.Xc(4)) -sin(obj.pPos.Xc(4)); 0 sin(obj.pPos.Xc(4)) cos(obj.pPos.Xc(4))];
RotY = [cos(obj.pPos.Xc(5)) 0 sin(obj.pPos.Xc(5)); 0 1 0; -sin(obj.pPos.Xc(5)) 0 cos(obj.pPos.Xc(5))];
RotZ = [cos(obj.pPos.Xc(6)) -sin(obj.pPos.Xc(6)) 0; sin(obj.pPos.Xc(6)) cos(obj.pPos.Xc(6)) 0; 0 0 1];

Rot = RotZ*RotY*RotX;

% ObjImag das partes do robô
C  = Rot * obj.pCAD.corpo2D;
Rd = Rot * obj.pCAD.rodad2D;
Re = Rot * obj.pCAD.rodae2D;

obj.pCAD.ObjImag(1) = patch(C(1,:)+obj.pPos.Xc(1), C(2,:)+obj.pPos.Xc(2), C(3,:)+obj.pPos.Xc(3),cor);
obj.pCAD.ObjImag(2) = patch(Rd(1,:)+obj.pPos.Xc(1), Rd(2,:)+obj.pPos.Xc(2), Rd(3,:)+obj.pPos.Xc(3),'k');
obj.pCAD.ObjImag(3) = patch(Re(1,:)+obj.pPos.Xc(1), Re(2,:)+obj.pPos.Xc(2), Re(3,:)+obj.pPos.Xc(3),'k');

end