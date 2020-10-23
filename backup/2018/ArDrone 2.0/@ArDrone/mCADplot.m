function mCADplot(drone,lines)
% Plot ArDrone 3D CAD model on its current position
% drone.pPos.X = [x y z psi theta phi dx dy dz dpsi dtheta dphi]^T

if nargin < 2
    drone.pCAD.flagLines = 0;
else
    drone.pCAD.flagLines = lines;
end

if drone.pCAD.flagLines == 1  % Create the demon line
    drone.pCAD.i2D.Vertices = [drone.pPos.X(1) drone.pPos.X(1) drone.pPos.X(1)+0.0001;...
    drone.pPos.X(2) drone.pPos.X(2) drone.pPos.X(2);...
    0 drone.pPos.X(3) drone.pPos.X(3)]';
end

if drone.pCAD.flagCreated == 0
    mCADmake(drone)
    mCADplot(drone)
else
    % Update drone pose
    %%% Rotational matrix
    RotX = [1 0 0; 0 cos(drone.pPos.X(4)) -sin(drone.pPos.X(4)); 0 sin(drone.pPos.X(4)) cos(drone.pPos.X(4))];
    RotY = [cos(drone.pPos.X(5)) 0 sin(drone.pPos.X(5)); 0 1 0; -sin(drone.pPos.X(5)) 0 cos(drone.pPos.X(5))];
    RotZ = [cos(drone.pPos.X(6)) -sin(drone.pPos.X(6)) 0; sin(drone.pPos.X(6)) cos(drone.pPos.X(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H = [Rot drone.pPos.X(1:3); 0 0 0 1];
    
    vertices = H*[drone.pCAD.obj.v; ones(1,size(drone.pCAD.obj.v,2))];
    drone.pCAD.i3D.Vertices = vertices(1:3,:)';
end
end

% =========================================================================
function mCADmake(drone)

drone.pCAD.i3D = patch('Vertices',drone.pCAD.obj.v','Faces',drone.pCAD.obj.f3');

for ii = 1:length(drone.pCAD.obj.umat3)
    mtlnum = drone.pCAD.obj.umat3(ii);
    for jj=1:length(drone.pCAD.mtl)
        if strcmp(drone.pCAD.mtl(jj).name,drone.pCAD.obj.usemtl(mtlnum-1))
            break;
        end
    end
    fvcd3(ii,:) = drone.pCAD.mtl(jj).Kd';
    %fvcd(ii,:) = rand(1,3);
end

drone.pCAD.i3D.FaceVertexCData = fvcd3;
drone.pCAD.i3D.FaceColor = 'flat';
drone.pCAD.i3D.EdgeColor = 'none';
drone.pCAD.i3D.FaceAlpha = 0.6;
drone.pCAD.i3D.Visible = 'on';

drone.pCAD.flagCreated = 1;

% Por algum motivo isso ta criando uma linha do demo
% drone.pCAD.i2D = patch([drone.pPos.X(1) drone.pPos.X(1) drone.pPos.X(1)+0.0001],...
%     [drone.pPos.X(2) drone.pPos.X(2) drone.pPos.X(2)],...
%     [0 drone.pPos.X(3) drone.pPos.X(3)],[0 0 0]);
end