function mCADplot(boat)
% Plot Catamaran 3D CAD model on its current position
% boat.pPos.X = [x y z psi theta phi dx dy dz dpsi dtheta dphi]^T


if boat.pCAD.flagCreated == 0
    mCADmake(boat)
    mCADplot(boat)
    
else
    % Update boat pose
    %%% Rotational matrix
    RotX = [1 0 0; 0 cos(boat.pPos.X(4)) -sin(boat.pPos.X(4)); 0 sin(boat.pPos.X(4)) cos(boat.pPos.X(4))];
    RotY = [cos(boat.pPos.X(5)) 0 sin(boat.pPos.X(5)); 0 1 0; -sin(boat.pPos.X(5)) 0 cos(boat.pPos.X(5))];
    RotZ = [cos(boat.pPos.X(6)) -sin(boat.pPos.X(6)) 0; sin(boat.pPos.X(6)) cos(boat.pPos.X(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H = [Rot boat.pPos.X(1:3); 0 0 0 1];
    
    vertices = H*[boat.pCAD.obj.v; ones(1,size(boat.pCAD.obj.v,2))];
    boat.pCAD.i3D.Vertices = vertices(1:3,:)';
end
end

% =========================================================================
function mCADmake(boat)

boat.pCAD.i3D = patch('Vertices',boat.pCAD.obj.v','Faces',boat.pCAD.obj.f3');


for ii = 1:length(boat.pCAD.obj.umat3)
    mtlnum = boat.pCAD.obj.umat3(ii);
    for jj=1:length(boat.pCAD.mtl)
        if strcmp(boat.pCAD.mtl(jj).name,boat.pCAD.obj.usemtl(mtlnum-1))
            break;
        end
    end
    fvcd3(ii,:) = boat.pCAD.mtl(jj).Kd';
    
end

boat.pCAD.i3D.FaceVertexCData = fvcd3;
boat.pCAD.i3D.FaceColor = 'flat';
% boat.pCAD.i3D.FaceColor = [0.698039 0.698039 0.698039];
boat.pCAD.i3D.EdgeColor = 'none';
boat.pCAD.i3D.FaceAlpha = 1;
boat.pCAD.i3D.Visible = 'on';


boat.pCAD.flagCreated = 1;

end