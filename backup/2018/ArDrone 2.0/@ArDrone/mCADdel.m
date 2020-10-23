function mCADdel(drone)
% Delete drone CAD model

if isfield(drone.pCAD,'i3D')
    delete(drone.pCAD.i3D)
    drone.pCAD = rmfield(drone.pCAD,'i3D');
end

end