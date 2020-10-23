function mCADdel(obj)
% Delete drone CAD model

if isfield(obj.pCAD,'i3D')
    delete(obj.pCAD.i3D)
    obj.pCAD = rmfield(obj.pCAD,'i3D');
end

end