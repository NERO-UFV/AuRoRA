function mApagarCAD(obj)

if isfield(obj.pCAD,'ObjImag')
    delete(obj.pCAD.ObjImag)
    obj.pCAD = rmfield(obj.pCAD,'ObjImag');
end

end