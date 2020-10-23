function mDefinirPosturaInicial(obj,Xo)

obj.pPos.Xs([1 2 6]) = Xo;

obj.pPos.Xc([1 2 3]) = obj.pPos.Xs([1 2 3]) - ...
    [obj.pPar.a*cos(obj.pPos.Xs(6)); obj.pPar.a*sin(obj.pPos.Xs(6)); 0];
obj.pPos.Xc([4 5 6]) = obj.pPos.Xs([4 5 6]);

obj.pPos.X  = obj.pPos.Xs;
obj.pPos.Xd = obj.pPos.X;
end

%if obj.verificaConexao
%    arrobot_setpose(obj.pPos.Xc(1)*1000,obj.pPos.Xc(2)*1000,obj.pPos.Xc(6)*180/pi);
%end
