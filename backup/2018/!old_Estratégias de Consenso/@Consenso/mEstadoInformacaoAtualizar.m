function robo = mEstadoInformacaoAtualizar(obj,robo)

if obj.pVC.ConexoesFlag
    ID = obj.pVF.ID;
else
    ID = robo.pID;
end

% Calcular dEid
Xi = obj.pVF.X(1:3,ID)*ones(1,obj.pVF.nID);
Xfi = obj.pVF.XF(1:3,ID)*ones(1,obj.pVF.nID);

Xj = obj.pVF.Ei;
Xfj = obj.pVF.XF;

dist = (Xj-Xi)-(Xfj-Xfi);
dist(isnan(dist)) = 0;

dXi = obj.pVF.X(7:9,ID)*ones(1,obj.pVF.nID);
dXj = obj.pVF.dEi;
% ddist = dXj - dXi;

% obj.pVF.dEid(:,ID) = dist*obj.pVF.Madj(ID,:)' + ddist*obj.pVF.Madj(ID,:)';
obj.pVF.dEid(:,ID) = dist*obj.pVF.Madj(ID,:)' ;

% obj.pVF.Eid = obj.pVF.Eid + obj.pVF.dEid * 0.01;
obj.pVF.Eid(:,ID) = robo.pPos.X(1:3) + obj.pVF.dEid(:,ID) * 0.1; % Período 100ms

robo.pPos.Xd(1:3) = obj.pVF.Eid(:,ID);
robo.pPos.Xd(7:9) = obj.pVF.dEid(:,ID);
robo.pPos.dXd(1:3) = obj.pVF.dEid(:,ID);

end


