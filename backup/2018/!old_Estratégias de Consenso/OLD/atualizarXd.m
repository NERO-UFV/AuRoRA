function obj = atualizarXd(obj,dXi)


obj.pPos.Xda = obj.pPos.Xd;
obj.pPos.Xd(1:3) = obj.pPos.Xda(1:3) + dXi * obj.pTempo.Ts;
obj.pPos.Xd(7:12) = (obj.pPos.Xd(1:6) - obj.pPos.Xda(1:6)) / obj.pTempo.Ts;

obj.pPos.dXd = (obj.pPos.Xd - obj.pPos.Xda)  / obj.pTempo.Ts;

obj.pPos.Xtil = obj.pPos.Xd - obj.pPos.X;