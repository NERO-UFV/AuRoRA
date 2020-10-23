function SC = calculaSC(obj)

Kp1 = [1,0;0,1]*.1;
Kp2 = [1,0;0,1]*2;
Kp3 = [1,0;0,1]*.5;

K = [ cos(obj.pPos.X(6)), -obj.pPar.a*sin(obj.pPos.X(6)); ...
    sin(obj.pPos.X(6)), +obj.pPar.a*cos(obj.pPos.X(6))];

% SC = K\(obj.pPos.dXd(1:2)*.5+Kp1*obj.pPos.Xtil(1:2));
SC = K\(Kp3*obj.pPos.dXd(1:2)*.5+Kp1*tanh(Kp2*obj.pPos.Xtil(1:2)) );
