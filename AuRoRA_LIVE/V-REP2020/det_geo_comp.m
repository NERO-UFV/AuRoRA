rb = OPT.RigidBody;  % read optitrack
A = getOptData(rb(idA),A); % Atualiza dados ArDrone
B1 = getOptData(rb(idB1),B1); % Atualiza dados bambolê livre
Rz1 = [cos(B1.pPos.X(6)) -sin(B1.pPos.X(6)) 0;...
sin(B1.pPos.X(6)) cos(B1.pPos.X(6)) 0; 0 0 1];



% Bambolê 1:
Bmb = B1.pPos.X(1:3)'; 
Nb = Rz1*[1;0;0]; Nb = Rn*Nb; 
Vb = B1.pPos.X(1:3)+[0;0.63;0]; Vb = Vb./norm(Vb); 
Ub = B1.pPos.X(1:3)+[0;0;.63];  Ub = Ub./norm(Ub);
Nb = Nb'; Ub = Ub'; Vb = Vb';

M = [A.pPos.X(1:2)',1;
Bmb(1:2)+dD.*(Nb(1:2)/norm(Nb)),1;
Bmb(1:2),1];
det(M)
sign(det(M))
disp(Nb)