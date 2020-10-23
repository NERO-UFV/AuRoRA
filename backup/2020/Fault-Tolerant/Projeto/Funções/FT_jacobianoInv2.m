function X = FT_jacobianoInv2(Q,dQr)
% Q = [x;     1
%      y;     2
%      z;     3
%      phi;   4
%      psi;   5
%      tetha; 6
%      p;     7
%      q;     8
%      beta]; 9

I = [1 0 0;
       0 1 0;
       0 0 1];

Nulo = [0 0 0;
          0 0 0;
          0 0 0];

J{1} = [Q(7)*cos(Q(4))*cos(Q(5))                                    -Q(7)*sin(Q(4))*sin(Q(5))          0;
        Q(7)*cos(Q(4))*sin(Q(5))*cos(Q(6))+Q(7)*sin(Q(4))*sin(Q(6)) Q(7)*sin(Q(4))*cos(Q(5))*cos(Q(6)) -Q(7)*sin(Q(4))*sin(Q(5))*sin(Q(6))-Q(7)*cos(Q(4))*cos(Q(6));
        Q(7)*cos(Q(4))*sin(Q(5))*sin(Q(6))-Q(7)*sin(Q(4))*cos(Q(6)) Q(7)*sin(Q(4))*cos(Q(5))*sin(Q(6)) Q(7)*sin(Q(4))*sin(Q(5))*cos(Q(6))-Q(7)*cos(Q(4))*sin(Q(6))];

J{2} = [sin(Q(4))*cos(Q(5))                               0 0;
        sin(Q(4))*sin(Q(5))*cos(Q(6))-cos(Q(4))*sin(Q(6)) 0 0;
        sin(Q(4))*sin(Q(5))*sin(Q(6))+cos(Q(4))*cos(Q(6)) 0 0];
    
J{3} = [Q(8)*cos(Q(4)-Q(9))*cos(Q(5))                                         -Q(8)*sin(Q(4)-Q(9))*sin(Q(5))               0;
        Q(8)*cos(Q(4)-Q(9))*sin(Q(5))*cos(Q(6))+Q(8)*sin(Q(4)-Q(9))*sin(Q(6)) Q(8)*sin(Q(4)-Q(9))*cos(Q(5))*cos(Q(6)) -Q(8)*sin(Q(4)-Q(9))*sin(Q(5))*sin(Q(6))-Q(8)*cos(Q(4)-Q(9))*cos(Q(6));
        Q(8)*cos(Q(4)-Q(9))*sin(Q(5))*sin(Q(6))-Q(8)*sin(Q(4)-Q(9))*cos(Q(6)) Q(8)*sin(Q(4)-Q(9))*cos(Q(5))*sin(Q(6)) Q(8)*sin(Q(4)-Q(9))*sin(Q(5))*cos(Q(6))-Q(8)*cos(Q(4)-Q(9))*sin(Q(6))];

J{4} = [0 sin(Q(4)-Q(9))*cos(Q(5))                               -Q(8)*cos(Q(4)-Q(9))*cos(Q(5));
        0 sin(Q(4)-Q(9))*sin(Q(5))*cos(Q(6))-cos(Q(4))*sin(Q(6)) -Q(8)*cos(Q(4)-Q(9))*sin(Q(5))*cos(Q(6))-Q(8)*sin(Q(4)-Q(9))*sin(Q(6));
        0 sin(Q(4)-Q(9))*sin(Q(5))*sin(Q(6))+cos(Q(4))*cos(Q(6)) -Q(8)*cos(Q(4)-Q(9))*sin(Q(5))*sin(Q(6))+Q(8)*sin(Q(4)-Q(9))*cos(Q(6))];

J_inv = [I Nulo Nulo;
         I J{1} J{2};
         I J{3} J{4}];
J_inv
X = J_inv*dQr;
end


