function X = FT_jacobianoInv(Q,dQr)
% Q = [x;     1
%      y;     2
%      z;     3
%      phi;   4
%      psi;   5
%      tetha; 6
%      p;     7
%      q;     8
%      beta]; 9

J_inv = [1 0 0 0                               0                                0 0                   0                          0;
         0 1 0 0                               0                                0 0                   0                          0;
         0 0 1 0                               0                                0 0                   0                          0;
         1 0 0 Q(7)*cos(Q(4))*cos(Q(5))        -Q(7)*sin(Q(4))*sin(Q(5))        0 sin(Q(4))*cos(Q(5)) 0                          0;
         0 1 0 Q(7)*cos(Q(4))*sin(Q(5))        Q(7)*sin(Q(4))*cos(Q(5))         0 sin(Q(4))*sin(Q(5)) 0                          0;
         0 0 1 -Q(7)*sin(Q(4))                 0                                0 cos(Q(4))           0                          0;
         1 0 0 Q(8)*cos(Q(4) - Q(9))*cos(Q(5)) -Q(8)*sin(Q(4) - Q(9))*sin(Q(6)) 0 0                   sin(Q(4) - Q(9))*cos(Q(5)) -Q(8)*cos(Q(4) - Q(9))*cos(Q(5));
         0 1 0 Q(8)*cos(Q(4) - Q(9))*sin(Q(5)) Q(8)*sin(Q(4) - Q(9))*cos(Q(6))  0 0                   sin(Q(4) - Q(9))*sin(Q(5)) -Q(8)*cos(Q(4) - Q(9))*sin(Q(5));
         0 0 1 -Q(8)*sin(Q(4) - Q(9))          0                                0 0                   cos(Q(4) - Q(9))           Q(8)*sin(Q(4) - Q(9))];

X = J_inv*dQr;
end


