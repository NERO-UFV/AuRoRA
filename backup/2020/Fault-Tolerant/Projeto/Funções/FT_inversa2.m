function X = FT_inversa2(Q)
% Q = [x;     1
%      y;     2
%      z;     3
%      phi;   4
%      psi;   5
%      theta; 6
%      p;     7
%      q;     8
%      beta]; 9

R = [cos(Q(5))          -sin(Q(5))          0;
    sin(Q(5))*cos(Q(6)) cos(Q(5))*cos(Q(6)) -sin(Q(6));
    sin(Q(5))*sin(Q(6)) cos(Q(5))*sin(Q(6)) cos(Q(6))];

Xf = [Q(1) Q(2) Q(3)]';

p = [Q(7)*sin(Q(4)) 0 Q(7)*cos(Q(4))]';
q = [Q(8)*sin(Q(4) - Q(9)) 0 Q(8)*cos(Q(4) - Q(9))]';

X = [Xf;
    R*p + Xf;
    R*q + Xf];
end

