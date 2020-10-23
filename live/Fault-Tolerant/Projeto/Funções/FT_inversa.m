function X = FT_inversa(Q)
% Q = [x;     1
%      y;     2
%      z;     3
%      phi;   4
%      psi;   5
%      tetha; 6
%      p;     7
%      q;     8
%      beta]; 9
    
X = [Q(1);
    Q(2);
    Q(3);
    Q(1) + Q(7)*sin(Q(4))*cos(Q(5));
    Q(2) + Q(7)*sin(Q(4))*sin(Q(5));
    Q(3) + Q(7)*cos(Q(4));
    Q(1) + Q(8)*sin(Q(4) - Q(9))*cos(Q(5));
    Q(2) + Q(8)*sin(Q(4) - Q(9))*sin(Q(5));
    Q(3) + Q(8)*cos(Q(4) - Q(9))];
end

