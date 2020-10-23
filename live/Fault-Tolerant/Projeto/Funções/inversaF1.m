function X = inversaF1(Q)
% Q = [0;              % x       1
%      0;              % y       2
%      0;              % z       3
%      0;              % phi     4
%      0;              % tetha   5
%      pi/6;           % psi     6
%      pi/4;           % beta    7
%      3;              % p       8
%      3];             % q       9
    
X = [Q(1);
    Q(2);
    Q(3);
    Q(1) + Q(8)*sin(Q(4))*cos(Q(6));
    Q(2) + Q(8)*sin(Q(4))*sin(Q(6));
    Q(3) + Q(8)*cos(Q(4));
    Q(1) + Q(9)*sin(Q(4) + Q(7))*cos(Q(6));
    Q(2) + Q(9)*sin(Q(4) + Q(7))*sin(Q(6));
    Q(3) + Q(9)*cos(Q(4) + Q(7))];
end

