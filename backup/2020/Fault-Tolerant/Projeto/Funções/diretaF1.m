function Q = FT_direta(X)
% X = [0;              % x       1
%      0;              % y       2
%      0;              % z       3
%      0;              % phi     4
%      0;              % tetha   5
%      pi/6;           % psi     6
%      pi/4;           % beta    7
%      3;              % p       8
%      3];             % q       9
Q = [X(1);
    X(2);
    X(3);
    sqrt((X(4)-X(1))^2 + (X(5)-X(2))^2 + (X(6)-X(3))^2);
    atan2((X(4)-X(1)),(X(5)-X(2)));
    atan2((X(6)-X(3)),(sqrt((X(4)-X(1))^2 + (X(5)-X(2))^2)))];
end

