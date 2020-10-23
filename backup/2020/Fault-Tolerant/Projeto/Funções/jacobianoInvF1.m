function X = jacobianoInvF1(Q,dQr)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
J_inv = [1 0 0 0                   0                         0;
         0 1 0 0                   0                         0;
         0 0 1 0                   0                         0;
         1 0 0 sin(Q(5))*cos(Q(6)) Q(4)*cos(Q(5))*cos(Q(6))  -Q(4)*sin(Q(5))*sin(Q(6));
         0 1 0 cos(Q(5))*cos(Q(6)) -Q(4)*sin(Q(5))*cos(Q(6)) -Q(4)*cos(Q(5))*sin(Q(6));
         0 0 1 sin(Q(6))           0                         Q(4)*cos(Q(5))];
X = J_inv*dQr;
end


