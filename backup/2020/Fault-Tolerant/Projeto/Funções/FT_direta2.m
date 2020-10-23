function Q = FT_direta2(X)
% X = [x;     1
%      y;     2
%      z;     3
%      x1;    4
%      y1;    5
%      z1;    6
%      x2;    7
%      y2;    8
%      z2];   9

p = sqrt((X(4) - X(1))^2 + (X(5) - X(2))^2 + (X(6) - X(3))^2);
q = sqrt((X(7) - X(1))^2 + (X(8) - X(2))^2 + (X(9) - X(3))^2);
r = sqrt((X(4) - X(7))^2 + (X(5) - X(8))^2 + (X(6) - X(9))^2);

PA1 = [X(4)-X(1) X(5)-X(2) X(6)-X(3)];  %Vetor PA1
PA2 = [X(7)-X(1) X(8)-X(2) X(9)-X(3)];  %Vetor PA2
Normal = cross(PA1,PA2);                %Normal do plano que a formação pertence
VetorZ = [0 0 1];                            %VetorZ (eixo-z)
theta = acos(dot(Normal,VetorZ)/norm(Normal)) - pi/2; %Theta é igual ao angulo entre a Normal e VetorZ menos pi/2

NormalPlanoXY = [Normal(1:2) 0];
VetorX = [1 0 0];
psi = pi/2 - acos(dot(NormalPlanoXY,VetorX)/norm(NormalPlanoXY));



Q = [X(1);
    X(2);
    X(3);
    acos((X(6) - X(3))/(cos(theta)*p));
    psi;
    theta;
    p;
    q;
    acos((p^2 + q^2 - r^2)/(2*p*q))];
end

