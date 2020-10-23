


function mFormationWeightedError(obj)
%MWEIGHTEDERROR Summary of this function goes here
%   Detailed explanation goes here

% Position Errors
xTil = obj.pPos.Qtil(1);
yTil = obj.pPos.Qtil(2);
zTil = obj.pPos.Qtil(3);

% Formation Errors
rTil = obj.pPos.Qtil(4);
bTil = obj.pPos.Qtil(5);
aTil = obj.pPos.Qtil(6);


% Get Norm Coeficient 
%obj.WFEc = (1-abs(rTil)) / (abs(xTil) + abs(yTil) + abs(zTil)-abs(rTil));

obj.WFEc(1) = 0.5/(abs(xTil) + abs(yTil) + abs(zTil)); % Position
obj.WFEc(2) = 0.5/(abs(rTil));                           % Formation   

end

