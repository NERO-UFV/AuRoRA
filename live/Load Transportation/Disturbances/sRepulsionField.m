function Repulsion = sRepulsionField(drone1,drone2)
%                        Dist
%       |------------------------------------|
%                   - no force -
%
%                     d_warning
%             |------------------------|
%                    <-  f_min ->
%   ----------                          ----------
%        \                                  /
%         \                                /  
%          \                              /  
%           \          d_threat          /
%            \       |----------|       /
%             \      <-  f_max ->      /
%              \                      /
%               \                    /
%                \                  /
%                 \                /
%                  \              /
%                   \            /
%                    \          /
%                     \        /
%                      \      /
%                       \    /
%                        \  /
%                         \/
%                         O
%

Repulsion = [0 0 0];

dthreat = 0.5;
dwarning = .7;
fmin = 0;
fmax = .02;


% Distância entre drones - Mudar para subtração normal
Dist(1) = norm(drone1.pPos.X(1)-drone2.pPos.X(1));
Dist(2) = norm(drone1.pPos.X(2)-drone2.pPos.X(2));
Dist(3) = norm(drone1.pPos.X(3)-drone2.pPos.X(3));

if Dist(1) > dwarning
    Repulsion(1) = fmin;
elseif Dist(1) > dthreat
    Repulsion(1) = (fmax-fmin)/(dwarning-dthreat)*(dwarning-Dist(1))+fmin;
else
    Repulsion(1) = fmax;
end


end