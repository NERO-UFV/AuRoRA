% This function gets the desired angle and the rotation axis and turn
% them into the equivalent quaternion
function [q] = turn_int_quaternion(theta, n, rad)
    % Defining Default for angles
    if nargin == 2
        rad = false;
    end
    if rad
%         theta = theta;
    else
        theta = theta * pi/180;
    end
    % Beginning Code:
    w = cos(theta);
    if sqrt(sum(n.^(2))) ~= 0
        n = n/sqrt(sum(n.^(2)));
    else
        n = [0, 0, 1];
    end
    n = n * sin(theta);
    q = MyQuaternion(w, n(1), n(2), n(3));
end

