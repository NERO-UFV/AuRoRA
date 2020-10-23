% This function applies a rotation, given the axis of rotation and the
% angle over some given quaternion
function [result] = quat_rotation(q, n, ang, view)
    % Defining whether the text will be seen
    if nargin == 3
        view = false;
    end
    if view
        view = true;
    else
        view = false;
    end
    
    % Main Function Body:
    norm = sqrt(sum(n.^(2)));
    normalized = n/norm;
    pw = cos(ang/2);
    pi = normalized(1) * sin(ang/2);
    pj = normalized(2) * sin(ang/2);
    pk = normalized(3) * sin(ang/2);
    p = [pw, pi, pj, pk];
    p_conj = [pw, -pi, -pj, -pk];
  
    % Display text or not:
    if view
        fprintf('Rotation Quaternion: ');
        fprintf('%f. ',p);
        fprintf('\n');
        fprintf('Conjugated Rotation Quaternion: ');
        fprintf('%f. ',p_conj);
        fprintf('\n');
    end
    
    % Getting the final results:
    result = quat_mul(p, quat_mul(q, p_conj));
    result = MyQuaternion(result(1), result(2), result(3), result(4));
end

