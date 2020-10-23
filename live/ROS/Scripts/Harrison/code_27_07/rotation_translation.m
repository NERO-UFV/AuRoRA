% This function gets the desired angle and the rotation axis and turn
% them into the equivalent quaternion
function [final_point] = rotation_translation(point, orientation, trans, text)
    % Defining whether the text will be seen
    if nargin == 3
        text = false;
    end
    if text
        text = true;
    else
        text = false;
    end
    
    % Main Function Body:
    point_quat = point_3d2dual_quaternion(point);
    translation = MyQuaternion(0, trans(1), trans(2), trans(3));
    sigma = rot_trans2dual_quat(orientation, translation);
    result = dual_quat_mul(sigma, point_quat);
    aux = sigma.conjugate_3();
    sigma_conjugate_3 = MyDualQuaternion(aux(1:4), aux(5:8));
    result = dual_quat_mul(result, sigma_conjugate_3);
    aux_result = result.dual_quaternion();
    final_point = [aux_result(6), aux_result(7), aux_result(8)];
    
    % Display text or not:
    if text
        fprintf('Initial Point: ');
        fprintf('%f. ',point);
        fprintf('\n');
        angle = 2 * orientation.arg() * (180/pi);
        fprintf('Angle Rotated: ');
        fprintf('%f. ',angle);
        fprintf('\n');
        fprintf('Axis Rotated Over: ');
        fprintf('%f. ',orientation.axis_of_rotation());
        fprintf('\n');
        fprintf('Translation Desired: ');
        fprintf('%f. ',trans);
        fprintf('\n');
        fprintf('Final Point: ');
        fprintf('%f. ',final_point);
    end
end

