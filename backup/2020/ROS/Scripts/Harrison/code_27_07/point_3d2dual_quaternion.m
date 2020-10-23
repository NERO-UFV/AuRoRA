% This function transforms a 3D point into its equivalent in
% dual-quaternion
function [sigma] = point_3d2dual_quaternion(point)
    point_quat = [0, point(1), point(2), point(3)];
    no_rotation = [1, 0, 0, 0];
    sigma = MyDualQuaternion(no_rotation, point_quat);
end
