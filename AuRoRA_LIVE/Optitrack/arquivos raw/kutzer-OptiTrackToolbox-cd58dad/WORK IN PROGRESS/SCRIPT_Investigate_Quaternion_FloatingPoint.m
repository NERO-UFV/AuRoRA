%% SCRIPT_Investigate_Quaternion_FloatingPoint
for i = 0:360
    H = Rz(deg2rad(i));
    R = H(1:3,1:3);
    q = rotm2quat(R);
    fprintf('%f - %f,%f,%f,%f\n',[i,q(1,1:4)]);
end