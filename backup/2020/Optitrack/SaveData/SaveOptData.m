function SaveOptData(journal,varargin)
% Must use the following header possibilities
% t X Y Z X_d Y_d Z_d X_til Y_til Z_til dx dy dz 
% Phi Theta Psi Phi_d Theta_d Psi_d Phi_til Theta_til Psi_til dphi dTheta dPsi 
% Accx Accy Accz Gyrox Gyroy Gyroz pwm1 pwm2 pwm3 pwm4 
% f_1 f_2 f_3 f_4 f_x f_y f_z

%varargin end -> time
Tempo = varargin{end};


for idx = 1:size(journal,2)
    fprintf(journal{idx}.DataFile, '%6.3f\t ',[varargin{idx}.pPos.X',varargin{idx}.pPos.Xd(1:6)',Tempo]); % x y z Phi Theta Psi dx dy dz dPhi dTheta dPsi % xd yd zd Phid Thetad Psid
    fprintf(journal{idx}.DataFile,'\n');
end





