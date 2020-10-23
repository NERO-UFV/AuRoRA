function SaveData(journal,varargin)
% Must use the following header possibilities
% t X Y Z X_d Y_d Z_d X_til Y_til Z_til dx dy dz 
% Phi Theta Psi Phi_d Theta_d Psi_d Phi_til Theta_til Psi_til dphi dTheta dPsi 
% Accx Accy Accz Gyrox Gyroy Gyroz pwm1 pwm2 pwm3 pwm4 
% f_1 f_2 f_3 f_4 f_x f_y f_z

Tempo = varargin{end};

for idx = 1:size(journal,2)
    switch journal{idx}.Typen
        case 1
            if(journal{idx}.TypeS == 1)
                % 1    2     3     4     5    6     7     8     9     10    11    12    13    14    15    16    17    18
                % Batt Pitch Roll  Yaw   Alt  Vx    Vy    Vz    Accx  Accy  Accz  Gyrox Gyroy Gyroz pwm1  pwm2  pwm3  pwm4
                % int  float float float int  float float float float float float float float float uint8 uint8 uint8 uint8
                fprintf(journal{idx}.DataFile,'%6.3f\t ',[varargin{idx}.pPos.X',varargin{idx}.pPos.Xd(1:6)',varargin{idx}.pSC.U',varargin{idx}.pSC.Ud',varargin{idx}.pCom.cRawData', Tempo]); % gravar dados
                fprintf(journal{idx}.DataFile,'\n');
            else
                fprintf(journal{idx}.DataFile, '%6.3f \t ',[varargin{idx}.pPos.X', varargin{idx}.pPos.Xd(1:6)', varargin{idx}.pSC.U', varargin{idx}.pSC.Ud', Tempo]); % gravar dados
                fprintf(journal{idx}.DataFile,'\n');
            end
        case 2
                fprintf(journal{idx}.DataFile, '%6.3f \t ',[varargin{idx}.pPos.X', varargin{idx}.pPos.Xd(1:6)', varargin{idx}.pSC.U', varargin{idx}.pSC.Ud', Tempo]); % gravar dados
                fprintf(journal{idx}.DataFile,'\n');
        case 3
            
    end
end

end





