classdef DroneMilton_Trajetoria < handle
    properties
        tmrCount;
        
        parametros
         
        robot_acelx
        robot_acely
        robot_acelz
        robot_acelpsi
        robot_ux
        robot_uy
        robot_uz
        robot_upsi
        
        robot_vx
        robot_vy
        robot_vz
        robot_vpsi
        
        robot_vxb
        robot_vyb
        robot_vzb
        robot_vpsib
        
        robot_x
        robot_y
        robot_z
        robot_psi
        
        robot_xd
        robot_yd
        robot_zd
        robot_psid
        
        robot_vxd
        robot_vyd
        robot_vzd
        robot_vpsid
        
        switched_sigma
        switched_C1
        switched_C2
        switched_C3
        x_filt_Glob
        P_filt_Glob
        
        x_filt_Loc1
        x_filt_Loc2
        x_filt_Loc3
        
        P_filt_Loc1
        P_filt_Loc2
        P_filt_Loc3
        
        % DIF
        P_ant
        Y_ant
        y_ant
        x_ant
        
        fusion_rho_x;
        fusion_rho_y;
        fusion_rho_z;
        fusion_rho_psi;
        
        color, controlador
        
         path_xd
         path_yd
         path_zd
         path_psid
         
         w
         dist
         rho
         alpha
         beta
         
         path_id
         path_checked
         checked_waypoint      
         
         Uc_ant
         Uc
         dUc
         
         k1, k2, k3, k4, k5, k6, k7, k8
         k1r, k2r, k3r, k4r, k5r, k6r, k7r, k8r
         
         %Controle Adaptable
         ativar_adaptable
         CA_Thetar_initial
         CA_Theta_initial
         CA_Theta
         CA_Theta_ant
         CA_dTheta
         CA_G
         CA_vError
         CA_dx_ref
         CA_K
         
         CA_Ud1
         CA_Ud2
         
         Vc_ant
         
        control_Uc
        control_Vc
        control_Accelc
        control_sc
        control_s
        control_vsc
        control_Ud
        
        control_Kc
        control_Vmax
        control_Vxd_maxTraj
        
        K1
        K2
        K
    end
    properties(Constant)
        T = 0.033;
        
        %k1 = 3.00, k2 = 0.2
        %k3 = 3.20, k4 = 0.1
        %k5 = 0.55, k6 = 0.7
        %k7 = 1.45, k8 = 0.7

        
        %Filtro DKF, DIF
        n = 4 %estados
        m = 2 %observacoes                
    end
    methods
        function DM = DroneMilton_Trajetoria                                  
%             DM.k1 =    4.72;
%             DM.k2 =    0.2766;
%             DM.k3 =    6.233;
%             DM.k4 =    0.53;
%             DM.k5 =    2.6504;
%             DM.k6 =    2.576;
%             DM.k7 =    2.3788;
%             DM.k8 =    1.5216;
            DM.w = 0.9;
            DM.dist = 1.75;
            
            DM.K1 = 1.2*[ 1   0   0   0
                        0   1   0   0
                        0   0   1   0
                        0   0   0   1];
              
            DM.K2 = 1.2*[ 1   0   0   0
                        0   1   0   0
                        0   0   1   0
                        0   0   0   1];
             
              
            DM.K = 1.2*[ 1   0   0   0
                        0   1   0   0
                        0   0   1   0
                        0   0   0   1];
            
            DM.ativar_adaptable = 1;

            DM.k1 =    1;
            DM.k2 =    1;
            DM.k3 =    1;
            DM.k4 =    1;
            DM.k5 =    1;
            DM.k6 =    1;
            DM.k7 =    1;
            DM.k8 =    1;
            
             DM.CA_Theta_initial = [DM.k1^-1
                                  (DM.k1^-1)*DM.k2
                                   DM.k3^-1
                                  (DM.k3^-1)*DM.k4
                                   DM.k5^-1
                                  (DM.k5^-1)*DM.k6
                                   DM.k7^-1
                                  (DM.k7^-1)*DM.k8];  
            
            DM.k1r =    14.72;
            DM.k2r =    0.2766;
            DM.k3r =    6.233;
            DM.k4r =    0.53;
            DM.k5r =    2.6504;
            DM.k6r =    2.576;
            DM.k7r =    2.3788;
            DM.k8r =    1.5216;
            
           DM.CA_Thetar_initial = [DM.k1r^-1
                                  (DM.k1r^-1)*DM.k2r
                                   DM.k3r^-1
                                  (DM.k3r^-1)*DM.k4r
                                   DM.k5r^-1
                                  (DM.k5r^-1)*DM.k6r
                                   DM.k7r^-1
                                  (DM.k7r^-1)*DM.k8r];  
            %%Controle adaptable
                                       
            
            DM.CA_Theta_ant = DM.CA_Theta_initial;                              
            DM.CA_Theta = ones(8,1);
            %DM.CA_Theta_ant = zeros(8,1);
            DM.CA_dTheta = zeros(8,1);              
            DM.CA_G = zeros(4,8);
            
            DM.CA_Ud1 = zeros(4,1);
            DM.CA_Ud2 = zeros(4,1);
            %%
                        
            DM.tmrCount = 1;
            DM.parametros{DM.tmrCount} = zeros(8,1);
            DM.robot_acelx(DM.tmrCount) = [0];
            DM.robot_acely(DM.tmrCount) = [0];
            DM.robot_acelz(DM.tmrCount) = [0];
            DM.robot_acelpsi(DM.tmrCount) = [0];
            
            DM.control_Uc{DM.tmrCount} = zeros(4,1);
            DM.control_Vc{DM.tmrCount} = zeros(4,1);
            DM.control_Accelc{DM.tmrCount} = zeros(4,1);
            DM.control_sc{DM.tmrCount} = zeros(4,1);
            DM.control_s{DM.tmrCount} = zeros(4,1);
            DM.control_vsc{DM.tmrCount} = zeros(4,1);
            DM.control_Ud{DM.tmrCount} = zeros(4,1);
            
            DM.robot_ux(DM.tmrCount) = [0];
            DM.robot_uy(DM.tmrCount) = [0];
            DM.robot_uz(DM.tmrCount) = [0];
            DM.robot_upsi(DM.tmrCount) = [0];
            
            DM.robot_vx(DM.tmrCount) = [0];
            DM.robot_vy(DM.tmrCount) = [0];
            DM.robot_vz(DM.tmrCount) = [0];
            DM.robot_vpsi(DM.tmrCount) = [0];
            
            DM.robot_vxb(DM.tmrCount) = [0];
            DM.robot_vyb(DM.tmrCount) = [0];
            DM.robot_vzb(DM.tmrCount) = [0];
            DM.robot_vpsib(DM.tmrCount) = [0];
            
            DM.robot_x(DM.tmrCount) = [2];
            DM.robot_y(DM.tmrCount) = [1];
            DM.robot_z(DM.tmrCount) = [0];
            DM.robot_psi(DM.tmrCount) = [pi/2];
            
            DM.robot_xd = [0];
            DM.robot_yd = [0];
            DM.robot_zd = [0];
            DM.robot_psid = [0];
            
            DM.robot_vxd = [0];
            DM.robot_vyd = [0];
            DM.robot_vzd = [0];
            DM.robot_vpsid = [0];
            
            %DKF
            DM.x_filt_Glob = zeros(DM.n,1)
            DM.P_filt_Glob = zeros(DM.n,DM.n)
            
            DM.x_filt_Loc1 = zeros(DM.n,1)
            DM.x_filt_Loc2 = zeros(DM.n,1)
            DM.x_filt_Loc3 = zeros(DM.n,1)
            
            DM.P_filt_Loc1 = zeros(DM.n,DM.n)
            DM.P_filt_Loc2 = zeros(DM.n,DM.n)
            DM.P_filt_Loc3 = zeros(DM.n,DM.n)
            
            % DIF
            DM.P_ant = 0.000001*eye(DM.n)
            DM.Y_ant = 0.000001*eye(DM.n) %Matriz de Informacoes Y
            DM.y_ant = zeros(DM.n,1)
            DM.x_ant = zeros(DM.n,1)            
            
            %ccaminho
            DM.alpha = 0;
            DM.beta = 0;
            DM.rho = 0;
            
            DM.Uc_ant = zeros(4,1);
            DM.Uc = zeros(4,1);
            DM.Vc_ant = 0;
            
            DM.path_id(DM.tmrCount) = 1;
            DM.path_checked(DM.tmrCount) = 0;
            DM.checked_waypoint = 0;
        end
        %      function tmrCount = get.tmrCount(obj)
        %          tmrCount = obj.tmrCount;
        %      end
        function A(DM)%CalculaAcoesControle(DM)
            DM.tmrCount = DM.tmrCount + 1;
            t = DM.tmrCount*DM.T
        end
        function pos = getPosicoes(obj)
            pos.x = obj.robot_x(obj.tmrCount);
            pos.y = obj.robot_y(obj.tmrCount);
            pos.z = obj.robot_z(obj.tmrCount);
            pos.psi = obj.robot_psi(obj.tmrCount);
        end   
          function a = SetEstadoInicial(DM, xr, yr, zr, psir, vxr, vyr, vzr, vpsir, acx, acy, acz, acpsi)
            DM.tmrCount
            DM.robot_acelx(DM.tmrCount) = acx;
            DM.robot_acely(DM.tmrCount) = acy;
            DM.robot_acelz(DM.tmrCount) = acz;
            DM.robot_acelpsi(DM.tmrCount) = acpsi;
                        
            DM.robot_vx(DM.tmrCount) = vxr;
            DM.robot_vy(DM.tmrCount) = vyr;
            DM.robot_vz(DM.tmrCount) = vzr;
            DM.robot_vpsi(DM.tmrCount) = vpsir;
                       
            DM.robot_x(DM.tmrCount) = xr;
            DM.robot_y(DM.tmrCount) = yr;
            DM.robot_z(DM.tmrCount) = zr;
            DM.robot_psi(DM.tmrCount) = psir;
            
        end
          function a = SetModelo(DM, k1, k2, k3, k4, k5, k6, k7, k8)
            DM.k1r =    k1;
            DM.k2r =    k2;
            DM.k3r =    k3;
            DM.k4r =    k4;
            DM.k5r =    k5;
            DM.k6r =    k6;
            DM.k7r =    k7;
            DM.k8r =    k8;            
          end
         function a = ModeloIdeal(DM)            
            DM.k1 =    DM.k1r;
            DM.k2 =    DM.k2r;
            DM.k3 =    DM.k3r;
            DM.k4 =    DM.k4r;
            DM.k5 =    DM.k5r;
            DM.k6 =    DM.k6r;
            DM.k7 =    DM.k7r;
            DM.k8 =    DM.k8r;   
            
            DM.CA_Thetar_initial = [DM.k1r^-1
                                  (DM.k1r^-1)*DM.k2r
                                   DM.k3r^-1
                                  (DM.k3r^-1)*DM.k4r
                                   DM.k5r^-1
                                  (DM.k5r^-1)*DM.k6r
                                   DM.k7r^-1
                                  (DM.k7r^-1)*DM.k8r];              
            DM.CA_Theta = [DM.k1r^-1
                                  (DM.k1r^-1)*DM.k2r
                                   DM.k3r^-1
                                  (DM.k3r^-1)*DM.k4r
                                   DM.k5r^-1
                                  (DM.k5r^-1)*DM.k6r
                                   DM.k7r^-1
                                  (DM.k7r^-1)*DM.k8r];  
                DM.CA_Theta_ant = DM.CA_Theta;    
                DM.CA_Theta_initial = DM.CA_Theta;                              
         end
         function a = MudaTrajetoria(DM, w, dist)             
             DM.w = w;
             DM.dist = dist;
         end                         
         function a = MudaGanhos(DM, k1, k2, k)             
             DM.K1 = k1*DM.K1;
             DM.K2 = k2*DM.K2;
             DM.K =  k*DM.K;
         end                
        function a = MudaModelo(DM)            
            DM.k1r =    DM.k1r + 2*5.15;
            DM.k2r =    DM.k2r + 2*3.15;
            DM.k3r =    DM.k3r + 2*5.15;
            DM.k4r =    DM.k4r + 2*3.15;
            DM.k5r =    DM.k5r + 2*5.15;
            DM.k6r =    DM.k6r + 2*2.15;
            DM.k7r =    DM.k7r + 2*5.15;
            DM.k8r =    DM.k8r + 2*2.15;   
            
            DM.CA_Thetar_initial = [DM.k1r^-1
                                  (DM.k1r^-1)*DM.k2r
                                   DM.k3r^-1
                                  (DM.k3r^-1)*DM.k4r
                                   DM.k5r^-1
                                  (DM.k5r^-1)*DM.k6r
                                   DM.k7r^-1
                                  (DM.k7r^-1)*DM.k8r];  
        end
        function DM = EnviaAcoesManual(DM)
            DM.tmrCount = DM.tmrCount +1;
        end
        function ExecutaAcoesControle(DM,ux,uy,uz,upsi)            
            %DM.tmrCount = DM.tmrCount + 1;            
            T = DM.T;            
            Uc = [ux; uy; uz; upsi];
            
            %Aceleração do robô simulado com a ação de controle calculada (Uc)
            id = find(Uc > 1);
            idx = find(Uc < -1);
            %Normaliza
            try
                Uc(id) = 1;
                Uc(idx) = -1;
            end
            
            DM.tmrCount
            psi = DM.robot_psi(DM.tmrCount-1);
            F1r = [DM.k1r*cos(psi)  -DM.k3r*sin(psi)    0   0;
                DM.k1r*sin(psi) DM.k3r*cos(psi)     0   0;
                0           0               DM.k5r  0;
                0           0               0   DM.k7r];
            
            F2r = [DM.k2r*cos(psi)   -DM.k4r*sin(psi)    0   0;
                DM.k2r*sin(psi)    DM.k4r*cos(psi)     0   0;
                0              0               DM.k6r  0;
                0              0               0   DM.k8r];
            
            V = [DM.robot_vx(DM.tmrCount-1);
                DM.robot_vy(DM.tmrCount-1);
                DM.robot_vz(DM.tmrCount-1);
                DM.robot_vpsi(DM.tmrCount-1)];            
            
            F = [cos(psi)   -sin(psi)   0   0;
                 sin(psi)   cos(psi)    0   0;
                 0          0           1   0;
                 0          0           0   1];
             
            
            
            %Acell = F1*Uc - F2*Vb;  Tá no referencial do robo, aqui e com
            %o filtro de fusao só trabalho no espaço do mundo Vw, logo Vw = R.Vb ou F.Vb         
            Acell = F1r*Uc - F2r*inv(F)*V;  
            
            DM.robot_acelx(DM.tmrCount) = Acell(1);
            DM.robot_acely(DM.tmrCount) = Acell(2);
            DM.robot_acelz(DM.tmrCount) = Acell(3);
            DM.robot_acelpsi(DM.tmrCount) = Acell(4);
            
            %Obtendo Velocidades e Posicao integrando a aceleracao
            DM.robot_vx(DM.tmrCount) = DM.robot_vx(DM.tmrCount-1) + DM.robot_acelx(DM.tmrCount)*T +0.0*0.1*rand();
            DM.robot_vy(DM.tmrCount)= DM.robot_vy(DM.tmrCount-1) + DM.robot_acely(DM.tmrCount)*T+0.0*0.1*rand();
            DM.robot_vz(DM.tmrCount)= DM.robot_vz(DM.tmrCount-1) + DM.robot_acelz(DM.tmrCount)*T+0.0*0.1*rand();
            DM.robot_vpsi(DM.tmrCount)= DM.robot_vpsi(DM.tmrCount-1) + DM.robot_acelpsi(DM.tmrCount)*T+0.0*0.1*rand();
            
            Vb = inv(F)*[DM.robot_vx(DM.tmrCount);
                         DM.robot_vy(DM.tmrCount);
                         DM.robot_vz(DM.tmrCount);
                         DM.robot_vpsi(DM.tmrCount)];
            
            DM.robot_vxb(DM.tmrCount) = Vb(1);
            DM.robot_vyb(DM.tmrCount)= Vb(2);
            DM.robot_vzb(DM.tmrCount)= Vb(3);
            DM.robot_vpsib(DM.tmrCount)= Vb(4);
            
            DM.robot_x(DM.tmrCount) = DM.robot_x(DM.tmrCount-1) + DM.robot_vx(DM.tmrCount)*T;
            DM.robot_y(DM.tmrCount) = DM.robot_y(DM.tmrCount-1) + DM.robot_vy(DM.tmrCount)*T;
            DM.robot_z(DM.tmrCount) = DM.robot_z(DM.tmrCount-1) + DM.robot_vz(DM.tmrCount)*T;
            DM.robot_psi(DM.tmrCount) = DM.robot_psi(DM.tmrCount-1) + DM.robot_vpsi(DM.tmrCount)*T;
        end                      
        function dTheta = ZeradTheta(DM)
              DM.CA_dTheta = zeros(8,1);
              DM.ativar_adaptable = 0;
        end
        function dTheta = CalculadTheta(DM)
            %DM_CA.vError = DM.Uc - [DM.robot_vx(end); DM.robot_vy(end); DM.robot_vz(end); DM.robot_vpsi(end)];
            DM.CA_vError = DM.CA_dx_ref - [DM.robot_vx(end); DM.robot_vy(end); DM.robot_vz(end); DM.robot_vpsi(end)];
            %DM_CA.vError = [DM.robot_ux(end); DM.robot_uy(end);DM.robot_uz(end);DM.robot_upsi(end)] - [DM.robot_vx(end); DM.robot_vy(end); DM.robot_vz(end); DM.robot_vpsi(end)];
             
            %              1   2    3    4    5    6    7    8
            gamma = [     13,  0,   0,   0,   0,   0,    0,   0;
                          0,  2,   0,    0,   0,   0,    0,   0;
                          0,  0,   12.4, 0,   0,   0,    0,   0;
                          0,  0,   0,    2.3, 0,   0,    0,   0;
                          0,  0,   0,    0,   0.1, 0,    0,   0;
                          0,  0,   0,    0,   0,   0.8,  0,   0;
                          0,  0,   0,    0,   0,   0,    0.1, 0;
                          0,  0,   0,    0,   0,   0,    0,   0.5];
			gamma = 1.0*gamma;

           %              1   2    3    4    5      6    7    8
			gamma2 = [    1,  0,   0,   0,   0,     0,     0,   0;
                          0,  1,   0,   0,   0,     0,     0,   0;
                          0,  0,   0.9, 0,   0,     0,     0,   0;
                          0,  0,   0,   1.3, 0,     0,     0,   0;
                          0,  0,   0,   0,   0.65,  0,     0,   0;
                          0,  0,   0,   0,   0,    1.2501, 0,   0;
                          0,  0,   0,   0,   0,     0,     1.8504,   0;
                          0,  0,   0,   0,   0,     0,     0,   3.84];
			gamma2 = 3.5*gamma2;
%                          1     2      3        4         5       6          7    8                      
            Gamma = [	0.008,  0,      0,       0,       0,       0,         0,   0;
                          0,    0.008, 0,       0,       0,       0,         0,   0;
                          0,    0,      0.008,   0,       0,       0,         0,   0;
                          0,    0,      0,       0.008,  0,       0,         0,   0;
                          0,    0,      0,       0,       0.0091,  0,         0,   0;
                          0,    0,      0,       0,       0,       0.000515,  0,   0;
                          0,    0,      0,       0,       0,       0,         0.02,  0;
                          0,    0,      0,       0,       0,       0,         0,   0.002];		
			
			Gamma = 0.1*Gamma;
            
            %DM.CA_dTheta = (gamma^-1)*DM.CA_G'*DM.CA_vError - (gamma2^-1)*Gamma*DM.CA_Theta;                        
            DM.CA_dTheta = (gamma^-1)*DM.CA_G'*DM.CA_vError - 0*(gamma2^-1)*Gamma*DM.CA_Theta;
            
        end
        function Ud = CalculaAcoesAdaptable(DM)
             DM.tmrCount = DM.tmrCount + 1;
             t = DM.tmrCount*DM.T;
             T = DM.T;            
                       
             %w = 0.5;
             %dist = 2.75;
             
             w = DM.w;
             dist = DM.dist;
            
             xd = dist*sin(w*t) +0.5;
             yd = 1.25*dist*cos(0.5*w*t) -0.5;
             zd = 2 + 0.5*sin(w*t);
             psid = (pi/6)*sin(w*t);            
             %%            
             vxd = w*dist*cos(w*t);
             vyd = -1.25*0.5*w*dist*sin(0.5*w*t);
             vzd = 0.5*w*cos(w*t);
             vpsid = w*(pi/6)*cos(w*t);
                                    
            DM.robot_xd(DM.tmrCount) = xd;
            DM.robot_yd(DM.tmrCount) = yd;
            DM.robot_zd(DM.tmrCount) = zd;
            DM.robot_psid(DM.tmrCount) = psid;
            
            DM.robot_vxd(DM.tmrCount) = vxd;
            DM.robot_vyd(DM.tmrCount) = vyd;
            DM.robot_vzd(DM.tmrCount) = vzd;
            DM.robot_vpsid(DM.tmrCount) = vpsid;
                   
            psi = DM.robot_psi(DM.tmrCount-1);
            
            F = [cos(psi)   -sin(psi)   0   0;
                 sin(psi)   cos(psi)    0   0;
                 0          0           1   0;
                 0          0           0   1];                        
                           
            V = [DM.robot_vx(DM.tmrCount-1);
                DM.robot_vy(DM.tmrCount-1);
                DM.robot_vz(DM.tmrCount-1);
                DM.robot_vpsi(DM.tmrCount-1)];
                       
            
            Error = [xd-DM.robot_x(DM.tmrCount-1);
                yd-DM.robot_y(DM.tmrCount-1);
                zd-DM.robot_z(DM.tmrCount-1);
                psid-DM.robot_psi(DM.tmrCount-1);
                ];
                        
            %velocidade de referencia no caminho
            %vmax = 0.735; %m/s
            %k = 1.85;
            %wref = 0;
            %vref = vmax;%/(1+k*DM.rho(DM.tmrCount));                                    
            
            Vref = [vxd; vyd; vzd; vpsid];                                       
            
            v = Vref + DM.K1*tanh(DM.K2*Error);      
            DM.CA_dx_ref = v;
            %Controle cinematico  
            
            Uc = inv(F)*(v);
            DM.Uc = Uc;
            
            %Controle Dinamico
            dUc = (DM.Uc - DM.Uc_ant)/DM.T;
            Vb = inv(F)*V;
            
            DM.dUc = dUc;            
            %% Calcula o valor de theta chapeu            
            %G = fcnCalculaG(F*[dUc(1); dUc(2); dUc(3); dUc(4)],F*[Uc(1); Uc(2); Uc(3); Uc(4)], psi);
                       
            k1c = DM.K(1,1);
            k2c = DM.K(2,2);
            k3c = DM.K(3,3);
            k4c = DM.K(4,4);
         
                                
            G = fcnCalculaG(F*[dUc(1); dUc(2); dUc(3); dUc(4)], F*[DM.Uc_ant(1); DM.Uc_ant(2); DM.Uc_ant(3); DM.Uc_ant(4)], F*[DM.Uc_ant(1); DM.Uc_ant(2); DM.Uc_ant(3); DM.Uc_ant(4)] - [DM.robot_vx(end); DM.robot_vy(end); DM.robot_vz(end); DM.robot_vpsi(end)], [DM.robot_vx(end); DM.robot_vy(end); DM.robot_vz(end); DM.robot_vpsi(end)],[k1c; k2c; k3c; k4c], psi);            
            %fcnCalculaG(Mat1f Accelw, Mat1f Vrefw, Mat1f ErrorVelocidades, Mat1f Velocidades, Mat1f K, double psi);
            DM.CA_G = G;              
            DM.CalculadTheta;
                        
            
                
            if(DM.ativar_adaptable == 0)   
                DM.CA_dTheta = zeros(8,1);
            end
            
                if (norm(DM.CA_dTheta) == 0)
                    DM.CA_Theta = DM.CA_Theta_ant;    
                else            
                    DM.CA_Theta = DM.CA_dTheta*DM.T + DM.CA_Theta_ant;                    
                end        
            
            
                            
            %Ud = G*DM.CA_Theta;                                    
            
            %Atualizando os parametros
            DM.k1 = 1/DM.CA_Theta(1);
            DM.k2 = DM.CA_Theta(2)/DM.CA_Theta(1);
            DM.k3 = 1/DM.CA_Theta(3);
            DM.k4 = DM.CA_Theta(4)/DM.CA_Theta(3);
            DM.k5 = 1/DM.CA_Theta(5);
            DM.k6 = DM.CA_Theta(6)/DM.CA_Theta(5);
            DM.k7 = 1/DM.CA_Theta(7);
            DM.k8 = DM.CA_Theta(8)/DM.CA_Theta(7);                 
            
            DM.CA_K = [DM.k1; DM.k2; DM.k3; DM.k4; DM.k5; DM.k6; DM.k7; DM.k8];
            %%            
            
            F1 = [DM.k1*cos(psi)  -DM.k3*sin(psi)    0   0;
                DM.k1*sin(psi) DM.k3*cos(psi)     0   0;
                0           0               DM.k5  0;
                0           0               0   DM.k7];
            
            F2 = [DM.k2*cos(psi)   -DM.k4*sin(psi)    0   0;
                DM.k2*sin(psi)    DM.k4*cos(psi)     0   0;
                0              0               DM.k6  0;
                0              0               0   DM.k8];
           
                                    
            %Ud = inv(F1)*(K*(F*Uc - V) + dUc - (-F2)*Vb);            
            
            %Ud = inv(F1)*(K*(F*Uc -V) + dUc - (-F2)*(inv(F)*(V))) + G*(DM.CA_Theta - DM.CA_Theta_initial);
            %Ud = inv(F1)*(K*(F*Uc -V) + dUc - (-F2)*(inv(F)*(V)));
            %Ud = DM.CA_G*DM.CA_Theta;
            
            DM.CA_Ud1 = inv(F1)*(DM.K*(F*Uc -V) + dUc - (-F2)*(inv(F)*(V)));
            DM.CA_Ud2 = DM.CA_G*DM.CA_Theta;
            
            Ud = DM.CA_Ud2;
            
            DM.CA_Theta_ant = DM.CA_Theta;
            %Vb velocidade do drone no seu referencia
            %dUc derivada das acoes de controle (aceleracao desejada)
            
                                                		
            %Aceleração do robô simulado com a ação de controle calculada (Uc)
            id1 = find(Ud > 1);
            id2 = find(Ud < -1);
            
            try
                Ud(id1) = 1;
                Ud(id2) = -1;
            end
                      
            DM.robot_ux(DM.tmrCount) = Ud(1);
            DM.robot_uy(DM.tmrCount) = Ud(2);
            DM.robot_uz(DM.tmrCount) = Ud(3);
            DM.robot_upsi(DM.tmrCount) = Ud(4);
                        
          
            DM.Uc_ant = DM.Uc;
        end 
        function Ud = CalculaAcoesAdaptableCinematico(DM)
             DM.tmrCount = DM.tmrCount + 1;
             t = DM.tmrCount*DM.T;
             T = DM.T;            
                       
             w = DM.w;
             dist = DM.dist;
            
             xd = dist*sin(w*t) +0.5;
             yd = 1.25*dist*cos(0.5*w*t) -0.5;
             zd = 2 + 0.5*sin(w*t);
             psid = (pi/6)*sin(w*t);            
             %%            
             vxd = w*dist*cos(w*t);
             vyd = -1.25*0.5*w*dist*sin(0.5*w*t);
             vzd = 0.5*w*cos(w*t);
             vpsid = w*(pi/6)*cos(w*t);
                                    
            DM.robot_xd(DM.tmrCount) = xd;
            DM.robot_yd(DM.tmrCount) = yd;
            DM.robot_zd(DM.tmrCount) = zd;
            DM.robot_psid(DM.tmrCount) = psid;
            
            DM.robot_vxd(DM.tmrCount) = vxd;
            DM.robot_vyd(DM.tmrCount) = vyd;
            DM.robot_vzd(DM.tmrCount) = vzd;
            DM.robot_vpsid(DM.tmrCount) = vpsid;
                   
            psi = DM.robot_psi(DM.tmrCount-1);
            
            F = [cos(psi)   -sin(psi)   0   0;
                 sin(psi)   cos(psi)    0   0;
                 0          0           1   0;
                 0          0           0   1];                        
                           
            V = [DM.robot_vx(DM.tmrCount-1);
                DM.robot_vy(DM.tmrCount-1);
                DM.robot_vz(DM.tmrCount-1);
                DM.robot_vpsi(DM.tmrCount-1)];
                       
            
            Error = [xd-DM.robot_x(DM.tmrCount-1);
                yd-DM.robot_y(DM.tmrCount-1);
                zd-DM.robot_z(DM.tmrCount-1);
                psid-DM.robot_psi(DM.tmrCount-1);
                ];
                        
            %velocidade de referencia no caminho
            %vmax = 0.735; %m/s
            %k = 1.85;
            %wref = 0;
            %vref = vmax;%/(1+k*DM.rho(DM.tmrCount));                                    
            
            Vref = [vxd; vyd; vzd; vpsid]; 
            
               
            
            v = Vref + DM.K1*tanh(DM.K2*Error);      
            DM.CA_dx_ref = v;
            %Controle cinematico  
            
            Uc = inv(F)*(v);
            DM.Uc = Uc;
            
            Ud = Uc;
            
            DM.CA_Theta_ant = DM.CA_Theta;
            %Vb velocidade do drone no seu referencia
            %dUc derivada das acoes de controle (aceleracao desejada)
            
                                                		
            %Aceleração do robô simulado com a ação de controle calculada (Uc)
            id1 = find(Ud > 1);
            id2 = find(Ud < -1);
            
            try
                Ud(id1) = 1;
                Ud(id2) = -1;
            end
                      
            DM.robot_ux(DM.tmrCount) = Ud(1);
            DM.robot_uy(DM.tmrCount) = Ud(2);
            DM.robot_uz(DM.tmrCount) = Ud(3);
            DM.robot_upsi(DM.tmrCount) = Ud(4);
                                  
            DM.Uc_ant = DM.Uc;
        end 
         function Ud = CalculaAcoesControleClaudio(DM)
             DM.tmrCount = DM.tmrCount + 1;
            t = DM.tmrCount*DM.T;
            T = DM.T;            
                       
             w = 0.3;
             dist = 1.75;
            
             xd = dist*sin(w*t) +0.1;
             yd = 1.25*dist*cos(0.5*w*t) -0.2;
             zd = 1 + 0.5*sin(w*t);
             psid = (pi/6)*sin(w*t);            
             %%
            
             vxd = w*dist*cos(w*t);
             vyd = -1.25*0.5*w*dist*sin(0.5*w*t);
             vzd = 0.5*w*cos(w*t);
             vpsid = w*(pi/6)*cos(w*t);
             
                                    
            DM.robot_xd(DM.tmrCount) = xd;
            DM.robot_yd(DM.tmrCount) = yd;
            DM.robot_zd(DM.tmrCount) = zd;
            DM.robot_psid(DM.tmrCount) = psid;
            
            DM.robot_vxd(DM.tmrCount) = vxd;
            DM.robot_vyd(DM.tmrCount) = vyd;
            DM.robot_vzd(DM.tmrCount) = vzd;
            DM.robot_vpsid(DM.tmrCount) = vpsid;
            
            
            if(DM.robot_xd(DM.tmrCount-1) - DM.robot_xd(DM.tmrCount) ~= 0 || DM.robot_yd(DM.tmrCount-1) - DM.robot_yd(DM.tmrCount) ~= 0 || DM.robot_zd(DM.tmrCount-1) - DM.robot_zd(DM.tmrCount) ~= 0 || DM.robot_psid(DM.tmrCount-1) - DM.robot_psid(DM.tmrCount) ~= 0)
                change_desiredPosition = 1;
            end
            
            psi = DM.robot_psi(DM.tmrCount-1);
            
            F = [cos(psi)   -sin(psi)   0   0;
                 sin(psi)   cos(psi)    0   0;
                 0          0           1   0;
                 0          0           0   1];
            
            F1 = [DM.k1*cos(psi)  -DM.k3*sin(psi)    0   0;
                DM.k1*sin(psi) DM.k3*cos(psi)     0   0;
                0           0               DM.k5  0;
                0           0               0   DM.k7];
            
            F2 = [DM.k2*cos(psi)   -DM.k4*sin(psi)    0   0;
                DM.k2*sin(psi)    DM.k4*cos(psi)     0   0;
                0              0               DM.k6  0;
                0              0               0   DM.k8];
            

            V = [DM.robot_vx(DM.tmrCount-1);
                DM.robot_vy(DM.tmrCount-1);
                DM.robot_vz(DM.tmrCount-1);
                DM.robot_vpsi(DM.tmrCount-1)];
            
           
            
            Error = [xd-DM.robot_x(DM.tmrCount-1);
                yd-DM.robot_y(DM.tmrCount-1);
                zd-DM.robot_z(DM.tmrCount-1);
                psid-DM.robot_psi(DM.tmrCount-1);
                ];               
                        
            %velocidade de referencia no caminho
            %vmax = 0.735; %m/s
            %k = 1.85;
            %wref = 0;
            %vref = vmax;%/(1+k*DM.rho(DM.tmrCount));                                    
            
            Vref = [vxd; vyd; vzd; vpsid]; 
            
            K1 = [1.2 0     0   0
                  0   1.2   0   0
                  0   0     1.2 0
                  0   0     0   1.2];
              
              K2 = [1.2 0     0   0
                  0   1.2   0   0
                  0   0     1.2 0
                  0   0     0   1.2];
            
            %v = Vref + K1*tanh(K2*Error);      
            v = Vref + K1*tanh(K2*Error);      
            %Controle cinematico  
            
            Uc = inv(F)*(v);
            DM.Uc = Uc;
            
            %Controle Dinamico
            dUc = (DM.Uc - DM.Uc_ant)/DM.T;
            Vb = inv(F)*V;
            
            k1c = 1.2;
            k2c = 1.2;
            k3c = 1.2;
            k4c = 1.2;
            K = [k1c,  0, 0, 0;
                    0,k2c, 0, 0;
                    0,	0,k3c, 0;
                    0,	0,	0,	k4c];

            
            %Ud = inv(F1)*(K*(F*Uc - V) + dUc - (-F2)*Vb);            
            Ud = inv(F1)*(dUc + K*(F*Uc - V) - (-F2)*(inv(F)*(V)));
            
            %Vb velocidade do drone no seu referencia
            %dUc derivada das acoes de controle (aceleracao desejada)
            %
                                                		
            %Aceleração do robô simulado com a ação de controle calculada (Uc)
            %id1 = find(Ud > 1);
            %id2 = find(Ud < -1);
            
            %try
             %   Ud(id1) = 1;
              %  Ud(id2) = -1;
            %end
                      
            DM.robot_ux(DM.tmrCount) = Ud(1);
            DM.robot_uy(DM.tmrCount) = Ud(2);
            DM.robot_uz(DM.tmrCount) = Ud(3);
            DM.robot_upsi(DM.tmrCount) = Ud(4);
                        
          
            DM.Uc_ant = DM.Uc;
        end 
        function Ud = CalculaAcoesControleClaudio_SaturandoVd(DM)
            DM.tmrCount = DM.tmrCount + 1;
            t = DM.tmrCount*DM.T;
            T = DM.T;                       
                                   
            %Trajetoria    
           
            
            %xd = 0;
            %yd = 2.5;
            %zd = 2;
            %psid = (pi/2);
            
            %vxd = 0;
            %vyd = 0;
            %vzd = 0;
            %vpsid = 0;
            
            %vvxd = 0;
            %vvyd = 0;
            %vvzd = 0;
            %vvpsid = 0;
              
             w = 0.4;
             dist = 1.75;
            
             xd = dist*sin(w*t) +0.1;
             yd = 1.25*dist*cos(0.5*w*t) -0.2;
             zd = 1 + 0.5*sin(w*t);
             psid = (pi/6)*sin(w*t);
            
             vxd = w*dist*cos(w*t);
             vyd = -1.25*0.5*w*dist*sin(0.5*w*t);
             vzd = 0.5*w*cos(w*t);
             vpsid = w*(pi/6)*cos(w*t);
            
            %vvxd = -0.5^2*w^2*dist*cos(w*t);
            %vvyd = -w^2*dist*sin(w*t);
            %vvzd = -1.2*w^2*cos(w*t);
            %vvpsid = -w^2*(pi)*sin(w*t);
            
%            if(t > 25 && t < 35)
%                tt = 25;
%            xd = 5*cos(0.5*w*tt);
%            yd = 5*sin(w*tt);
%            zd = 1.2*ones(size(tt));
%            psid = (pi)*sin(w*tt);
            
%            vxd = 0;
%            vyd = 0;
%            vzd = 0;
%            vpsid = 0;
            
%                vvxd = 0;
%                vvyd = 0;
 %               vvzd = 0;
  %              vvpsid = 0;
   %         end
                                    
            DM.robot_xd(DM.tmrCount) = xd;
            DM.robot_yd(DM.tmrCount) = yd;
            DM.robot_zd(DM.tmrCount) = zd;
            DM.robot_psid(DM.tmrCount) = psid;
                                    
            psi = DM.robot_psi(DM.tmrCount-1);
            
            F = [cos(psi)   -sin(psi)   0   0;
                 sin(psi)   cos(psi)    0   0;
                 0          0           1   0;
                 0          0           0   1];
            
            F1 = [DM.k1*cos(psi)  -DM.k3*sin(psi)    0   0;
                DM.k1*sin(psi) DM.k3*cos(psi)     0   0;
                0           0               DM.k5  0;
                0           0               0   DM.k7];
            
            F2 = [DM.k2*cos(psi)   -DM.k4*sin(psi)    0   0;
                DM.k2*sin(psi)    DM.k4*cos(psi)     0   0;
                0              0               DM.k6  0;
                0              0               0   DM.k8];
            
                        
            %Convertendo x, dotx para x e s   
            Vd = [vxd; vyd; vzd; vpsid];

            A = F1^(-1);
            B = F1^(-1)*F2*inv(F);
            
            Accel = [DM.robot_acelx(DM.tmrCount-1);
                DM.robot_acely(DM.tmrCount-1);
                DM.robot_acelz(DM.tmrCount-1);
                DM.robot_acelpsi(DM.tmrCount-1)]; 
            
            V = [DM.robot_vx(DM.tmrCount-1);
                DM.robot_vy(DM.tmrCount-1);
                DM.robot_vz(DM.tmrCount-1);
                DM.robot_vpsi(DM.tmrCount-1)];                       
            
            Error = [xd-DM.robot_x(DM.tmrCount-1);
                    yd-DM.robot_y(DM.tmrCount-1);
                    zd-DM.robot_z(DM.tmrCount-1);
                    psid-DM.robot_psi(DM.tmrCount-1);
                    ];  
                
           
            DM.robot_vxd(DM.tmrCount) = vxd;%;(DM.robot_xd(DM.tmrCount) - DM.robot_xd(DM.tmrCount-1))/T;
            DM.robot_vyd(DM.tmrCount) = vyd;%;(DM.robot_yd(DM.tmrCount) - DM.robot_yd(DM.tmrCount-1))/T;
            DM.robot_vzd(DM.tmrCount) = vzd;%;(DM.robot_zd(DM.tmrCount) - DM.robot_zd(DM.tmrCount-1))/T;
            DM.robot_vpsid(DM.tmrCount) = vpsid;%;(DM.robot_psid(DM.tmrCount) - DM.robot_psid(DM.tmrCount-1))/T;
                                   
                        
            v_Error = [DM.robot_vxd(DM.tmrCount)-DM.robot_vx(DM.tmrCount-1);
                DM.robot_vyd(DM.tmrCount)-DM.robot_vy(DM.tmrCount-1);
                DM.robot_vzd(DM.tmrCount)-DM.robot_vz(DM.tmrCount-1);
                DM.robot_vpsid(DM.tmrCount)-DM.robot_vpsi(DM.tmrCount-1);
                ];
            
            Vref = [vxd; vyd; vzd; vpsid];
            
            Vmax = 2*[1.6  0      0    0
                    0    1.1    0    0
                    0    0      0.25 0
                    0    0      0    0.25];
                        
            
            Vxd_maxTraj = [w*dist      0       0       0
                           0            1.25*0.5*w*dist     0       0                           
                           0            0       0.5*w   0
                           0            0       0       w*(pi/6)]; %depende da trajetoria
            
            Kc = 0.9*abs(Vmax - Vxd_maxTraj); 
            DM.control_Kc = Kc;
            DM.control_Vmax = Vmax;
            DM.control_Vxd_maxTraj = Vxd_maxTraj;
            
            v = Vref + Kc*tanh(3.1*Error);      
            %Controle cinematico  
            
            Uc = inv(F)*(v);
            DM.Uc = Uc;                      
                        
            DM.control_Uc{DM.tmrCount} = Uc;
            %Saturando as acoes
            %Convertendo x, dotx para x e s   
            %Vd = [vxd; vyd; vzd; vpsid];
            %Vvd = [vvxd; vvyd; vvzd; vvpsid];
            
            %Velcidades geradas pelo Controlador Cinematico
            Vc = F*Uc;           %w
            
            %if(Vc(1) > Vmax(1,1) || Vc(2) > Vmax(2,2) || Vc(3) > Vmax(3,3) || Vc(4) > Vmax(4,4))
            %    disp('Vc')
            %    Vc
            %    disp('Vmax')
            %    [Vmax(1,1) Vmax(2,2) Vmax(3,3) Vmax(4,4)]'
            %    disp('deu zica');
            %    pause(10);
            %end
            
            Accelc = (Vc - DM.Vc_ant)/DM.T;
            
            DM.control_Vc{DM.tmrCount} = Vc;
            DM.control_Accelc{DM.tmrCount} = Accelc;
            
            %velocidade maxima que pode tem q obedecer que a velocidade das
            %acoes de controle Xc_p nao podem ultrapassar o Vmax, assim
            %fica max(trajetoria_x) + (constante de saturacacao da tanh de
            %Uc)
            
            %kmax =  max(-w*2.5*sin(w*t)), 0.0877
            %Vmax = (0.1 + kh)*eye(4);  %velocidades maximas do drone em x,y,z e psi                
            %Vmax = 4*[(w*2.5 + kh)  0   0   0;
             %        0         (w*2.5 + kh)   0   0;
              %       0          0   (kh + 0.2)   0;
               %      0          0   0   (kh + w*(pi))];
            
            %sc = atanh(Vc./Vmax); %w                        
            %vsc = Accelc./(Vmax.*cosh(sc).^(-2)); %w                        
            
            
            sc = atanh(inv(Vmax)*Vc);                          
            Csh = [cosh(sc(1))    0   0   0;
                   0    cosh(sc(2))   0   0;
                   0    0   cosh(sc(3))   0;
                   0    0   0   cosh(sc(4))];
            
            vsc = Csh^(2)*inv(Vmax)*Accelc;            
                                    
            s = atanh(inv(Vmax)*V);            %w
            
            DM.control_sc{DM.tmrCount} = sc;
            DM.control_s{DM.tmrCount} = s;
            DM.control_vsc{DM.tmrCount} = vsc;
            
            k1c = 1.0;
            k2c = 1.0;
            k3c = 1.0;
            k4c = 1;
            K = [k1c,  0, 0, 0;
                    0,k2c, 0, 0;
                    0,	0,k3c, 0;
                    0,	0,	0,	k4c];
                            
            %Ud = A*Vmax*inv(cosh(sc)'*cosh(sc))*(vsc + K*(sc - s)) + B*Vmax*tanh(s);
            Ud = A*Vmax*Csh^(-2)*(vsc + K*(sc - s)) + B*Vmax*tanh(s);
            
            
            DM.control_Ud{DM.tmrCount} = Ud;
           % Ud = Uc;
            %(vsc + K*(sc - s))
            %Ud
            %Ud = inv(F1)*(K*(F*Uc - V) + dUc - (-F2)*Vb);
            %Vb velocidade do drone no seu referencia
            %dUc derivada das acoes de controle (aceleracao desejada)
            %
                                                		                               
            DM.robot_ux(DM.tmrCount) = Ud(1);
            DM.robot_uy(DM.tmrCount) = Ud(2);
            DM.robot_uz(DM.tmrCount) = Ud(3);
            DM.robot_upsi(DM.tmrCount) = Ud(4);
            
            % Parametros Estimando
            ddx = Accel(1);
            ddy = Accel(2);
            ddz = Accel(3);
            ddpsi = Accel(4);
            
            dx = V(1);
            dy = V(2);
            dz = V(3);
            dpsi = V(4);
            
            PSI = [cos(psi)*ddx+sin(psi)*ddy    -cos(psi)*dx-sin(psi)*dy     0                           0                          0       0    0       0;
                   0                            0                           -sin(psi)*ddx+cos(psi)*ddy  sin(psi)*dx-cos(psi)*dy     0       0    0       0;
                   0                            0                            0                           0                          ddz     -dz  0       0;
                   0                            0                            0                           0                          0       0    ddpsi   -dpsi];
               
            
            %id = find(Ud > 1);
            %idx = find(Ud < -1);
            %Normaliza
            %try
%                Ud(id) = 1;
 %               Ud(idx) = -1;
  %          end
            
                           
            %DM.parametros{DM.tmrCount} = inv(PSI'*PSI)*PSI'*Ud;
            DM.parametros{DM.tmrCount} = pinv(PSI)*Ud;
                                     
          
            DM.Uc_ant = DM.Uc;
            DM.Vc_ant = Vc;
        end            
    end
end