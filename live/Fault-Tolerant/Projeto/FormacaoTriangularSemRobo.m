close all
clear all
clc

try
    fclose(instrfindall);
catch
end
%% ADD TO PATH
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA 2018';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
% addpath(genpath(pwd))

%% INICIO DO PROGRAMA
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Variáveis iniciais
TF = TriangularFormationBaju;

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Formação Inicial
TF.pPos.X = [0; -1.5; 0; 0; -1.5; 1.5; 0; -0.4393; 1.0607];
X = TF.pPos.X;

% Posição dos Robôs
TF.tDirTrans;

% Formação Inicial
Q = [0;              % x
     -1.5;              % y
     0;              % z
     pi/4;            % phi
     pi/2;            % psi
     0;          % tetha
     1.5;           % p
     1.5;           % q
     pi/4];          % beta
    
Qd = Q;

% Posição dos Robôs
X_C = FT_inversa2(Q);
Xd = X_C;

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Esfera limite
% E = 0:0.1:pi;
% 
% X_E = [raio*cos(psi)*cos(E);
%        raio*sin(psi)*cos(E);
%        raio*sin(E)];
%    
% % R rotaciona em relação ao eixo-z
% R = [cos(pi/10)  -sin(pi/10)  0;
%      sin(pi/10)  cos(pi/10)   0;
%      0           0            1];
 
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Figura da simulação
figure
H(1) = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'b','LineWidth',2);
hold on
H(2) = plot3([X(1) X(7)],[X(2) X(8)],[X(3) X(9)],'b','LineWidth',2);
H(3) = plot3([X(4) X(7)],[X(5) X(8)],[X(6) X(9)],'r','LineWidth',2);
H(4) = plot3([X_C(1) X_C(4)],[X_C(2) X_C(5)],[X_C(3) X_C(6)],'r');
H(5) = plot3([X_C(1) X_C(7)],[X_C(2) X_C(8)],[X_C(3) X_C(9)],'r');
H(6) = plot3([X_C(4) X_C(7)],[X_C(5) X_C(8)],[X_C(6) X_C(9)],'b');
grid on

% for i=1:10
% for i=1:(size(X_E,2)-1)
%     plot3([X_E(1,i) X_E(1,i+1)],[X_E(2,i) X_E(2,i+1)],[X_E(3,i) X_E(3,i+1)],'k');
% end
%     X_E = R*X_E;
% end

axis([X(1)-3 X(1)+3 X(2)-3 X(2)+3 X(3) X(3)+3])
% axis equal
view([-50 30])

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Variáveis iniciais
Tmax = 5;
ta = 0.1;
tp = 0.1;
t1 = tic;
t2 = tic;
% A.pPar.ti = tic;

pause
t = tic;
%% SIMULAÇÃO
while toc(t) < Tmax
    if toc(t1) > ta
        t1 = tic;
        
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
        % TRAJETORIA DA FORMAÇÃO
        TF.pPos.Qd = [0;
                      -1.5; 
                      0; 
                      -pi/4*toc(t)/Tmax; 
                      0; 
                      pi/2; 
                      1.5; 
                      1.5; 
                      pi/4];
        
        TF.pPos.dQd = [0; 0; 0;
                       -pi/4/Tmax; 0; 0;
                       0; 0; 0];
                  
        TF.tFormationControl;
                  
        TF.tInvTrans;
            
        
        TF.pPos.X = TF.pPos.X + ta*TF.pPos.dXr;
        X = TF.pPos.X;
        
        
        
        Qd = [0;                % x
            -1.5;                % y
            0;                % z
            pi/4*toc(t)/Tmax;% phi
            pi/2;              % psi
            0;            % theta
            1.5;             % p
            1.5;             % q
            pi/4];            % beta
        dQd = [0;              % x
            0;                % y
            0;                % z
            pi/4/Tmax;       % phi
            0;                % psi
            0;                % tetha
            0;                % p
            0;                % q
            0];               % beta
        Xd_C = FT_inversa2(Qd);
        Qtil = Qd - Q;
        dQ = dQd + TF.pPar.K1*Qtil;
        
        
        dXr_C = FT_jacobianoInv2(Q,dQ);
        
        X_C = X_C + ta*dXr_C;
        Q = FT_direta2(X_C);
        
        try
            delete(H)
        end
                
        H(1) = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'b','LineWidth',2);
        H(2) = plot3([X(1) X(7)],[X(2) X(8)],[X(3) X(9)],'b','LineWidth',2);
        H(3) = plot3([X(4) X(7)],[X(5) X(8)],[X(6) X(9)],'r','LineWidth',2);
        H(4) = plot3([X_C(1) X_C(4)],[X_C(2) X_C(5)],[X_C(3) X_C(6)],'r');
        H(5) = plot3([X_C(1) X_C(7)],[X_C(2) X_C(8)],[X_C(3) X_C(9)],'r');
        H(6) = plot3([X_C(4) X_C(7)],[X_C(5) X_C(8)],[X_C(6) X_C(9)],'b');

        drawnow
    end
end
