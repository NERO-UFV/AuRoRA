% Demo 1 - Conectar ao ArDrone

close all
clear
clc
try
    fclose(instrfindall);
end
addpath(genpath(pwd))

% 


figure(1)
drawnow

A = ArDrone;
A.mConnect;

% Decolar
A.mTakeOff

pause(5)

X = [];
pac_tot = 0;
pac_rec = 0;

tmax = 30; % Tempo Simulação em segundos
t = tic;
tc = tic;


while toc(t) < tmax
    if toc(tc) > 1/30
        tc = tic;
        % Obter dados de voo
        A.rGetStatusRawData
        
        pac_tot = pac_tot + 1;
        
        if sum(A.pCom.cRawData) > 1
            pac_rec = pac_rec + 1;
            % disp('ok')
            Y = [A.pCom.cRawData toc(t)];
            X = [X; Y];
            
            try
                delete(h1,h2,h3)
            end
            
            subplot(3,1,1),h1 = plot(X(:,end),X(:,2:4)); axis([0 tmax -45 45])
            subplot(3,1,2),h2 = plot(X(:,end),X(:,5)); axis([0 tmax 0 3])
            subplot(3,1,3),h3 = plot(X(:,end),X(:,8)); axis([0 tmax -0.5 0.5])
            drawnow
            
        end
    end
end

% Aterrissar/Desconectar
A.mLand
A.mDisconnect

% display([pac_tot pac_rec])

