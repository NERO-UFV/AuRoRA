close all
clear
clc
try
    fclose(instrfindall);
end
%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Início da rotina
n = 4; %Numero de robôs

%Criar objeto da nova classe Robo
for ii=1:n
    P{ii} = Pioneer3DX;
    P{ii}.pID = ii;
    P{ii}.pPar.nF = 0;
end
%%  Definir postura dos robôs e enviar aos clientes
distx = .62;
disty = .60;

P{1}.pPos.X = [-distx -disty 0 0 0 0 0 0 0 0 0 0]';
P{1}.pPos.Xi = P{1}.pPos.X';
P{2}.pPos.X = [distx -disty 0 0 0 0 0 0 0 0 0 0]';
P{2}.pPos.Xi = P{2}.pPos.X';
P{3}.pPos.X = [-distx disty 0 0 0 0 0 0 0 0 0 0]';
P{3}.pPos.Xi = P{3}.pPos.X';
P{4}.pPos.X = [distx disty 0 0 0 0 0 0 0 0 0 0]';
P{4}.pPos.Xi = P{4}.pPos.X';

Fg.Xi = [P{1}.pPos.X(1) P{2}.pPos.X(1) P{3}.pPos.X(1) P{4}.pPos.X(1);...
    P{1}.pPos.X(2) P{2}.pPos.X(2) P{3}.pPos.X(2) P{4}.pPos.X(2); ones(1,4)];
Losango = [0, -distx, distx, 0; ...
    -1.5*disty, 0, 0, 1.5*disty; ones(1,4)];
%Fg.Xd = [ones(1,4); zeros(2,4)]+ Losango;
Fg.Xd = Fg.Xi;
%[Fg.Xi, Fg.Xd, nMod] = PermutaPosicoes(Fg.Xi,Fg.Xd);

Fg.Xcalc = Fg.Xd;
%Ponderacao = input('Qual método controle individual? \n 0 - Tradicional \n 1 - Média \n  ');
Ponderacao = 1;

%% Cria objetos formação
nF = n - 2;
MatrizPonderacao = zeros(n, nF);
for ii=1:nF
    F(ii)= FormacaoTriangular ;
    F(ii).pID = ii;
    F(ii).mIdentSeq(Fg.Xd(1:2, ii:ii+2));
    F(ii).mTransDir(P);
    F(ii).mTransDirDes(Fg.Xd);
    F(ii).pPonderacao = Ponderacao;
    MatrizPonderacao(ii, ii) = 1;
    MatrizPonderacao(ii+1, ii) = 1;
    MatrizPonderacao(ii+2, ii) = 1;
    F(ii).mTransInv(P);
end
for ii=1:n
    P{ii}.pPos.mXd = zeros(2,nF);
    P{ii}.pPos.mdXd = zeros(2,nF);
end
%% Numero de formações que um robo pertence
for ii=1:nF
    P{ii}.pPar.nF = P{ii}.pPar.nF + 1;
    P{ii+1}.pPar.nF = P{ii+1}.pPar.nF + 1;
    P{ii+2}.pPar.nF = P{ii+2}.pPar.nF + 1;
end

%% Definições
for ii = 1:n
    RastrosR(ii).X = [];
    RastrosR(ii).Xd = [];
    RastrosR(ii).Xtil = [];
    IASC(ii) = 0;
end
for ii = 1:nF
    RastrosF(ii).X = [];
    RastrosF(ii).Xd = [];
    RastrosF(ii).Xtil = [];
    erro(ii).Integral = zeros(6,1);
    IAE(ii) = 0;
    ITAE(ii) = 0;
end

Ka = 1*eye(6);
Ki = 1e-02*ones(6,1);
kp1 = .2;
kp2 = 2;

%Parâmetros da trajetória de referência
rx = 1.7;
ry = .9;
%rx = 0.3;
%ry = 0.3;

T = 50;
w = 2*pi/T*[1/4 1 1.5];
phi = 0;
t_trans = T/10;

parar = 0;
c = 1;
figure

RastroControle = [];

t_experimento = tic;
t_plot = tic;
t_controle = tic;
t_case=tic;
RastroSimulacao = toc(t_experimento);

while(parar == 0)
    
    if(toc(t_controle)>0.05)
        RastroControle = [RastroControle; toc(t_controle)];
        t_controle = tic;
        RastroSimulacao = [RastroSimulacao; toc(t_experimento)];
        
        for ii=1:n
            P{ii}.rGetSensorData;
        end
        for ii=1:nF
            F(ii).mTransDir(P);
            RastrosF(ii).X = [RastrosF(ii).X; F(ii).pPos.X(1:2)'];
        end
        
        switch c
            case 1
                Fg.cg(1) = rx*sin(w(1)*toc(t_experimento));
                Fg.cg(2) = ry*sin(2*w(1)*toc(t_experimento));
                Fg.cg(4) = rx*w(1)*cos(w(1)*toc(t_experimento));
                Fg.cg(5) = 2*ry*w(1)*cos(2*w(1)*toc(t_experimento));
                
                if toc(t_experimento)>t_trans(c)
                    phi = (w(1)-w(2))*t_trans(c);
                    c = c+1;
                    t_trans(c) = (pi/3+2*pi - phi)/w(c);
                end
                
            case 2
                Fg.cg(1) = rx*sin(w(2)*toc(t_experimento) + phi);
                Fg.cg(2) = ry*sin(2*(w(2)*toc(t_experimento)+ phi));
                Fg.cg(4) = rx*w(2)*cos(w(2)*toc(t_experimento)+ phi);
                Fg.cg(5) = 2*ry*w(2)*cos(2*(w(2)*toc(t_experimento)+ phi));
                
                if toc(t_experimento)>t_trans(c)
                    phi = (w(1)-w(2))*t_trans(c);
                    c = c+1;
                    t_trans(c) = t_trans(c-1) + (1/1.5)*T;
                end
                
            case 3
                Fg.cg(1) = 1.3*rx*cos(w(3)*(toc(t_experimento)-t_trans(c-1)));
                Fg.cg(2) = 1.3*ry*sin(w(3)*(toc(t_experimento)-t_trans(c-1)));
                Fg.cg(4) = -1.3*rx*w(3)*sin(w(3)*(toc(t_experimento)-t_trans(c-1)));
                Fg.cg(5) = 1.3*ry*w(3)*cos(w(3)*(toc(t_experimento)-t_trans(c-1)));
                if toc(t_experimento)>t_trans(c)
                    c = c+1;
                    t_trans(c) = t_trans(c-1) + 10 ;
                end
                
            case 4
                break;
        end
        
        Fg.cg(3) = 0;               %-pi/4;%atan2(Fg.cg(5),Fg.cg(4));
        H = [cos(Fg.cg(3)), -sin(Fg.cg(3)), Fg.cg(1); sin(Fg.cg(3)), cos(Fg.cg(3)), Fg.cg(2); 0 0 1];
        Fg.X = H*Fg.Xd;
 
        for ii=1:nF
            F(ii).mTransDirDes(Fg.X((1:2),ii:(ii+2)));
            F(ii).pPos.Xd(7) = Fg.cg(4);
            F(ii).pPos.Xd(8) = Fg.cg(5);  
        end
        
        for ii=1:nF
            RastrosF(ii).Xd = [RastrosF(ii).Xd; F(ii).pPos.Xd(1:2)'];
            F(ii).pPos.Xtil = F(ii).pPos.Xd(1:6) - F(ii).pPos.X(1:6);
            RastrosF(ii).Xtil = [RastrosF(ii).Xtil; F(ii).pPos.Xtil(1:3)'];
            if( abs(F(ii).pPos.Xtil(3))>pi)
                if(F(ii).pPos.Xtil(3)>=0)
                    F(ii).pPos.Xtil(3) = -2*pi + F(ii).pPos.Xtil(3);
                else
                    F(ii).pPos.Xtil(3) = 2*pi + F(ii).pPos.Xtil(3);
                end
            end
            
            if norm(F(ii).pPos.Xtil(1:2))>0
                F(ii).pPos.dXr = F(ii).pPos.Xd(7:12) + 750*Ka*tanh(.001*Ka*F(ii).pPos.Xtil(1:6));
                %                 F(ii).pPos.dXr = F(ii).pPos.Xd(7:12) + 100*Ka*tanh(.002*Ka*F(ii).pPos.Xtil(1:6)) +...
                %                     (Ki./(F(ii).pPos.Xtil(1:6).^2)).*(Ka*tanh(.0002*Ka*F(ii).pPos.Xtil(1:6)));
                
            else
                %                 erro(ii).Integral = erro(ii).Integral + 0.1*F(ii).pPos.Xtil(1:6);
                %                 F(ii).pPos.dXr = F(ii).pPos.Xd(7:12) + .7*Ka*tanh(1*Ka*F(ii).pPos.Xtil(1:6));%+ 0.15*erro(ii).Integral; %(5e-2)./(F(ii).pPos.Xtil).*tanh( 2.5*(F(ii).pPos.Xtil).^2 );
            end
            
            F(ii).mJacobianoInv(P);
            F(ii).mTransInv(P);
        end
        
        %
        %
        %         disp('==============================================')
        %         disp('Triangulo 1 - Desejado')
        %         disp([P{1}.pPos.Xd(1:2), P{2}.pPos.Xd(1:2), P{3}.pPos.Xd(1:2)])
        %         disp('Triangulo 1 - Atual')
        %         disp([P{1}.pPos.X(1:2), P{2}.pPos.X(1:2), P{3}.pPos.X(1:2)])
        %         disp('==============================================')
        %         disp('Triangulo 2 - Desejado')
        %         disp([P{2}.pPos.Xd(1:2), P{3}.pPos.Xd(1:2), P{4}.pPos.Xd(1:2)])
        %         disp('Triangulo 2 - Atual')
        %         disp([P{2}.pPos.X(1:2), P{3}.pPos.X(1:2), P{4}.pPos.X(1:2)])
        %
        %pause
        if Ponderacao == 1
            for ii=1:n
                P{ii}.pPos.Xd(7:8) = sum(P{ii}.pPos.mdXd,2)./P{ii}.pPar.nF;
                P{ii}.pPos.Xd(1:2) = sum(P{ii}.pPos.mXd,2)./P{ii}.pPar.nF;
            end
        end
        
        if Ponderacao == 2
            mQj = zeros(1, nF);
            for ii=1:nF
                mQj(1,ii) = norm(F(ii).pPos.Xtil);
            end
            for ii=1:n
                somaDenominador = 0;
                somaNumerador = zeros(4,1);
                for jj=1:nF
                    somaDenominador = somaDenominador + MatrizPonderacao(ii,jj)*mQj(1,jj);
                    somaNumerador = somaNumerador + MatrizPonderacao(ii,jj)*mQj(1,jj)*[P{ii}.pPos.mXd(:,jj);P{ii}.pPos.mdXd(:,jj)];
                end
                P{ii}.pPos.Xd(1:2) = somaNumerador(1:2)./somaDenominador;
                P{ii}.pPos.Xd(7:8) = somaNumerador(3:4)./somaDenominador;
            end
        end
        
        for ii=1:n
            K = [cos(P{ii}.pPos.X(6)), -P{ii}.pPar.a*sin(P{ii}.pPos.X(6)); sin(P{ii}.pPos.X(6)), P{ii}.pPar.a*cos(P{ii}.pPos.X(6))];
            P{ii}.pSC.Ur = K\P{ii}.pPos.Xd(7:8);
            fCompensadorDinamico(P{ii});
            P{ii}.rSendControlSignals;
            RastrosR(ii).Robo.X = [RastrosR(ii).X; P{ii}.pPos.X(1),P{ii}.pPos.X(2) ];
            RastrosR(ii).Robo.Xd = [RastrosR(ii).Xd; P{ii}.pPos.Xd(1),P{ii}.pPos.Xd(2)];
        end
        for ii = 1:nF
            IAE(ii) = IAE(ii) + norm(F(ii).pPos.Xtil(1:2))*(RastroSimulacao(end,1)-RastroSimulacao(end-1,1));
            ITAE(ii) = ITAE(ii) + RastroSimulacao(end,1)*norm(F(ii).pPos.Xtil(1:2))*(RastroSimulacao(end,1)-RastroSimulacao(end-1,1));
        end
        for ii = 1:n
            IASC(ii) = IASC(ii) + norm(P{ii}.pSC.U)*(RastroSimulacao(end,1)-RastroSimulacao(end-1,1));
        end
        
        if(toc(t_plot)>0.2)
            t_plot = tic;
            for ii=1:n
                P{ii}.mCADdel;
            end
            for ii=1:n
                P{ii}.pPos.Xc = P{ii}.pPos.X(1:6);
                P{ii}.mCADplot2D(bin2dec(dec2bin(P{ii}.pID,3)')');
            end
            
            for ii=1:nF
                try
                    delete(fig(ii+20));
                end
                if (F(ii).pSeq == 1)
                    fig(ii+20) = patch('Faces',[1 2 3],'Vertices',[P{ii}.pPos.X(1:2)';P{ii+1}.pPos.X(1:2)';P{ii+2}.pPos.X(1:2)'],'FaceColor',[.6 0 .6],'FaceAlpha',0.2,'EdgeColor',[.5 .5 .5]);
                else
                    fig(ii+20) = patch('Faces',[1 2 3],'Vertices',[P{ii}.pPos.X(1:2)';P{ii+1}.pPos.X(1:2)';P{ii+2}.pPos.X(1:2)'],'FaceColor',[1 0 0],'FaceAlpha',0.2,'EdgeColor',[.5 .5 .5]);
                end
            end
           
            grid on
            axis([-4 4 -4 4])
            drawnow;
        end
    end
end

%%
if(toc(t_experimento))>=(T)
    figure
    axis([-4 4 -3 3])
    hold on
    grid on
    for ii=1:nF
        color = [0.25 0.25 0.25];
        color(ii) = 1;
        plot(RastrosF(ii).X(:,1), RastrosF(ii).X(:,2), 'Color', color);
        color = [0.5 0.5 0.5];
        color(ii) = 1;
        plot(RastrosF(ii).Xd(:,1), RastrosF(ii).Xd(:,2),'Color', color, 'LineStyle', '--','LineWidth', 1.5);
    end
    parar = 1;
end

%%
Arq.nome = ['Dados_',num2str(n),'Robos_',num2str(Ponderacao),'Ponderacao_',datestr(now,'dd-mm-yy_HH-MM-SS')];
Arq.Resp = questdlg(['Salvar ', Arq.nome, ' ?'], '', 'Sim', 'Não', 'Sim');
if strcmp(Arq.Resp,'Sim')
    save(Arq.nome);
end;

disp('Fim de simulação')