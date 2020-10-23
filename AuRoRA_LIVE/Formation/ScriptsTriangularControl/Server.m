close all
clear all
clc
try
    fclose(instrfindall);
end
%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'Chakal';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))
cd(PastaAtual)

%% Início da rotina
n = 4; %Numero de robôs

%%Criar objeto da nova classe Robo
for ii=1:n
    P{ii} = Robot;
    P{ii}.pID = ii;
    P{ii}.mSetUDP('255.255.255.255');
    
    P{ii}.pStatus.Conectado = 1;
    
    P{ii}.pStatus.nF = 0;
    P{ii}.pStatus.Flag = 0;
end
%%  Definir postura dos robôs e enviar aos clientes

distx = .62;
disty = .60;

%distx = .63;
%disty = .62;

%Fg.Xd = [-distx distx -distx distx; -disty -disty disty disty; ones(1,4)];


P{4}.pPos.X = [distx disty 0 0 0 0 0 0 0 0 0 0]';
%P{4}.pPos.X = [0 -disty 0 0 0 0 0 0 0 0 0 0]';

P{4}.pPos.Xi = P{4}.pPos.X';

P{3}.pPos.X = [-distx disty 0 0 0 0 0 0 0 0 0 0]';
%P{3}.pPos.X = [distx 0 0 0 0 0 0 0 0 0 0 0]';

P{3}.pPos.Xi = P{3}.pPos.X';

P{1}.pPos.X = [-distx -disty 0 0 0 0 0 0 0 0 0 0]';
%P{1}.pPos.X = [0 +disty 0 0 0 0 0 0 0 0 0 0]';
P{1}.pPos.Xi = P{1}.pPos.X';

P{2}.pPos.X = [distx -disty 0 0 0 0 0 0 0 0 0 0]';
%P{2}.pPos.X = [-distx 0 0 0 0 0 0 0 0 0 0 0]';

P{2}.pPos.Xi = P{2}.pPos.X';

Fg.Xd = [P{1}.pPos.X(1) P{2}.pPos.X(1) P{3}.pPos.X(1) P{4}.pPos.X(1);...
    P{1}.pPos.X(2) P{2}.pPos.X(2) P{3}.pPos.X(2) P{4}.pPos.X(2); ones(1,4)];

% P{5}.pPos.X([0 2 0 0 0 0 0 0 0 0 0 0]);

for ii=1:n
    P{ii}.mAtualizarServidor;
end

%Ponderacao = input('Qual método controle individual? \n 0 - Tradicional \n 1 - Média \n  ');
Ponderacao = 2;
%%
nF = n - 2;
MatrizPonderacao = zeros(n, nF);
for ii=1:nF
    F(ii)= FormacaoTriangular ;
    F(ii).pID = ii;
    F(ii).mIdentSeq(P);
    F(ii).mTransDir(P);
    F(ii).pPos.Xd =  F(ii).pPos.X;
    F(ii).pPonderacao = Ponderacao;
    MatrizPonderacao(ii, ii) = 1;
    MatrizPonderacao(ii+1, ii) = 1;
    MatrizPonderacao(ii+2, ii) = 1;
    F(ii).mTransInv(P);
    % fig(ii+20) = patch('Faces',[1 2 3],'Vertices',[P{ii}.pPos.Xd(1:2)';P{ii+1}.pPos.Xd(1:2)';P{ii+2}.pPos.Xd(1:2)'],'FaceColor',[.6 0 .6],'FaceAlpha',0.2,'EdgeColor',[.5 .5 .5]);
    
end
%drawnow
%pause
for ii=1:n
    P{ii}.pPos.mXd = zeros(2,nF);
    P{ii}.pPos.mdXd = zeros(2,nF);
end
%%
%Numero de formações que um robo pertence
for ii=1:nF
    P{ii}.pStatus.nF = P{ii}.pStatus.nF + 1;
    P{ii+1}.pStatus.nF = P{ii+1}.pStatus.nF + 1;
    P{ii+2}.pStatus.nF = P{ii+2}.pStatus.nF + 1;
end
%%

for ii = 1:n
    RastrosR(ii).X = [];
    RastrosR(ii).Xd = [];
    RastrosR(ii).U = [];
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

%Ka = 1*eye(6);
Ka = 1*eye(6);
Ki = 1e-02*ones(6,1);

%Parâmetros da trajetória de referência
rx = 1.7; %1.7
ry = .9;
%rx = 0.3;
%ry = 0.3;

T = 50;
w = 2*pi/T*[1/4 1 1.25];
%w = 2*pi/T*[1/4 1 1];

phi = 0;
t_trans = T/10;

parar = 0;

figure

c = 1;
breakloop = 0;
t_experimento = tic;
t_plot = tic;
t_controle = tic;
t_case=tic;

RastroControle = [];
RastroSimulacao = toc(t_experimento);

while(parar == 0)
    
    if(toc(t_controle)>0.1)
        RastroControle = [RastroControle; toc(t_controle)];
        t_controle = tic;
        RastroSimulacao = [RastroSimulacao; toc(t_experimento)];
        
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
                        t_trans(c) = t_trans(c-1) + (1/(w(3)/(2*pi/T)))*T;
                    end
                    
                case 3
                    Fg.cg(1) = 1.3*rx*cos(w(3)*(toc(t_experimento)-t_trans(c-1)));
                    Fg.cg(2) = 1.3*ry*sin(w(3)*(toc(t_experimento)-t_trans(c-1)));
                    Fg.cg(4) = -1.3*rx*w(3)*sin(w(3)*(toc(t_experimento)-t_trans(c-1)));
                    Fg.cg(5) = 1.3*ry*w(3)*cos(w(3)*(toc(t_experimento)-t_trans(c-1)));
                    if toc(t_experimento)>t_trans(c)
                        c = c+1;
                        t_trans(c) = t_trans(c-1);
                    end
                    
                case 4
                    Fg.cg(4) = 0;
                    Fg.cg(5) = 0;
%                     break;
%                     Fg.cg(1) = 0;
%                     Fg.cg(2) = 0;
%                     Fg.cg(4) = 0;
%                     Fg.cg(5) = 0;
%                     if toc(t_experimento)>t_trans(c)
%                         c = c+1;
%                         t_trans(c) = t_trans(c-1) + 2 ;
%                     end
                    if toc(t_experimento)>t_trans(c)
                        break;
                    end
%                 case 5
%                     if toc(t_experimento)>t_trans(c)
%                         break;
%                     end
            end
            
            Fg.cg(3) = 0;%-pi/4;%atan2(Fg.cg(5),Fg.cg(4));
            H = [cos(Fg.cg(3)), -sin(Fg.cg(3)), Fg.cg(1); sin(Fg.cg(3)), cos(Fg.cg(3)), Fg.cg(2); 0 0 1];
            Fg.X = H*Fg.Xd;
        for ii=1:nF
            F(ii).mTransDirDes(Fg.X(:,ii:ii+2));
            F(ii).pPos.Xd(7) = Fg.cg(4);
            F(ii).pPos.Xd(8) = Fg.cg(5);
        end
        %         for ii=1:nF
        %             try
        %                 delete(fig(ii+20));
        %                 delete(fig(ii+30));
        %             end
        %         end
        
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
                F(ii).pPos.dXr = F(ii).pPos.Xd(7:12) + 500*Ka*tanh(.001*Ka*F(ii).pPos.Xtil(1:6));
                %750
            else
            end
            
            F(ii).mJacobianoInv(P);
            F(ii).mTransInv(P);
            %
            %             fig(ii+20) = patch('Faces',[1 2 3],'Vertices',[P{ii}.pPos.Xd(1:2)';P{ii+1}.pPos.Xd(1:2)';P{ii+2}.pPos.Xd(1:2)'],'FaceColor',[.6 0 .6],'FaceAlpha',0.2,'EdgeColor',[.5 .5 .5]);
            %             fig(ii+30) = patch('Faces',[1 2 3],'Vertices',[P{ii}.pPos.X(1:2)';P{ii+1}.pPos.X(1:2)';P{ii+2}.pPos.X(1:2)'],'FaceColor',[.1 .6 .1],'FaceAlpha',0.2,'EdgeColor',[.5 .5 .5]);
        end
        %         grid on
        %         axis([-3 3 -3 3])
        %         drawnow;
        
        if Ponderacao == 1
            for ii=1:n
                P{ii}.pPos.Xd(7:8) = sum(P{ii}.pPos.mdXd,2)./P{ii}.pStatus.nF;
                P{ii}.pPos.Xd(1:2) = sum(P{ii}.pPos.mXd,2)./P{ii}.pStatus.nF;
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
            P{ii}.mAtualizarServidor;
            RastrosR(ii).X = [RastrosR(ii).X; P{ii}.pPos.X'];
            RastrosR(ii).Xd = [RastrosR(ii).Xd; P{ii}.pPos.Xd'];
            RastrosR(ii).U = [RastrosR(ii).U; P{ii}.pSC.U];
        end
        for ii = 1:nF
            IAE(ii) = IAE(ii) + norm(F(ii).pPos.Xtil(1:2))*(RastroSimulacao(end,1)-RastroSimulacao(end-1,1));
            ITAE(ii) = ITAE(ii) + RastroSimulacao(end,1)*norm(F(ii).pPos.Xtil(1:2))*(RastroSimulacao(end,1)-RastroSimulacao(end-1,1));
        end
        for ii = 1:n
            IASC(ii) = IASC(ii) + norm(P{ii}.pSC.U)*(RastroSimulacao(end,1)-RastroSimulacao(end-1,1));
        end
        
        if(toc(t_plot)>1000)
            t_plot = tic;
            for ii=1:n
                P{ii}.mCADdel;
            end
            for ii=1:n
                P{ii}.mCADplot2D(bin2dec(dec2bin(P{ii}.pID,3)')');
            end
            
            for ii=1:n
                try
                    delete(figRobo(ii));
                end
                %                 figRobo(ii) = plot(P{ii}.pPos.X(1), P{ii}.pPos.X(2), 'ks','LineWidth',2);
                figRobo(ii) = plot(P{ii}.pPos.Xd(1), P{ii}.pPos.Xd(2), 'ks','LineWidth',2);
            end
            hold on
            %         for ii=1:(3*nF)
            %             try
            %                 delete(fig(ii));
            %             end
            %         end
            iii=1;
            for ii=1:nF
                try
                    delete(fig(ii+20));
                    delete(fig(iii));
                    delete(fig(iii+1));
                    delete(fig(iii+2));
                end
                
                % fig(iii) = plot(RastrosF(ii).X(:,1),RastrosF(ii).X(:,2), 'b');
                fig(iii+1) = plot(RastrosF(ii).Xd(:,1),RastrosF(ii).Xd(:,2), 'r--');
                %fig(iii+2) = plot(F(ii).pPos.X(1),F(ii).pPos.X(2),'ks');
                iii = iii+3;
            end
            %
            %         fig(1) = plot(Rastros(1).X(:,1),Rastros(1).X(:,2), 'b');
            %         fig(2) = plot(Rastros(1).Xd(:,1),Rastros(1).Xd(:,2), 'r--');
            %         fig(3) = plot(F(1).pPos.X(1),F(1).pPos.X(2),'ks');
            %
            %         fig(4) = plot(Rastros(2).X(:,1),Rastros(2).X(:,2), 'b');
            %         fig(5) = plot(Rastros(2).Xd(:,1),Rastros(2).Xd(:,2), 'r--');
            %         fig(6) = plot(F(2).pPos.X(1),F(2).pPos.X(2),'ks');
            %
            %         fig(7) = plot(Rastros(2).X(:,1),Rastros(2).X(:,2), 'b');
            %         fig(8) = plot(Rastros(2).Xd(:,1),Rastros(2).Xd(:,2), 'r--');
            %         fig(9) = plot(F(2).pPos.X(1),F(2).pPos.X(2),'ks');
            
            %
            %             for ii=1:nF
            %                 if (F(ii).pSeq == 1)
            %                     fig(ii+20) = patch('Faces',[1 2 3],'Vertices',[P{ii}.pPos.X(1:2)';P{ii+1}.pPos.X(1:2)';P{ii+2}.pPos.X(1:2)'],'FaceColor',[.6 0 .6],'FaceAlpha',0.2,'EdgeColor',[.5 .5 .5]);
            %                 else
            %                     fig(ii+20) = patch('Faces',[1 2 3],'Vertices',[P{ii}.pPos.X(1:2)';P{ii+1}.pPos.X(1:2)';P{ii+2}.pPos.X(1:2)'],'FaceColor',[1 0 0],'FaceAlpha',0.2,'EdgeColor',[.5 .5 .5]);
            %                 end
            %             end
            
            %             for ii=1:nF
            %                 if (F(ii).pSeq == 1)
            %                     fig(ii+20) = patch('Faces',[1 2 3],'Vertices',[P{ii}.pPos.Xd(1:2)';P{ii+1}.pPos.Xd(1:2)';P{ii+2}.pPos.Xd(1:2)'],'FaceColor',[.6 0 .6],'FaceAlpha',0.2,'EdgeColor',[.5 .5 .5]);
            %                 else
            %                     fig(ii+20) = patch('Faces',[1 2 3],'Vertices',[P{ii}.pPos.Xd(1:2)';P{ii+1}.pPos.Xd(1:2)';P{ii+2}.pPos.Xd(1:2)'],'FaceColor',[1 0 0],'FaceAlpha',0.2,'EdgeColor',[.5 .5 .5]);
            %                 end
            %             end
            % title(['V1 = ',num2str(P{1}.pSC.U(1),3),'    V2 = ',num2str(P{2}.pSC.U(1),3),'    V3 = ',num2str(P{3}.pSC.U(1),3), '    V4 = ', num2str(P{4}.pSC.U(1),3)])
            %             axis([-3 3 -2 2])
            %             grid on
            %             drawnow;
        end
    end
end

if(toc(t_experimento))>=(T+2)
    for nmsg=1:20
        %enviar mensagem parar
        for ii=1:n
            P{ii}.mParar;
        end
    end
    figure
    axis([-3 3 -2 2])
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

Arq.nome = ['Dados_',num2str(n),'Robos_',num2str(Ponderacao),'Ponderacao_',datestr(now,'dd-mm-yy_HH-MM-SS')];
Arq.Resp = questdlg(['Salvar ', Arq.nome, ' ?'], '', 'Sim', 'Não', 'Sim');
if strcmp(Arq.Resp,'Sim')
    save(Arq.nome);
end;

disp('Fim de simulação')

