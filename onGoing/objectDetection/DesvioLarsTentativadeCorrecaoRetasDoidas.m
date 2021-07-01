%% Inicializa��o do Programa
clear
close all
clc
%% 
% A corre��o das retas se dar� por esperar ter 3 retas com uma varia��o
% pequena em seus coeficientes angulares pra tomar ela como v�lida. Isso
% deve evitar que flutua��es nos sensores d�em resultados inesperados.
try
    fclose(instrfindall);
end
addpath(genpath(pwd))
P = Pioneer3DX;
P.pPar.a = 0.15;
try
    P.mConectarMobileSim;
    P.mDefinirPosturaInicial([0 0 0]);  
    
    max_sensor_distancia_sonar = 5.0;     
    medidas_sensor_sonar = [90 50 30 10 -10 -30 -50 -90];
    medidas_sensor_sonar(1,:) = medidas_sensor_sonar(1,:)*pi/180;   

%    og = robotics.OccupancyGrid(25,25,5);
 %   og.GridLocationInWorld = [-3 -3];   
    
    gamma = 0;
    fe=0.8; 
    dobs = 1;
    k1 = 0.35;
    k2 = 0.5;  
    HH=figure(3);
%     HHH=figure(4);
%     matriz(:,:,1) = uint8(og.occupancyMatrix*255);
%     matriz(:,:,2) = uint8(og.occupancyMatrix*255);
%     matriz(:,:,3) = uint8(og.occupancyMatrix*255);
%     figure(1);
%     h1 = imshow(matriz);
    mov_robo = [];
    PosicaoRobo=[];
    VelocidadeRobo=[];
    SensoresUsados=[];
    XLido=[];
    YLido=[];
    RetasRQuadrado=zeros(56,7);
    RQuadradoMinimo=0.8;
    RQuadrado=[];
    Sensores=[];
    DadosRobo=[];
    SinaisControle=[];
    DistanciaSensores=[];
    ult_pos = -1;
    d = 0;
    tmax = 180; t = tic; ts = tic;
    %Posi��o desejada
    
    contador=1;
    Xplot=[];
    Yplot=[];
%% Inicializa a figura
nn = 0:250;
hh = figure(2);
set(hh,'units','pix','pos',[5 100 1000 900],'PaperPositionMode','auto')

for k = 1:6
    ax(k) = subplot(6,1,k);
%     if k < 4
%         tracet(k) = plot(nn,zeros(1,size(nn,2)),'b--'); hold on
%     end
    
    trace(k) = plot(nn,zeros(1,size(nn,2)),'r'); hold on
    
    if k==6
        Pontilhado(1:251)=RQuadradoMinimo;
        tracet(k) = plot(nn,Pontilhado,'b--');
        hold on
    end
    xlim([0 nn(end)])
    
    grid on
end

hold off
set(ax(1),'YLim',[-5 15]);ylabel(ax(1),'Pos_X')
set(ax(2),'YLim',[-5 15]);ylabel(ax(2),'Pos_Y')
set(ax(3),'YLim',[-3.5 3.5]);ylabel(ax(3),'Psi')

set(ax(4),'YLim',[-2 2]);ylabel(ax(4),'VelocidadeLinear')
set(ax(5),'YLim',[-2 2]);ylabel(ax(5),'VelocidadeAngular')
set(ax(6),'YLim',[0.988 1.0]);ylabel(ax(6),'RQuadrado') 
 %Inicializa as vari�vel

PosicaoRobo = zeros(3,size(nn,2));
VelocidadeRobo = zeros(2,size(nn,2));
RQuadrado=zeros(1,size(nn,2));
kk = 1;
%% Inicializando par�metros para corre��o de reta
NumeroRetas=3; % Numero de retas que ir�o ser comparadas 
VariacaoMaxima=0.1; % Varia��o maxima aceita nos coeficiente em porcentagem
CoefTeste=zeros(NumeroRetas,1); % Cria vetor que armazenar� os coeficientes das retas
ContRetas=1; % Contador 
gatilho=0; % Gatilho para usar a reta ou n�o.
as=[];%Armazena o a das retas para analise futura
theta2=3;
% 

Xdest=[10 0];
Ydest=[8 0];
    %%
for II=1:size(Xdest,2)
    P.pPos.Xd([1 2 6]) = [Xdest(II) Ydest(II) 0]';
    P.pPos.Xtil(1:2) = [inf,inf];
    if II==2
        CaminhoVolta=size(PosicaoRobo,2)
    end
    while toc(t) < tmax && norm([Xdest(II)-P.pPos.X(1) Ydest(II)-P.pPos.X(2)]) > 0.01
        if(toc(ts) > 0.1)
            P.mLerDadosSensores;
            P.pPos.X = P.pPos.Xs;
            psi = P.pPos.X(6);
            pose = [P.pPos.X(1) P.pPos.X(2) P.pPos.X(6)];
            k = [cos(psi), -P.pPar.a*sin(psi); sin(psi), P.pPar.a*cos(psi)];
            kinv = inv(k);
            %L� Dados dos ultrassons
            for i = 0:7
                distancia = P.mLerSensorDistancia(i);
                medidas_sensor_sonar(2,i+1) = distancia;
            end
            %Insere os pontos lidos no grid
            %insertRay(og,pose,medidas_sensor_sonar(2,:),medidas_sensor_sonar(1,:),max_sensor_distancia_sonar);
            %Encontra a menor distancia lida dos ultrassons e armazena seu
            %�ndice na vari�vel pos
            [dmin,pos] = min(medidas_sensor_sonar(2,:));
            %Beta � o �ngulo da menor medidade dos sensores
            beta = medidas_sensor_sonar(1,pos);
            % Essa fun��o Varre um conjunto de sensores agrupando de 3 em 3
            % para encontrar os pontos no mundo que os sensores est�o lendo
            % e com isso utilizar polyfit praa encontrar uma reta que passe
            % nesses pontos. Avalia-se ent�o se essas retas s�o boas
            % utilizando o R�. O retorno dessa fun��o � todas as retas
            % encontradas ordenadas em rela��o ao R� delas,os coeficientes
            % da reta perpendicular � melhor reta encontrada e os pontos X
            % e Y no mundo da leitura dos sensores equivalentes � melhor
            % reta
            %Formato da RetasRQuadrado : [a,b,R�,distancia do rob� �
            %reta,sensor utilizado1, sensor utilizado2, sensor utilizado3]
            %Sendo a ultima linha a melhor reta encontrada(com o maior R�)
            [RetasRQuadrado,RetaPerpendicular,Xsensores,Ysensores] = paredeVirtual3(medidas_sensor_sonar,dobs,pos,theta2);
            %Esse if verifica se a melhor reta tem o R� maior que o minimo
            %requisitado, no caso de sim, atualiza-se o beta utilizado no
            %desvio tangencial para a inclina��o da reta perpendicular �
            %melhor reta encontrada
            %% Corre��o Das Retas
            % Pega o melhor coeficiente
            if ContRetas==NumeroRetas+1 % Reseta contador quando tiver no maximo
                ContRetas=1;
            end
            CoefTeste(ContRetas)=RetasRQuadrado(56,1);
            % Verifica se tem retas anormais
            for i=1:NumeroRetas-1
               if sum(abs(CoefTeste(i)./(CoefTeste((i+1):NumeroRetas)))-1<VariacaoMaxima)==NumeroRetas-i 
                   %Esse if esta dividindo o i�simo termo pontualmente por
                   %cada termo seguinte no array. Ent�o verifica se cada uma dessas divis�es menos 1
                   % ( em m�dulo) � menor que a varia��o m�xima. Isso retorna um arrray logico de 0
                   % ou 1, ent�o verifica a soma de todos eles, se for igual ao numero de retas
                   % analisadas, significa que todos foram true. Entao significa que pode usar as
                   % A ultima reta encontrada
                   gatilho=gatilho+1;
               else
                   gatilho=0;
               end
                    
                
                   
                
                
            end
            ContRetas=ContRetas+1;
            
            
            
            %%
           
            if gatilho>=NumeroRetas-1
                gatilho==0;%Reseta o gatilho
                if RetasRQuadrado(56,3)>RQuadradoMinimo
                    
                    Angulo=atan(RetaPerpendicular(1));% tomar cuidado com o /y
                    betav=Angulo;
                    as=[as RetasRQuadrado(56,1)];
                    SensoresUsados=[SensoresUsados; RetasRQuadrado(56,5:7) RetasRQuadrado(56,3)];
    %                 fprintf('Usando tangencial novo pela reta dos sensores %d, %d e %d, com RQuadrado de %d a uma distancia de %d\n',RetasRQuadrado(56,5:7),RetasRQuadrado(56,3:4)),;

                    %Aqui temos fun��es matem�ticas para poder adequadamente
                    %rotacionar osw pontos encontrados pela orienta��o do rob�
                    %e deslocamos eles em rela��o � posi��o do rob� para poder
                    %plotar essas retas em rela��o � refer�ncia global e n�o em
                    %rela��o � refer�ncia do rob�
                    kk1=[cos(psi),-sin(psi); sin(psi), cos(psi)];

                    Ynovo=polyval(RetasRQuadrado(56,1:2),Xsensores);
                    A=kk1*[Xsensores;Ynovo];
                    Xsensores=A(1,:);
                    Ynovo=A(2,:);
                    Xsensores=(Xsensores+P.pPos.X(1))';


                    Ynovo=(Ynovo+P.pPos.X(2))';
                    Xplot=[Xplot Xsensores];
                    Yplot=[Yplot Ynovo];

                    try 
                        delete(h)
                        
                    end
    %                 figure(3);
                    set(0, 'CurrentFigure', HH)
                    h=plot(Xplot(1:2,:),Yplot(1:2,:));  
%                     ylim([-5 20])
%                     xlim([-5 20])
%                     drawnow
%                     PP=plot(Xd(1),Xd(2),'x');
                    ylim([-2 13])
                    xlim([-2 13])
                    drawnow;
                else
                    %Se n�o tiver reta que chegue nos crit�rios almejados,
                    %utliza-se o beta que encontrou-se nos sensores do rob�
                    %para fazer o desvio tangencial
                    betav=beta;
    %                 fprintf(' Tangencial default\n');
                end
            else
                betav=beta;
            end
           
            Xd = P.pPos.Xd;
%             rad2deg(betav);
            [Xd,gamma,ult_pos] = VerificaDesvioTangencial(gamma,fe,dobs,dmin,betav,P.pPos.X,P.pPos.Xd,pos,ult_pos);
            xponto = 0;
            P.pPos.Xtil = Xd - P.pPos.X(1:2);
            fx = k1*tanh(k2 * P.pPos.Xtil(1:2));
            u = kinv * (xponto+fx);
            P.pSC.Ur = u;
            P.mEnviarSinaisControle;
            %Atualiza a figura do grid
%             mov_robo = [mov_robo; world2grid(og,[P.pPos.X(1) P.pPos.X(2)])];
%             matriz(:,:,1) = uint8(og.occupancyMatrix*255);
%             matriz(:,:,2) = uint8(og.occupancyMatrix*255);
%             matriz(:,:,3) = uint8(og.occupancyMatrix*255);
%             pos_robo = world2grid(og,[P.pPos.X(1) P.pPos.X(2)]);
%             pos_xd = world2grid(og,[Xd(1) Xd(2)]);
%             matriz(pos_robo(1),pos_robo(2),3) = 255;
%             matriz(pos_xd(1),pos_xd(2),2) = 255;
%             for i = 1:size(mov_robo,1)
%                 matriz(mov_robo(i,1),mov_robo(i,2),3) = 255;
%             end
%             h1.CData = imcomplement(matriz);
%             drawnow;
            % Eu tinha mais nada pra fazer ai to salvando informa��es agora
            % Salvando posi��o do rob� e psi
            PosicaoRobo=[PosicaoRobo,[P.pPos.X(1);P.pPos.X(2);P.pPos.X(6)]];
            VelocidadeRobo=[VelocidadeRobo,P.pSC.U];
            DadosRobo=[DadosRobo,P.pPos.X];
            SinaisControle=[SinaisControle,P.pSC.Ur];

            RQuadrado=[RQuadrado,RetasRQuadrado(56,3)];
            Sensores=[Sensores; RetasRQuadrado(56,5:7)];
            
            kk1=[cos(psi),-sin(psi); sin(psi), cos(psi)];
            %% Salvar os X lidos pelo rob� rotacionando eles
            for iii=1:size(medidas_sensor_sonar,2)
                %Esses s�o as coordenadas X e Y em rela��o ao robo
                XRobo(iii)=medidas_sensor_sonar(2,iii)*cos(medidas_sensor_sonar(1,iii));
                YRobo(iii)=medidas_sensor_sonar(2,iii)*sin(medidas_sensor_sonar(1,iii));
                DistanciaSensores=[DistanciaSensores medidas_sensor_sonar(2,iii)];
            end    
             Aux=kk1*[XRobo;YRobo];
             XLido=[XLido Aux(1,:)+P.pPos.X(1)];
             YLido=[YLido Aux(2,:)+P.pPos.X(2)];
             
                
            

            
            %% Atualiza a imagem com os dados da navega��o
            for ii = 1:2
%                 PosicaoRobo(ii,kk) = P.pPos.X(ii);
                trace(ii).YData = PosicaoRobo(ii,end-size(nn,2)+1:end);
            end
%                 PosicaoRobo(3,kk)=P.pPos.X(6);
                trace(3).YData = PosicaoRobo(3,end-size(nn,2)+1:end);
            for ii = 4:5
%                 VelocidadeRobo(ii-3,kk) = P.pSC.Ur(ii-3,1);
               
              trace(ii).YData = VelocidadeRobo(ii-3,end-size(nn,2)+1:end);
            end
                kk = kk+1;
               trace(6).YData = RQuadrado(1,end-size(nn,2)+1:end);
             drawnow;
%            hold off
        end
    end
end   
                %% Plotar o caminho com os pontos lidos
            figure(7)
            hold on
            for jjj=1:size(XLido,2)
               if mod(jjj,1)==0
                   if DistanciaSensores(jjj)<4
                        plot(XLido(jjj),YLido(jjj),'.k') 
                   end
                  
               end
            end
            
                plot(PosicaoRobo(1,1:CaminhoVolta),PosicaoRobo(2,1:CaminhoVolta),'b')
                plot(PosicaoRobo(1,CaminhoVolta:size(PosicaoRobo,2)),PosicaoRobo(2,CaminhoVolta:size(PosicaoRobo,2)),'--b')
             hold off
            
    %% Plotando caminho do Rob�
    ylim([-2 13])
    xlim([-2 13])
    save('PosicaoCorrecaoMapaAlvo.mat','PosicaoRobo', 'VelocidadeRobo')
    save('DadosRoboCorrecaoMapaAlvo.mat','DadosRobo')   
    save('SinaisCorrecaoMapaAlvo.mat','SinaisControle')
    save('Inclina��oParedeMapaAlvo.mat','as')
    save('RquadradoMapaAlvo.mat','RQuadrado')
    save('MapaCorretoMapaAlvo.mat','XLido','YLido','CaminhoVolta')
    save('DistanciaCorrecaoMapaAlvo.mat','DistanciaSensores')
    %% FInalizando a simula��o
    P.pSC.Ur(1) = 0;
    P.pSC.Ur(2) = 0;
    P.mEnviarSinaisControle;
    toc(t);

     
catch err
    disp(err.message);
end
