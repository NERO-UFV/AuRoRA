function [CB,dCB,Curva,INDICE] = CurvaBmbExp1(p0,n0,Bmb,Nb,Vb,Ub,Curva)
%CurvaBmb calcula caminho que atravessa bambolês
%   Detailed explanation goes here

    NN = 100; % Número de pontos para curva inicial
    Ts = 1/30; % Tempo de amostragem ArDrone
    dD = .4; % Raio do bambolê/drone

    figure(2)
    axis([-4 3 -4 3 0 3])
%     view(100,42)
    view(228.1000,49.1520)
    grid on
    xlabel('X [m]')
    ylabel('Y [m]')
    
    flag = 1;

    CB = [];    dCB = [];   ps = p0; % Iniciar curva
    
    for i = 1:size(Nb,1)
        if (i > 1)    % Reseta ponto de partida
            if flag
                n0 = -n;
                p0 = ps + dD.*(n./norm(n));
                
                flag =0;
            end
            na = n;
            ps = pfd;
            psd = ps -dD.*(na/norm(na));
            psd2 =psd -dD.*(na/norm(na));
            hold on
            plot3(psd(1),psd(2),psd(3),'^k','MarkerSize',5,'LineWidth',2)
        else
            psd = p0 +dD.*(n0/norm(n0));
            psd2 =psd -dD.*(n0/norm(n0));
            hold on
            plot3(psd(1),psd(2),psd(3),'^k','MarkerSize',5,'LineWidth',2)
        end 

        pf = Bmb(i,:);
        n  = Nb(i,:);
        V = Vb(i,:)/norm(Vb(i,:));
        U = Ub(i,:)/norm(Ub(i,:));

        pfd = pf +dD.*(n/norm(n)); % ponto final deslocado por segurança   
        pc = pfd +2*dD.*(n/norm(n)); % ponto de controle
        dp = pfd-ps(1,:); % ponto inicial ao ponto deslocado

        % Definição Retas:
        t = -1.5:0.1:1.5;
        L = t.*n'; % passa pelo bambolê
        R = t.*dp'; % do A ao bambolê
        z = t.*cross(R,L)+pfd'; % normal ao plano

        % Desenho dos pontos e retas ======================================
        % Ponto inicial:                     (PRETO)
        hold on
        plot3(ps(1),ps(2),ps(3),'ko')
        hold on
        plot3(ps(1),ps(2),ps(3),'k*')

        % Ponto final:                     (VERMELHO)
        hold on
        plot3(pf(1),pf(2),pf(3),'ro')
        hold on
        plot3(pf(1),pf(2),pf(3),'r*')

        % Ponto final deslocado:        (PRETO/VERMELHO)
        hold on
        plot3(pfd(1),pfd(2),pfd(3),'ko')
        hold on
        plot3(pfd(1),pfd(2),pfd(3),'r*')

        % Ponto de controle:                 (AZUL)
        hold on
        plot3(pc(1),pc(2),pc(3),'b*')
        hold on
        plot3(pc(1),pc(2),pc(3),'bo')

        % Normal do bambolê:
        hold on
        plot3(L(1,:)+pfd(1),L(2,:)+pfd(2),L(3,:)+pfd(3),'b-.','LineWidth',1)

        %  Do ArDrone ao bambolê:
        hold on
        plot3([pfd(1) ps(1)],[pfd(2) ps(2)],[pfd(3) ps(3)],'r:.','LineWidth',1.2)

        % Desenha bambolê: 
        u = 0:0.01:2*pi;
        XB = pf(1) +0.63.*cos(u).*V(1) +0.63*sin(u).*U(1);
        YB = pf(2) +0.63.*cos(u).*V(2) +0.63*sin(u).*U(2);
        ZB = pf(3) +0.63.*cos(u).*V(3) +0.63*sin(u).*U(3);
        hold on
        plot3(XB,YB,ZB,'LineWidth',2)
        grid on

        % Normal do bambolê:
        hold on
        plot3(L(1,:)+pfd(1),L(2,:)+pfd(2),L(3,:)+pfd(3),'b-.','LineWidth',1)

        % Do ArDrone ao bambolê:
        hold on
        plot3([pfd(1) ps(1)],[pfd(2) ps(2)],[pfd(3) ps(3)],'r:.','LineWidth',1.2)

        % Calcular e desenhar curva de Bèzier:
        ControlPoints = [ps;psd;pc;pfd];
        PlotControlPoints(ControlPoints);

        time = linspace(0,1,NN);
        B = bernsteinMatrix(3, time);
        bezierCurve = (B*ControlPoints);

        comp = 0;
        for kk = 1:size(bezierCurve,1)-1
            comp = comp + norm(bezierCurve(kk,:)-bezierCurve(kk+1,:));
        end
    %     comp
        N = floor(((comp)/Curva.Vmax)/Ts);
        B = bernsteinMatrix(3,  linspace(0,1,N));
        bezierCurve = (B*ControlPoints);

        CB = [CB(1:end-1,:); bezierCurve];
        
        if flag 
            INDICE = size(bezierCurve,1);
        end
        
        dCB = [dCB,DerivBezierCurve(ControlPoints',time)];
        hold on
        plot3(bezierCurve(:,1),bezierCurve(:,2),bezierCurve(:,3),'LineWidth',1.8)

        
        % Atravessar o bambolê:
        ps = pfd; psd = ps-dD.*(n/norm(n));  pfd = pf-dD.*(n/norm(n)); 
        ControlPoints = [ps;psd;pfd];         PlotControlPoints(ControlPoints);

        time = linspace(0,1,NN);
        B = bernsteinMatrix(2, time);
        bezierCurve = (B*ControlPoints);

        comp = 0;
        for kk = 1:size(bezierCurve,1)-1
            comp = comp + norm(bezierCurve(kk,:)-bezierCurve(kk+1,:));
        end
        N = floor(((comp)/Curva.Vmax)/Ts);
        B = bernsteinMatrix(2,  linspace(0,1,N));
        bezierCurve = (B*ControlPoints);

        CB = [CB(1:end-1,:); bezierCurve];
        dCB = [dCB,DerivBezierCurve(ControlPoints',time)];
        hold on
        plot3(bezierCurve(:,1),bezierCurve(:,2),bezierCurve(:,3),'LineWidth',1.8)

        if i==size(Nb,1) % Fechar curva:
            ControlPoints = [pfd;pfd-dD.*(n/norm(n));p0-dD.*(n0/norm(n0));p0;p0+dD.*(n0/norm(n0))];
            PlotControlPoints(ControlPoints);

            hold on
            plot3(pfd(1),pfd(2),pfd(3),'^k','MarkerSize',5,'LineWidth',2)

            time = linspace(0,1,NN);
            B = bernsteinMatrix(4, time);
            bezierCurve = (B*ControlPoints);

            comp = 0;
            for kk = 1:size(bezierCurve,1)-1
                comp = comp + norm(bezierCurve(kk,:)-bezierCurve(kk+1,:));
            end
            N = floor(((comp)/Curva.Vmax)/Ts);
            B = bernsteinMatrix(4,  linspace(0,1,N));
            bezierCurve = (B*ControlPoints);

            CB = [CB(1:end-1,:); bezierCurve];
            dCB = [dCB,DerivBezierCurve(ControlPoints',time)];
            hold on
            plot3(bezierCurve(:,1),bezierCurve(:,2),bezierCurve(:,3),'LineWidth',1.8)
            disp('Fechou a curva!')
        end
    end

    %% Calculo da derivada
    k = size(CB,1);      
    dCB =[];
    for ii = 2:size(CB,1)
        dCB = [dCB;(CB(ii,:)-CB(ii-1,:))./Ts];
    end

    Curva.rho = 0;       
    Curva.psi = atan2(dCB(:,2),dCB(:,1));
%     Curva.psi = atan2(dCB(2,:),dCB(1,:));
    Curva.X = CB';        
    Curva.dX = dCB';
%     Curva.dX = dCB;
    Curva.Pos = 1;       Curva.Size = k;
    Curva.dpsi = [0,diff(Curva.psi,1,2)];
    Curva.dXr = zeros(3,k);
end

