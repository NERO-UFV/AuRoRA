% Variáveis do Caminho
RaioX = 1.5;
RaioY = 1.5;
CentroX = 0;
CentroY = 0;
inc = 0.1; % Resolução do Caminho
nCaminho = 355; % Número de pontos total no caminho
s = 5:inc:nCaminho; % Abcissa curvilinea
x = -RaioX*sin(pi*s/180) + CentroX;
y = RaioY*cos(pi*s/180) + CentroY;
z = 0*ones(1,length(s));
C_normal = [x; y; z];

        hold on
        plot(C_normal(1,:),C_normal(2,:),'-k','LineWidth',2)
        plot(C_normal(1,1),C_normal(2,1),'*r')
        plot(C_normal(1,end),C_normal(2,end),'*r')
        txt_inicio = 'Path start \rightarrow';
        text(-1.55,1.55,txt_inicio,'FontSize',14)
        txt_fim = '\leftarrow Path end';
        text(0.3,1.55,txt_fim,'FontSize',14)
        dim = [.62 .52 .5 .1];
        str_x = 'R = 1.5 m';
        text(0.52,0.16,str_x,'FontSize',10)
        annotation('doublearrow',[0.53 0.75],[0.52 0.52]);
        axis([-2.5 2.5 -2 2])
        hold off