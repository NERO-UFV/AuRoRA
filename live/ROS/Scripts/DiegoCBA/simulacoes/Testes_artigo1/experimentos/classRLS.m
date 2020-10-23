classdef classRLS < handle
    properties
        n
        ganho
        linhas_g
        colunas_g
        V_f %entrada de controle
        Y% = G1(2:end); %tamanho Nx x n

        P %=zeros(n,n); %n: Numero de parámetros

        P0%=1000000*eye(n,n);
        T0 %=zeros(n,1);

        e        
        Error
        T
        contador
        NumCondP;   
    end
    methods
        
        function RLS = classRLS(n_parametros)            
            %n Numero de parametros            
            RLS.n = n_parametros;
            RLS.contador = 0;
            RLS.ganho = 0.1;
            %RLS.linhas_g = size(G,1);
            %RLS.colunas_g = size(G,2);
        end   
        
        function RLS = inicializa(RLS, u, G, condInicial)
        %disp('   Incializando');
            RLS.contador = 1;       
            RLS.linhas_g = size(G,1);
            RLS.P=zeros(RLS.n,RLS.n); %n: Numero de parámetros

            RLS.P0=1000000*eye(RLS.n,RLS.n);
            %disp(['nargin = ' num2str(nargin)]);
            if(nargin <= 3)
                RLS.T0=ones(RLS.n,1);
%                 RLS.T0=zeros(RLS.n,1);
%                 RLS.T0=2*ones(RLS.n,1); % Pacheco & vSA
            else
                RLS.T0=ones(RLS.n,1);
%                 RLS.T0=zeros(RLS.n,1);
                %RLS.T0= [1/condInicial(1)
                 %        condInicial(2)/condInicial(1)];
                    %Condicao inicial tem que ser um vetor nx1
                %condInicial é dado nos parametros do robo que obedece as
                %equacoes de theta1 = 1/k1; theta2 = k2/k1; ou theta2 =
                %k2*theta1;
            end
            
        %Condicao Inicial
        %disp('   Pronto para Estimar Parámetros')  
        
        
        
        RLS.Y{1} = G';
        RLS.e(:,1) = u' - RLS.Y{1}'*RLS.T0;
        RLS.Error(:,1)=RLS.ganho*(RLS.e(:,1).^2);
        RLS.T(:,1) = RLS.T0 + RLS.P0*RLS.Y{1}*inv(eye(RLS.linhas_g) + RLS.Y{1}'*RLS.P0*RLS.Y{1})*RLS.Error(:,1); %eye(1,1) 1 indica a qtd de linhas do regressor (matriz G)
        RLS.P = RLS.P0 - RLS.P0*RLS.Y{1}*inv(eye(RLS.linhas_g) + RLS.Y{1}'*RLS.P0*RLS.Y{1})*RLS.Y{1}'*RLS.P0;
        RLS.NumCondP(1)=cond(RLS.P);
        end
        
        function parametros_estimados = atualiza(RLS,uk,Gk)
            RLS.contador = RLS.contador +1;
            k = RLS.contador;
            
%             alpha= 0.98;
            alpha= 1;
            
            RLS.Y{k} = Gk';
            RLS.e(:,k) = uk' - RLS.Y{k}'*RLS.T(:,k-1);
            RLS.Error(:,k)=RLS.ganho*(RLS.e(:,k).^2) + RLS.Error(:,k-1);
            RLS.T(:,k) = RLS.T(:,k-1) + RLS.P*RLS.Y{k}*inv(alpha*eye(RLS.linhas_g) + RLS.Y{k}'*RLS.P*RLS.Y{k})*RLS.Error(:,k);
            RLS.P = RLS.P - RLS.P*RLS.Y{k}*inv(alpha*eye(RLS.linhas_g) + RLS.Y{k}'*RLS.P*RLS.Y{k} )*RLS.Y{k}'*RLS.P;
            RLS.NumCondP(k)=cond(RLS.P);
            
            parametros_estimados = RLS.T(:,k);
        end
    end
end