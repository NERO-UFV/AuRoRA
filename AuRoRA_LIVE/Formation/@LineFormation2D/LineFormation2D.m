classdef LineFormation2D < handle
% Establish a Line Formation in 2D
% FORMATION DESIGN ................................................
%  [Superior view]         
%                     
%                        __    
%                       /  \   
%                     ||    || Robot 2 (x2,y2)
%                      .`--´ 
%                    .°       
%        {Rho_f}   .°
%                .°       
%              .°
%       __   .°  ) {Alpha_f}
%      /  \    ----------------    
%    ||    || Robot 1 (x1,y1) 
%      `--´ 
% 
%  The center of formation can be: 
%  I  = robot 1                  >> LineFormation2D('robot')  
%  II = middle of formation line >> LineFormation2D('center')   [default]
% .................................................................
    properties
        pPos   % posturas dos robos
        pPar   % parâmetros do robô
        pSC    % sinal de controle
    end
    
    methods
        
        % Define tipo de formação:
        % 'center' = referência da formação é o ponto médio dos dois robôs
        % 'robot'  = referência da formação é o robô 1        
        function obj = LineFormation2D(type)
        % Caso não seja especificado o tipo, considera referência no ponto
        % médio dos robôs
            if nargin < 1
                type = 'center';
            elseif nargin>1
                disp('Defina apenas um tipo de formação ["center" ou "robot"].');
            end
            
            obj.pPar.Type = type;       % salva tipo de formação
            mInit(obj);                 % inicializa variáveis
        end
        
        mInit(obj)                      % Inicializa variáveis
        mFormationControl(obj);         % Controla formação
        mDirTrans(obj)                  % obtém variáveis de formação    
        mInvTrans(obj,par)              % obtém posição dos robôs 1 e 2 
        
    end
end