classdef ROSNetwork < handle
    % In a methods block, set the method attributes
    % and add the function signature
    properties
        %% Devices
        master
        node
        
        %% Publisher Topics objects
        Pubchatter
        Pubpose
        Pubvel
        Pubsonar
        Publaser
        
        %% Subscriber Topics objects
        Subchatter
        Subpose
        Subvel
        Subsonar
        Sublaser
        
        SubMsgchatter
        SubMsgpose
        SubMsgvel
        SubMsgsonar
        SubMsglaser
        
        %% Messages objects
        Msgchatter
        Msgpose
        Msgvel
        Msgsonar
        Msglaser
        
        map    
        device        
        pPar   % Parametros
        pSC    % Control Signal
        pPos   % Pose
    end
    
    properties (Access = private)
        scan
    end
    
    methods
        function obj = ROSNetwork                
            obj.pPos.X      = zeros(12,1);
            obj.pPos.Xd     = zeros(12,1);
            obj.pPos.Xtil   = obj.pPos.Xd - obj.pPos.X;
            obj.pPos.Xc     = zeros(12,1);
            obj.pPos.Xs     = zeros(12,1);
            obj.pSC.U       = [0;0];
            obj.pSC.Ur      = [0;0];
%             obj.pPar.a      = 0.15;
%             obj.pPar.Modelo = 'P3DX';
        end               
        
        
        %% Init ROS Network (Master or Node)
        InitROS(obj,NodeName,ip);
        PublisherROS(obj,Node,Topic);
        SubscriberROS(obj,Node,Topic);
        SendROS(obj,Topic,Msg);
        ReceivedMsg = ReceiveROS(obj,Topic);
        
        mDefinirPosturaInicial(obj,Xo);
        mLerDadosOdometria(obj);
        mConectarNodos(obj);
        mShutdown(obj);
        mDesconectar(obj);
        mEnviarSinaisControle(obj);
        
%         function con = verificaConexao(obj)
%             con = false;
%             if robotics.ros.internal.Global.isNodeActive
%                 if isNodeRunning(obj.dispositivo,'rosaria')
%                     con = true;
%                 end
%             end
%             obj.conn = con;
%         end
%         
%         function temNodo = mProcuraNodo(~,nome)
%             temNodo = false;
%             listaNodo = rosnode('list');
%             for i=1:size(listaNodo,2)
%                 if nome == string(listaNodo(i))
%                     temNodo = true;
%                 end
%             end
%         end
%         
%         function s = temLaser(obj)
%             s = obj.scan;
%         end     
%         
% %         function g = mLerMapa2d(obj)
% %             if obj.mProcuraNodo('/rtabmap/rtabmap') && obj.mProcuraNodo('/nodeMapa')
% %                 m  = receive(obj.mapa.sub);
% %                 occupancymatrix = [];
%                 for i=1:m.Info.Height
%                     for j=m.Info.Width:-1:1
%                         v = m.Data(((i-1)*m.Info.Width) + j);
%                         if v < 0
%                             v = 2;
%                         elseif v > 1
%                             v = 1;
%                         end
%                         occupancymatrix(i,j) = v;
%                     end
%                 end
%                 obj.mapa.grid2d.og = readBinaryOccupancyGrid(receive(obj.mapa.sub),55,1);
%                 obj.mapa.grid2d.mod = flipud(occupancymatrix);
%                 g = obj.mapa.grid2d;
%                 %obj.mapa.grid2d = flipud(occupancymatrix);
%                 %g = obj.mapa.grid2d;
%             end
%         end
    end
end