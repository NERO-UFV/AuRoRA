classdef AhrsYEIT < handle
    % In a methods block, set the method attributes
    % and add the function signature
    properties
        pTipo  		% ArDrone
        pPar   		% Parametros
        pPos   		% Postura
        pTempo 		% Variáveis de temporização
        pCAD
        pArquivo    % Arquivos de Dados
        pflag       % Flags para gerenciamento dos Dados
        pComm  		% Comunicação
    end
    methods
        function mAHRSInit(obj)
            %%
            obj.pTipo = 'AHRS';
            obj.pPos.Traco.index = 0;
        end
        function obj = mCriarFiguraTracoAng(obj,varargin)
            %%
            if nargin == 1
                tamanho = 500;
            elseif  nargin == 2
                tamanho = varargin{1};
            else
                tamanho = 500;
                fprintf('Comando Inválido\n');
            end
            obj.pCAD.TamanhoTraco = tamanho;
            nn = 1:obj.pCAD.TamanhoTraco;
            hh = figure(2);
            set(hh,'units','pix','pos',[5 100 1000 900],'PaperPositionMode','auto');
            ones(1,200);
            
            subplot(3,1,1),obj.pCAD.trace(1) = plot(nn,zeros(1,nn(end)),'r'); hold on
            obj.pCAD.traced(1) = plot(nn,zeros(1,nn(end)),'b--');
            plot(nn,90*ones(1,nn(end)),'g--');
            plot(nn,-90*ones(1,nn(end)),'g--');hold off
            ylim([-120 120]);
            grid on
            
            subplot(3,1,2),obj.pCAD.trace(2) = plot(nn,zeros(1,nn(end)),'r'); hold on
            obj.pCAD.traced(2) = plot(nn,zeros(1,nn(end)),'b--');
            plot(nn,90*ones(1,nn(end)),'g--');
            plot(nn,-90*ones(1,nn(end)),'g--');hold off
            ylim([-120 120]);
            grid on
            
            subplot(3,1,3),obj.pCAD.trace(3) = plot(nn,zeros(1,nn(end)),'r'); hold on
            obj.pCAD.traced(3) = plot(nn,zeros(1,nn(end)),'b--');
            plot(nn,90*ones(1,nn(end)),'g--');
            plot(nn,-90*ones(1,nn(end)),'g--');hold off
            ylim([-180 180]);
            grid on
            
            obj.pPos.Euler = zeros(obj.pCAD.TamanhoTraco,3);
        end
        function obj = mCriarFigura(obj)
            %% Cria o ambiente 3D com o bloco
            obj.pCAD.Figura(1) = figure(1);
            
            set(obj.pCAD.Figura(1),'units','pix','pos',[900 250 1000 700],'PaperPositionMode','auto');
            set(obj.pCAD.Figura(1),'renderer','opengl');
            % Criar mundo
            view(25,22);
            axis equal
            axis([-2 2 -2 2 -2 2]);
            grid on
            
        end
        function obj = mCriarBloco(obj)
            %%
            
            obj = mCriarFigura(obj);
            
            V = [0 0 0; 0 0 1; 0 1 1; 0 1 0; 2 0 1; 2 0 0; 2 1 0; 2 1 1];
            F = [1 2 3 4; 1 2 5 6; 5 6 7 8; 2 3 8 5; 1 4 7 6; 3 4 7 8];
            C = [.5 .5 .5;.5 .5 .5;.2 .2 .2;.8 .8 .8;.8 .8 .8; .5 .5 .5];
            
            obj.pCAD.virt = patch('Faces',F,'Vertices',V,'FaceVertexCData',C,'LineWidth',1.2);
            obj.pCAD.virt.FaceColor = 'flat';
            set(obj.pCAD.virt,'XData',obj.pCAD.virt.XData-1,'YData',obj.pCAD.virt.YData-0.5,'ZData',obj.pCAD.virt.ZData-0.5);
            
            % Angulos para Rotacionar
            obj.pPar.Eulerangs = [0 pi/2 0];
            obj.pPos.X(1:3) = [0 0 0];
            RotX = [1 0 0; 0 cos(obj.pPar.Eulerangs(1)) -sin(obj.pPar.Eulerangs(1)); 0 sin(obj.pPar.Eulerangs(1)) cos(obj.pPar.Eulerangs(1))];
            RotY = [cos(obj.pPar.Eulerangs(2)) 0 sin(obj.pPar.Eulerangs(2)); 0 1 0; -sin(obj.pPar.Eulerangs(2)) 0 cos(obj.pPar.Eulerangs(2))];
            RotZ = [cos(obj.pPar.Eulerangs(3)) -sin(obj.pPar.Eulerangs(3)) 0; sin(obj.pPar.Eulerangs(3)) cos(obj.pPar.Eulerangs(3)) 0; 0 0 1];
            Rot = RotZ*RotY*RotX;
            
            H = [Rot obj.pPos.X(1:3)'; 0 0 0 1];
            vertices = H*[obj.pCAD.virt.Vertices'; ones(1,size(obj.pCAD.virt.Vertices,1))];
            set(obj.pCAD.virt,'XData',obj.pCAD.virt.XData-1,'YData',obj.pCAD.virt.YData-0.5,'ZData',obj.pCAD.virt.ZData-0.5);
            obj.pCAD.virt.Vertices = vertices(1:3,:)';
            % Salva a base do bloco
            obj.pCAD.Base.Vertices = obj.pCAD.virt.Vertices;
            
        end
        function obj = mAtualizarBloco(obj)
            %%
            RotX = [1 0 0; 0 cos(obj.pPar.Eulerangs(1)) -sin(obj.pPar.Eulerangs(1)); 0 sin(obj.pPar.Eulerangs(1)) cos(obj.pPar.Eulerangs(1))];
            RotY = [cos(obj.pPar.Eulerangs(2)) 0 sin(obj.pPar.Eulerangs(2)); 0 1 0; -sin(obj.pPar.Eulerangs(2)) 0 cos(obj.pPar.Eulerangs(2))];
            RotZ = [cos(obj.pPar.Eulerangs(3)) -sin(obj.pPar.Eulerangs(3)) 0; sin(obj.pPar.Eulerangs(3)) cos(obj.pPar.Eulerangs(3)) 0; 0 0 1];
            Rot = RotZ*RotY*RotX;
            
            H = [Rot obj.pPos.X(1:3)'; 0 0 0 1];
            vertices = H*[obj.pCAD.Base.Vertices'; ones(1,size(obj.pCAD.virt.Vertices,1))];
            set(obj.pCAD.virt,'XData',obj.pCAD.virt.XData-1,'YData',obj.pCAD.virt.YData-0.5,'ZData',obj.pCAD.virt.ZData-0.5);
            obj.pCAD.virt.Vertices = vertices(1:3,:)';
        end
        function obj = mAtualizarTraco(obj)
            %%
            obj.pPos.Traco.index = obj.pPos.Traco.index + 1;
            
            obj.pPos.Euler(obj.pPos.Traco.index,1) = obj.pPar.Eulerangs(1)*180/pi;
            obj.pPos.Euler(obj.pPos.Traco.index,2) = obj.pPar.Eulerangs(2)*180/pi;
            obj.pPos.Euler(obj.pPos.Traco.index,3) = obj.pPar.Eulerangs(3)*180/pi;
            
            obj.pCAD.trace(1).YData = obj.pPos.Euler(end-obj.pCAD.TamanhoTraco+1:end,1)';
            obj.pCAD.trace(2).YData = obj.pPos.Euler(end-obj.pCAD.TamanhoTraco+1:end,2)';
            obj.pCAD.trace(3).YData = obj.pPos.Euler(end-obj.pCAD.TamanhoTraco+1:end,3)';
            
            %             obj.pCAD.traced(1).YData = phi(end-obj.pCAD.TamanhoTraco-1:end,1);
            %             obj.pCAD.traced(2).YData = theta(end-obj.pCAD.TamanhoTraco-1:end,2);
            %             obj.pCAD.traced(3).YData = psi(end-obj.pCAD.TamanhoTraco-1:end,3);
        end
        function obj = mIniciarCommSerialAHRS(obj)
            %% Inicializa a AHRS, habilitando a mesma para a captura.
            fprintf('Inicializando Comunicação Serial FTDI! \n');
            fprintf('Portas Serial dispiníveis %s \n',seriallist);
            obj.pComm.ObjSerial = serial('COM3','BaudRate',115200,'Timeout',.020); % Indica a porta e taxa utilizada
            fopen(obj.pComm.ObjSerial);
            obj.pComm.FlagRestart = 0;
            obj.pPar.Raw.Eulerangs = [];
            
            fprintf('Comunicação Bluetooth Inicializada! \n')
            flushinput(obj.pComm.ObjSerial);
        end
        function obj = mIniciarCommBluetoothAHRS(obj)
            %% Inicializa a AHRS, habilitando a mesma para a captura.
            fprintf('Inicializando Comunicação Bluetooth! \n');
            obj.pComm.ObjSerial = Bluetooth('IhBP_Bth_115200',1,'Timeout',.020); % Indica a porta e taxa utilizada
            fopen(obj.pComm.ObjSerial);
            obj.pComm.FlagRestart = 0;
            obj.pPar.Raw.Eulerangs = [];
            
            fprintf('Comunicação Bluetooth Inicializada! \n')
            flushinput(obj.pComm.ObjSerial);
            
            mAHRSCaptureEuler(obj);
            mAHRSCaptureEuler(obj);
            
        end
        function obj = mTerminarCommAHRS(obj)
            %% Fecha a comunicação Bluetooth
            fclose(obj.pComm.ObjSerial);
        end
        function obj = mAHRSCaptureQuaternions(obj)
            %% Recebe a string pela comunicação Bluetooth
            % Ordem dos dados
            % obj.pComm.ObjSerial('>0,0'); //Get tared orientation as quaternion
            % Returns the filtered, tared orientation estimate in quaternion form
            % Quaternion (float x4)
            % • Euler angles are always returned in pitch, yaw, roll order.
            % • For quaternions, data is always returned in x, y, z, w order.
            
            fprintf(obj.pComm.ObjSerial,'>0,0\n');
            obj.pPar.Raw.Quaternions = fscanf(obj.pComm.ObjSerial,'%f,%f,%f,%f\n')';
            flushinput(obj.pComm.ObjSerial);
            obj.pPar.Quaternions = [obj.pPar.Raw.Quaternions(4) obj.pPar.Raw.Quaternions(1) obj.pPar.Raw.Quaternions(2) obj.pPar.Raw.Quaternions(3)];
        end
        function obj = mAHRSCaptureEuler(obj)
            %% Recebe a string pela comunicação Bluetooth
            % obj.pComm.ObjSerial('>0,1'); // Get tared orientation as euler angles
            % Returns the filtered, tared orientation estimate in euler angle form
            % Euler Angles (float x3)
            % • Euler angles are always returned in pitch, yaw, roll order.
            % • For quaternions, data is always returned in x, y, z, w order.
            
            fprintf(obj.pComm.ObjSerial,'>0,1\n');
            obj.pPar.Raw.Eulerangs = fscanf(obj.pComm.ObjSerial,'%f,%f,%f\n');
            flushinput(obj.pComm.ObjSerial);
            
            obj.pPar.Eulerangs = [obj.pPar.Raw.Eulerangs(3) obj.pPar.Raw.Eulerangs(1) obj.pPar.Raw.Eulerangs(2)];
            
        end
        function obj = mGetEuler(obj)
            %% Recebe a string pela comunicação Bluetooth
            % obj.pComm.ObjSerial('>0,1'); // Get tared orientation as euler angles
            % Returns the filtered, tared orientation estimate in euler angle form
            % Euler Angles (float x3)
            % • Euler angles are always returned in pitch, yaw, roll order.
            % • For quaternions, data is always returned in x, y, z, w order.
            obj.pPar.Raw.Eulerangs = fscanf(obj.pComm.ObjSerial,'%f,%f,%f \n');
            flushinput(obj.pComm.ObjSerial);
            obj.pPar.Eulerangs = [obj.pPar.Raw.Eulerangs(3) obj.pPar.Raw.Eulerangs(1) obj.pPar.Raw.Eulerangs(2)];
        end
        function obj = mSetEulerDecompositionOrder(obj,varargin)
            %% Set Current Euler Decomposition Order
            % Sets the current euler angle decomposition order, which determines how the angles returned from
            % command 0x1 are decomposed from the full quaternion orientation. Possible values are
            % 0x0 for XYZ,
            % 0x1 for YZX,
            % 0x2 for ZXY,
            % 0x3 for ZYX,
            % 0x4 for XZY,
            % 0x5 for YXZ (default)
            if nargin == 1
                comando = 5;
            elseif  nargin == 2
                comando = varargin{1};
            else
                comando = 5;
                fprintf('Comando Inválido\n')
            end
            fprintf(obj.pComm.ObjSerial,['>0,16,',num2str(comando),'\n']); %
            obj.mGetEulerDecompositionOrder;
        end
        function obj = mGetEulerDecompositionOrder(obj,varargin)
            %% Set Current Euler Decomposition Order
            % Sets the current euler angle decomposition order, which determines how the angles returned from
            % command 0x1 are decomposed from the full quaternion orientation. Possible values are
            % 0x0 for XYZ,
            % 0x1 for YZX,
            % 0x2 for ZXY,
            % 0x3 for ZYX,
            % 0x4 for XZY,
            % 0x5 for YXZ (default)
            fprintf(obj.pComm.ObjSerial,'>0,156\n'); %
            resp = str2double(fscanf(obj.pComm.ObjSerial));
            if (resp == 0)
                fprintf('Euler Order: XYZ \n');
            elseif (resp==1)
                fprintf('Euler Order: YZX \n');
            elseif (resp==2)
                fprintf('Euler Order: ZXY \n');
            elseif (resp==3)
                fprintf('Euler Order: ZYX \n'); % Não
            elseif (resp==4)
                fprintf('Euler Order: XZY \n'); % Não
            elseif (resp==5)
                fprintf('Euler Order: YXZ (default) \n');
            end
        end
        function obj = mBeginGyroscopeCalibration(obj)
            %% Begin Gyroscope Calibration
            % Performs auto-gyroscope calibration. Sensor should remain still while samples are taken. The gyroscope
            % bias will be automatically placed into the bias part of the gyroscope calibration coefficient list.
            fprintf('Calibração do Giroscópio Iniciada \n');
            fprintf(obj.pComm.ObjSerial,'>0,165\n'); %
            pause(3)
            fprintf('Calibração do Giroscópio Concluída \n');
        end
        function obj = mBeginMICalibration(obj)
            %% Begin MI mode field calibration
            % Begins the calibration process for MI mode. The sensor should be left in a magnetically unperturbed
            % area for 3-4 seconds after this is called for calibration to succeed.
            fprintf('Calibração MI Iniciada\n');
            fprintf(obj.pComm.ObjSerial,'>0,114\n'); %
            pause(3)
            fprintf('Calibração Concluída \n');
        end
        function obj = mSetFiltermode(obj,varargin)
            %% Set Filter mode
            if nargin == 1
                comando = 1;
            elseif  nargin == 2
                comando = varargin{1};
            else
                comando = 1;
                fprintf('Comando Inválido\n');
            end
            fprintf(obj.pComm.ObjSerial,['>0,123,',num2str(comando),'\n']); % Set Filter mode
            obj.mGetFiltermode;
        end
        function obj = mGetFiltermode(obj)
            %% Set Filter mode
            fprintf(obj.pComm.ObjSerial,'>0,152\n'); % Get Filter mode
            Ahfilterflag = str2double(fscanf(obj.pComm.ObjSerial));
            if (Ahfilterflag == 1)
                fprintf('Filtro de Kalman selecionado\n');
            elseif (Ahfilterflag==2)
                fprintf('Filtro de Q-COMP selecionado\n');
            elseif  (Ahfilterflag == 3)
                fprintf('Filtro de Q-GRAD selecionado\n');
            end
            
        end
        function obj = mSetAxisDirections(obj,varargin)
            %% Set axis directions
            % 000: X: Right, Y: Up, Z: Forward (left-handed system, standard operation)
            % 001: X: Right, Y: Forward, Z: Up (right-handed system)
            % 002: X: Up, Y: Right, Z: Forward (right-handed system)
            % 003: X: Forward, Y: Right, Z: Up (left-handed system)
            % 004: X: Up, Y: Forward, Z: Right (left-handed system)
            % 005: X: Forward, Y: Up, Z: Right (right-handed system)
            % Bit 4: Positive/Negative Z (Third resulting component)
            % Bit 5: Positive/Negative Y (Second resulting component)
            % Bit 6: Positive/Negative X (First resulting component)
            if nargin == 1
                comando = 0;
            elseif  nargin == 2
                comando = varargin{1};
            else
                comando = 0;
                fprintf('Comando Inválido\n');
            end
            fprintf(obj.pComm.ObjSerial,['>0,116,',num2str(comando),'\n']); %
            obj.mGetAxisDirections;
        end
        function obj = mGetAxisDirections(obj)
            %% Set axis directions
            % 000: X: Right, Y: Up, Z: Forward (left-handed system, standard operation)
            % 001: X: Right, Y: Forward, Z: Up (right-handed system)
            % 002: X: Up, Y: Right, Z: Forward (right-handed system)
            % 003: X: Forward, Y: Right, Z: Up (left-handed system)
            % 004: X: Up, Y: Forward, Z: Right (left-handed system)
            % 005: X: Forward, Y: Up, Z: Right (right-handed system)
            % Bit 4: Positive/Negative Z (Third resulting component)
            % Bit 5: Positive/Negative Y (Second resulting component)
            % Bit 6: Positive/Negative X (First resulting component)
            fprintf(obj.pComm.ObjSerial,'>0,143\n'); %
            resp = str2double(fscanf(obj.pComm.ObjSerial));
            if (resp == 0)
                fprintf('X: Right, Y: Up, Z: Forward (Default)\n');
            elseif (resp==1)
                fprintf('X: Right, Y: Forward, Z: Up \n');
            elseif (resp==2)
                fprintf('X: Up, Y: Right, Z: Forward \n');
            elseif (resp==3)
                fprintf('X: Forward, Y: Right, Z: Up \n');
            elseif (resp==4)
                fprintf('X: Up, Y: Forward, Z: Right \n');
            elseif (resp==5)
                fprintf('X: Forward, Y: Up, Z: Right \n');
            end
        end
        function obj = mSetRunningAverageMode(obj,varargin)
            %% Running Average Mode
            % Used to further smooth out the orientation at the cost of higher latency. Passing in a parameter of 0 places
            % the sensor into a static running average mode, a 1 places the sensor into a confidence-based running
            % average mode, which changes the running average factor based upon the confidence factor, which is a
            % measure of how 'in motion' the sensor is.
            % 0 - static running average mode,
            % 1 - confidence-based running average mode
            if nargin == 1
                comando = 0;
            elseif  nargin == 2
                comando = varargin{1};
            else
                comando = 0;
                fprintf('Comando Inválido\n');
            end
            fprintf(obj.pComm.ObjSerial,['>0,124,',num2str(comando),'\n']);  % Set Running Average Mode
            obj.mGetRunningAverageMode;
        end
        function obj = mGetRunningAverageMode(obj)
            %% Running Average Mode
            % Used to further smooth out the orientation at the cost of higher latency. Passing in a parameter of 0 places
            % the sensor into a static running average mode, a 1 places the sensor into a confidence-based running
            % average mode, which changes the running average factor based upon the confidence factor, which is a
            % measure of how 'in motion' the sensor is.
            % 0 - static running average mode,
            % 1 - confidence-based running average mode
            fprintf(obj.pComm.ObjSerial,'>0,153\n'); % Get Running Average Mode
            fprintf(['Running Average Mode: ',num2str(fscanf(obj.pComm.ObjSerial)),'\n']);
        end
        function obj = mResetBaseOffset(obj)
            %% % Reset Base Offset
            % Sets the base offset to an identity quaternion.
            fprintf(obj.pComm.ObjSerial,'>0,20\n'); % Reset Base Offset
        end
        function obj = mSetBaseOffset(obj)
            %% % Reset Base Offset
            % Sets the offset orientation to be the same as the supplied orientation,
            % which should be passed as a quaternion.
            %
            % 1) Place the sensor as close as possible to the mounting point, but in an orientation aligned with the overall
            % vehicle or device the sensor is being mounted on, or in the orientation that you would like the sensor to act like
            % it is in.
            % 2) Call command 22, which sets a hidden variable called the “base offset” which affects the operation of the
            % “Offset with current orientation” command. This will record your desired orientation later. If you ever want to
            % reset this base offset, use command 20(0x14).
            % 3) Mount the sensor onto the vehicle or device as you intend to for the end application.
            % 4) Call command 19(0x13), which will set the offset based on the difference between the current orientation and
            % the base offset. After this command is called, the sensor should now be acting as though it were in the desired
            % orientation.
            % 5) Make sure to commit the sensor settings to keep this change. Note that the base offset is not committable, but
            % the offset itself is committable.
            fprintf(obj.pComm.ObjSerial,'>0,22\n'); % Reset Base Offset
        end
        function obj = mSetCurrentOffset(obj)
            %% Offset with current orientation
            % Sets the offset orientation to be the same as the current filtered orientation.
            fprintf(obj.pComm.ObjSerial,'>0,19\n'); % Offset with current orientation
        end
        function obj = mGetOrientationVector(obj)
            %% Forward Vector and Down Vector
            % Forward Vector (float x3)
            % Down Vector (float x3)
            fprintf(obj.pComm.ObjSerial,'>0,11\n'); % Get tared two vector in sensor frame
            fprintf(['Tared two vector in sensor frame: \n',num2str(fscanf(obj.pComm.ObjSerial)),'\n']);
            fprintf(obj.pComm.ObjSerial,'>0,10\n'); % Get offset orientation as quaternion
            fprintf(['Untared two vector in sensor frame \n',num2str(fscanf(obj.pComm.ObjSerial)),'\n']);
        end
        function obj = mSetOversampleRate(obj,Rate)
            %% Oversample rate
            % Sets the number of times to sample each component sensor for each iteration of the filter. This
            % can smooth out readings at the cost of responsiveness. If this value is set to 0 or 1, no
            % oversampling occurs—otherwise, the number of samples per iteration depends on the specified
            % parameter, up to a maximum of 65535.
            
            fprintf(obj.pComm.ObjSerial,['>0,106,',Rate,'\n']); % Set Sample Rate
            obj.mGetOversampleRate;
        end
        function obj = mGetOversampleRate(obj)
            %% Oversample rate
            fprintf(obj.pComm.ObjSerial,'>0,144\n');
            resp = str2double(fscanf(obj.pComm.ObjSerial));
            fprintf(['Oversample Rate: ', num2str(resp),'\n']);
        end
        function obj = mSetCompassTrustValues(obj,Value)
            %% Compass Trust Values
            % Determines how trusted the compass contribution is to the overall orientation estimation. Instead of using
            % a single value, uses a minimum and maximum value. Trust values will be selected from this range
            % depending on the confidence factor. This can have the effect of smoothing out the compass when the
            % sensor is in motion.
            
            fprintf(obj.pComm.ObjSerial,['>0,102,',Value,'\n']); % Set confidence compass trust values
            obj.mGetCompassTrustValues;
        end
        function obj = mGetCompassTrustValues(obj)
            %% Get Compass Trust Values
            % Returns the current compass min and max trust values.
            % If static trust values were set, both of these will be the same.

            fprintf(obj.pComm.ObjSerial,'>0,131\n'); %
            resp = fscanf(obj.pComm.ObjSerial);
            fprintf(['Compass Trust Values: ', num2str(resp(1:end-2)),'\n']);
        end
        function obj = mSetCompassEnable(obj,varargin)
            %% Compass enabled state
            
            if nargin == 1
                State = 1;
            elseif  nargin == 2
                State = varargin{1};
            else
                State = 1;
                fprintf('Comando Inválido\n')
            end
            fprintf(obj.pComm.ObjSerial,['>0,109,',num2str(State),'\n']); % Set compass enabled (0 - Disable)
            obj.mGetCompassEnable;
        end
        function obj = mGetCompassEnable(obj)
            %% Compass enabled state
            % Returns a value indicating whether the compass contribution is currently part of the orientation estimate:
            % 0 for off, 1 for on.
            
            fprintf(obj.pComm.ObjSerial,'>0,142\n'); %
            resp = str2double(fscanf(obj.pComm.ObjSerial));
            if (resp == 1)
                fprintf('Compass Enable \n')
            elseif (resp==0)
                fprintf('Compass Disable \n')
            end
        end
        function obj = mSetCompassRange(obj,varargin)
            %% Set compass range
            % Only parameter is the new compass range, which can be 0 for ±0.88G, 1 for ±1.3G (Default range), 2
            % for ±1.9G, 3 for ±2.5G, 4 for ±4.0G, 5 for ±4.7G, 6 for ±5.6G, or 7 for ±8.1G. Higher ranges can detect and
            % report larger magnetic field strengths but are not as accurate for smaller magnetic field strengths.
            
            if nargin == 1
                State = 1;
            elseif  nargin == 2
                State = varargin{1};
            else
                State = 1;
                fprintf('Comando Inválido\n')
            end
            fprintf(obj.pComm.ObjSerial,['>0,126,',num2str(State)'\n']); %
            fprintf(obj.pComm.ObjSerial,'>0,155\n'); %
            
            resp = str2double(fscanf(obj.pComm.ObjSerial));
            if (resp == 0)
                fprintf('Compass Range: ±0.88G \n')
            elseif (resp==1)
                fprintf('Compass Range: ±1.3G (Default range)\n')
            elseif (resp==2)
                fprintf('Compass Range: ±1.9G \n')
            elseif (resp==3)
                fprintf('Compass Range: ±2.5G \n')
            elseif (resp==4)
                fprintf('Compass Range: ±4.0G \n')
            elseif (resp==5)
                fprintf('Compass Range: ±4.7G \n')
            elseif (resp==6)
                fprintf('Compass Range: ±5.6G \n')
            elseif (resp==7)
                fprintf('Compass Range: ±8.1G \n')
            end
        end
        function obj = mSetCalibrationMode(obj,varargin)
            %% Calibration Mode
            % Sets the current calibration mode, which can be 0 for Bias or 1 for Scale-Bias. For more information, refer
            % to the Calibration Modes section. This setting can be saved to non-volatile flash memory using the Commit
            % Settings command.
            % 0 for Bias
            % 1 for Scale-Bias.
            
            if nargin == 1
                State = 0;
            elseif  nargin == 2
                State = varargin{1};
            else
                State = 0;
                fprintf('Comando Inválido\n')
            end
            fprintf(obj.pComm.ObjSerial,['>0,169,',num2str(State),'\n']); %
            fprintf(obj.pComm.ObjSerial,'>0,170\n'); %
            resp = str2double(fscanf(obj.pComm.ObjSerial));
            if (resp == 0)
                fprintf('Scale-Bias Calibration Mode \n')
            elseif (resp==1)
                fprintf('Scale-Bias Calibration Mode \n')
            end
        end
        function obj = mGetCalibrationMode(obj)
            %% Calibration Mode
            % Sets the current calibration mode, which can be 0 for Bias or 1 for Scale-Bias. For more information, refer
            % to the Calibration Modes section. This setting can be saved to non-volatile flash memory using the Commit
            % Settings command.
            % 0 for Bias
            % 1 for Scale-Bias.
            fprintf(obj.pComm.ObjSerial,'>0,170\n'); %
            resp = str2double(fscanf(obj.pComm.ObjSerial));
            if (resp == 0)
                fprintf('Scale-Bias Calibration Mode \n')
            elseif (resp==1)
                fprintf('Scale-Bias Calibration Mode \n')
            end
        end
        function obj = mTaredWithCurrentOrientation(obj)
            %% Tare with current orientation
            % Sets the tare orientation to be the same as the current filtered orientation.
            fprintf(obj.pComm.ObjSerial,'>0,96\n');
        end
        function obj = mSetLedColor(obj,cor)
            %%
            fprintf(obj.pComm.ObjSerial,['>0,238,',cor,'\n']); % Set Led Color
            obj.mGetLedColor;
        end
        function obj = mGetLedColor(obj)
            %%
            fprintf(obj.pComm.ObjSerial,'>0,239\n'); % Get Filter mode % Get Led Color
            Resp = fscanf(obj.pComm.ObjSerial);
            fprintf(['Cor do Led:',Resp(1:end-1)]);
        end
        function obj = mCustomCommand(obj,Comando)
            %%
            fprintf(obj.pComm.ObjSerial,['>0,',Comando,'\n']); % Get Filter mode % Get Led Color
            Resp = fscanf(obj.pComm.ObjSerial);
            fprintf(Resp);
        end
        function obj = mSetBaudRate(obj,BaudRate)
            %%
            % Sets the baud rate of the physical UART. This setting does not need to be committed, but will not
            % take effect until the sensor is reset. 
            % Valid baud rates are 1200, 2400, 4800, 9600, 19200, 28800, 38400, 57600, 115200 (default), 230400, 460800 and 921600. 
            fprintf(obj.pComm.ObjSerial,['>0,231',BaudRate,'\n']); % Get Filter mode % Get Led Color
        end
        function obj = mGetBaudRate(obj)
            %%
            % Sets the baud rate of the physical UART. This setting does not need to be committed, but will not
            % take effect until the sensor is reset. 
            % Valid baud rates are 1200, 2400, 4800, 9600, 19200, 28800, 38400, 57600, 115200 (default), 230400, 460800 and 921600.
            fprintf(obj.pComm.ObjSerial,'>0,232\n'); % Get Filter mode % Get Led Color
            Resp = fscanf(obj.pComm.ObjSerial);
            fprintf(['Baud Rate:',Resp(1:end-1)]);
        end
        function obj = mSetStreamingSlots(obj,command)
            %% 
            % Configures data output slots for streaming mode. Command accepts a list of eight bytes, where each byte corresponds to a different data command.
            % Every streaming iteration, each command will be executed in order and the resulting data will be output in the specified slot.
            % Valid commands are commands in the ranges 0x0 – 0x10, 0x20 – 0x30, 0x40 – 0x50, 0xC9 – 0xCA (for battery-powered sensors) and 0xFA.
            % A slot value of 0xFF 'clears' the slot and prevents any data from being written in that position.
            % This command can fail if there is an invalid command passed in as any of the parameters or if the total
            % allotted size is exceeded. Upon failure, all slots will be reset to 0xFF.
            % 
            fprintf(obj.pComm.ObjSerial,['>0,80,',num2str(command),',255,255,255,255,255,255,255\n']); % Get Filter mode % Get Led Color
            mGetStreamingSlots(obj);
        end
        function obj = mGetStreamingSlots(obj)
            %% Returns the current streaming slots configuration.
            fprintf(obj.pComm.ObjSerial,'>0,81\n'); % Get Filter mode % Get Led Color
            Resp = fscanf(obj.pComm.ObjSerial);
            fprintf(['Streaming Slots: ',Resp(1:end-1)]);
        end
        function obj = mSetStreamingTimming(obj,interval,duration,delay)
            %%
            % Configures timing information for a streaming session. All parameters are specified in microseconds. 
            % The first parameter is the interval, which specifies how often data will be output. 
            % A value of 0 means that data will be output at the end of every filter loop. Aside from 0, values lower than 1000 will be clamped to 1000. 
            % The second parameter is the duration, which specifies the length of the streaming session. 
            % If this value is set to 0xFFFFFFFF, streaming will continue indefinitely until it is stopped via command 0x56. 
            % The third parameter is the delay, which specifies a n amount of time the sensor will wait before outputting the first packet of streaming data. 
            % >0,82,10000,4294967295,200000
            fprintf(obj.pComm.ObjSerial,['>0,82,',num2str(interval*1000),',',num2str(duration*1000),',',num2str(delay*1000),'\n']); % Get Filter mode % Get Led Color
            mGetStreamingTimming(obj);
        end
        function obj = mGetStreamingTimming(obj)
            %% Returns the current streaming timing information.
            fprintf(obj.pComm.ObjSerial,'>0,83\n'); % Get Filter mode % Get Led Color
            Resp = fscanf(obj.pComm.ObjSerial);
            fprintf(['Streming Timming:',Resp(1:end-1)]);
        end 
        function obj = mStartStreaming(obj)
            %% Start a streaming session using the current slot and timing configuration.
            fprintf(obj.pComm.ObjSerial,'>0,85\n'); % Get Filter mode % Get Led Color
        end
        function obj = mStopStreaming(obj)
            %% Start a streaming session using the current slot and timing configuration.
            fprintf(obj.pComm.ObjSerial,'>0,86\n'); % Get Filter mode % Get Led Color
        end   
        
    end
    
end
