function PlotDataFiles()
%% Plotar Gráficos de Experimento ou Simulação
%%
% try cd('D:\Dropbox\AuRoRA 2018\DataFiles'); end
% try cd('I:\Dropbox\AuRoRA 2018\DataFiles'); end
close all
try cd('DataFiles'); end

addpath(genpath(pwd));
DataFile = InitFiles();
[DataBank,FlagData]= FileOrganizer(DataFile);
Menu = creategui;

%% -------------------------------------------------------------------------------------------------------------
%  -------------------------------------------------------------------------------------------------------------
%  ---------                                    Carregar arquivos                                      ---------
%  -------------------------------------------------------------------------------------------------------------
%  -------------------------------------------------------------------------------------------------------------
    function DataFile = InitFiles()
        
        Menu.MainFolder = pwd;
        
        Init.d = dir;
        DataFolder = {Init.d.name};
        [Init.s,Init.v] = listdlg('PromptString','Selecione a pasta:',...
		            'ListSize',[330 600],...
            'SelectionMode','multiple',...
            'ListString',DataFolder);
        cd(DataFolder{Init.s})
        Init.d = dir;
        DataFolder = {Init.d.name};
        [Init.s,Init.v] = listdlg('PromptString','Selecione a pasta:',...
		            'ListSize',[330 600],...
            'SelectionMode','multiple',...
            'ListString',DataFolder);
        cd(DataFolder{Init.s})
        
        Menu.PastadeArquivos = pwd;
        
        Init.d = dir;
        DataFile = {Init.d.name};
        
        % cd(Menu.MainFolder)
    end
%% -------------------------------------------------------------------------------------------------------------
%  -------------------------------------------------------------------------------------------------------------
%  ---------                               Organizador dos aquivos                                     ---------
%  -------------------------------------------------------------------------------------------------------------
%  -------------------------------------------------------------------------------------------------------------

    function [DataBank, FlagData] = FileOrganizer(DataFile)
        
        AvailableColors =[  0               0           1.0000;
            1.0000          0           0;
            0.7             1.0000      0.1;
            0               0           0.1724;
            1.0000          0.1034      0.7241;
            1.0000          0.8276      0;
            0               0.3448      0;
            0.5172          0.5172      1.0000;
            0.6207          0.3103      0.2759;
            0               1.0000      0.7586];
        
        FlagData.Video.Index = 0;
        FlagData.Enable.Gains =     'Off';
        FlagData.Enable.Video =     'Off';
        FlagData.Enable.DDPlot =    'Off';
        FlagData.Enable.DDDPlot =   'Off';
        FlagData.Enable.Coord =     'Off';
        FlagData.Enable.Angles =    'Off';
        FlagData.Enable.Vel =       'Off';
        FlagData.Enable.DX =        'Off';
        FlagData.Enable.AngVel =    'Off';
        FlagData.Enable.XErr =      'Off';
        FlagData.Enable.AngErr =    'Off';
        FlagData.Enable.Acc =       'Off';
        FlagData.Enable.Gyro =      'Off';
        FlagData.Enable.Pwm =       'Off';
        FlagData.Enable.Forces =    'Off';
        FlagData.Enable.XForces =   'Off';
        FlagData.nFiles = size(DataFile,2)-2;
        
        %%
        idx = 0;
        for ki=1:size(DataFile,2) % Varre arquivos da pasta de dados
            if ~isempty(strfind(DataFile{ki},'.txt'))
                idx = idx + 1;
                DataBank{idx} = importdata(DataFile{ki});
                try
                Header = textscan(DataBank{idx}.textdata{1},'%s');
                catch
                    error('The log file must have a text header.')
                end
                DataBank{idx}.Header = Header{1}';
                posunderline = strfind(DataFile{ki},'_');
                postrace = strfind(DataFile{ki},'-');
                
                DataBank{idx}.FileName = DataFile{ki};
                
                if ~isempty(postrace) % If is not empty, it means it has more then one vehicle
                    DataBank{idx}.Type = DataFile{ki}(posunderline(1)+1:postrace(1)-1);
                    DataBank{idx}.Index = str2double(DataFile{ki}(postrace(1)+1:posunderline(2)-1));
                else
                    DataBank{idx}.Type = DataFile{ki}(posunderline(1)+1:posunderline(2)-1);
                    DataBank{idx}.Index = 1;
                end
                
                if size(posunderline,2) == 3 % There is a Comment
                    DataBank{idx}.Comment = DataFile{ki}(posunderline(3)+1:end-4);
                    FlagData.Indentifier = DataBank{idx}.Comment;
                    DataBank{idx}.TimeStamp = DataFile{ki}(posunderline(2)+1:posunderline(3)-1);
                else
                    DataBank{idx}.TimeStamp = DataFile{ki}(posunderline(2)+1:end-4);
                    FlagData.Indentifier = DataBank{idx}.TimeStamp(end-3:end);
                end
                DataBank{idx}.Color = AvailableColors(idx,:); 
                DataBank{idx}.FileIndex = ki;
            elseif ~isempty(strfind(DataFile{ki},'.mp4')) % Busca pelo Video
                FlagData.Video.Name = DataFile{ki};
                FlagData.Video.Index = ki;
                FlagData.Video.Enable = 'On';
            end
        end
        %%
        for idy = 1:size(DataBank,2)
            
            DataBank{idy}.Have.Desird = 0;
            DataBank{idy}.Have.AngDesird = 0;
            
            DataBank{idy}.Have.XVel = 0;
            DataBank{idy}.Have.Xtil = 0;
            DataBank{idy}.Have.Acc = 0;
            DataBank{idy}.Have.Pwm = 0;
            
            DataBank{idy}.Have.Angles = 0;
            DataBank{idy}.Have.AngVel = 0;
            DataBank{idy}.Have.AngErr = 0;
            DataBank{idy}.Have.Gyro = 0;
            
            DataBank{idy}.Have.PForces = 0;
            DataBank{idy}.Have.XForces = 0;
            
            for idx = 1:size(DataBank{idy}.Header,2)
                switch DataBank{idy}.Header{idx}
                    case 't'
                        DataBank{idy}.Var.t = DataBank{idy}.data(:,idx);
                    case 'X'
                        FlagData.Enable.Coord =   'on';
                        FlagData.Enable.DDPlot =  'on';
                        FlagData.Enable.DDDPlot = 'On';
                        DataBank{idy}.Var.X(:,1) = DataBank{idy}.data(:,idx);
                    case 'Y'
                        DataBank{idy}.Var.X(:,2) = DataBank{idy}.data(:,idx);
                    case 'Z'
                        DataBank{idy}.Var.X(:,3) = DataBank{idy}.data(:,idx);
                    case 'Xd'
                        DataBank{idy}.Have.Desird = 1;
                        FlagData.Enable.Coorddis = 'on';
                        DataBank{idy}.Var.X(:,4) = DataBank{idy}.data(:,idx);
                    case 'Yd'
                        DataBank{idy}.Var.X(:,5) = DataBank{idy}.data(:,idx);
                    case 'Zd'
                        DataBank{idy}.Var.X(:,6) = DataBank{idy}.data(:,idx);
                    case 'Xtil'
                        DataBank{idy}.Have.Xtil = 1;
                        FlagData.Enable.XErr = 'on';
                        DataBank{idy}.Var.X(:,7) = DataBank{idy}.data(:,idx);
                    case 'Ytil'
                        DataBank{idy}.Var.X(:,8) = DataBank{idy}.data(:,idx);
                    case 'Ztil'
                        DataBank{idy}.Var.X(:,9) = DataBank{idy}.data(:,idx);
                        %%%%%%
                    case 'Vx'
                        DataBank{idy}.Have.XVel = 1;
                        FlagData.Enable.Vel = 'on';
                        DataBank{idy}.Var.X(:,10) = DataBank{idy}.data(:,idx);
                    case 'Vy'
                        DataBank{idy}.Var.X(:,11) = DataBank{idy}.data(:,idx);
                    case 'Vz'
                        DataBank{idy}.Var.X(:,12) = DataBank{idy}.data(:,idx);
                    case 'Phi'
                        DataBank{idy}.Have.Angles = 1;
                        FlagData.Enable.Angles = 'On';
                        DataBank{idy}.Var.Ang(:,1) = DataBank{idy}.data(:,idx);
                    case 'Theta'
                        DataBank{idy}.Var.Ang(:,2) = DataBank{idy}.data(:,idx);
                    case 'Psi'
                        DataBank{idy}.Var.Ang(:,3) = DataBank{idy}.data(:,idx);
                    case 'phid'
                        DataBank{idy}.Have.AngDesird = 1;
                        DataBank{idy}.Var.Ang(:,4) = DataBank{idy}.data(:,idx);
                    case 'Thetad'
                        DataBank{idy}.Var.Ang(:,5) = DataBank{idy}.data(:,idx);
                    case 'Psid'
                        DataBank{idy}.Var.Ang(:,6) = DataBank{idy}.data(:,idx);
                    case 'Phitil'
                        DataBank{idy}.Have.AngErr = 1;
                        FlagData.Enable.AngErr = 'On';
                        DataBank{idy}.Var.Ang(:,7) = DataBank{idy}.data(:,idx);
                    case 'Thetatil'
                        DataBank{idy}.Var.Ang(:,8) = DataBank{idy}.data(:,idx);
                    case 'Psitil'
                        DataBank{idy}.Var.Ang(:,9) = DataBank{idy}.data(:,idx);
                    case 'dPhi'
                        DataBank{idy}.Have.AngVel = 1;
                        FlagData.Enable.AngVel = 'On';
                        DataBank{idy}.Var.Ang(:,10) = DataBank{idy}.data(:,idx);
                    case 'dTheta'
                        DataBank{idy}.Var.Ang(:,11) = DataBank{idy}.data(:,idx);
                    case 'dPsi'
                        DataBank{idy}.Var.Ang(:,12) = DataBank{idy}.data(:,idx);
                        
                    case 'Accx'
                        DataBank{idy}.Have.Acc = 1;
                        FlagData.Enable.Acc = 'On';
                        DataBank{idy}.Var.Sensor(:,1) = DataBank{idy}.data(:,idx);
                    case 'Accy'
                        DataBank{idy}.Var.Sensor(:,2) = DataBank{idy}.data(:,idx);
                    case 'Accz'
                        DataBank{idy}.Var.Sensor(:,3) = DataBank{idy}.data(:,idx);
                        
                    case 'Gyrox'
                        DataBank{idy}.Have.Gyro = 1;
                        FlagData.Enable.Gyro = 'On';
                        DataBank{idy}.Var.Sensor(:,4) = DataBank{idy}.data(:,idx);
                    case 'Gyroy'
                        DataBank{idy}.Var.Sensor(:,5) = DataBank{idy}.data(:,idx);
                    case 'Gyroz'
                        DataBank{idy}.Var.Sensor(:,6) = DataBank{idy}.data(:,idx);
                        
                    case 'pwm1'
                        DataBank{idy}.Have.Pwm = 1;
                        FlagData.Enable.Pwm = 'On';
                        DataBank{idy}.Var.Sensor(:,7) = DataBank{idy}.data(:,idx);
                    case 'pwm2'
                        DataBank{idy}.Var.Sensor(:,8) = DataBank{idy}.data(:,idx);
                    case 'pwm3'
                        DataBank{idy}.Var.Sensor(:,9) = DataBank{idy}.data(:,idx);
                    case 'pwm4'
                        DataBank{idy}.Var.Sensor(:,10) = DataBank{idy}.data(:,idx);
                        
                    case 'f1'
                        DataBank{idy}.Have.PForces = 1;
                        FlagData.Enable.Forces = 'On';
                        DataBank{idy}.Var.Forces(:,1) = DataBank{idy}.data(:,idx);
                    case 'f2'
                        DataBank{idy}.Var.Forces(:,2) = DataBank{idy}.data(:,idx);
                    case 'f3'
                        DataBank{idy}.Var.Forces(:,3) = DataBank{idy}.data(:,idx);
                    case 'f4'
                        DataBank{idy}.Var.Forces(:,4) = DataBank{idy}.data(:,idx);
                        
                    case 'fx'
                        DataBank{idy}.Have.XForces = 1;
                        FlagData.Enable.XForces = 'On';
                        DataBank{idy}.Var.Forces(:,5) = DataBank{idy}.data(:,idx);
                    case 'fy'
                        DataBank{idy}.Var.Forces(:,6) = DataBank{idy}.data(:,idx);
                    case 'fz'
                        DataBank{idy}.Var.Forces(:,7) = DataBank{idy}.data(:,idx);
                        
                    case 'U1'
                        DataBank{idy}.Have.U = 1;
                        FlagData.Enable.U = 'On';
                        DataBank{idy}.Var.Forces(:,8) = DataBank{idy}.data(:,idx);
                    case 'U2'
                        DataBank{idy}.Var.Forces(:,9) = DataBank{idy}.data(:,idx);
                    case 'U3'
                        DataBank{idy}.Var.Forces(:,10) = DataBank{idy}.data(:,idx);
                    case 'U4'
                        DataBank{idy}.Var.Forces(:,11) = DataBank{idy}.data(:,idx);
                        
                    case 'Ud1'
                        DataBank{idy}.Have.Ud = 1;
                        FlagData.Enable.Ud = 'On';
                        DataBank{idy}.Var.Forces(:,12) = DataBank{idy}.data(:,idx);
                    case 'Ud2'
                        DataBank{idy}.Var.Forces(:,13) = DataBank{idy}.data(:,idx);
                    case 'Ud3'
                        DataBank{idy}.Var.Forces(:,14) = DataBank{idy}.data(:,idx);
                    case 'Ud4'
                        DataBank{idy}.Var.Forces(:,15) = DataBank{idy}.data(:,idx);        
                end
            end
        end
    end
%% -------------------------------------------------------------------------------------------------------------
%  -------------------------------------------------------------------------------------------------------------
%  ---------                                     Criação do GUI                                        ---------
%  -------------------------------------------------------------------------------------------------------------
%  -------------------------------------------------------------------------------------------------------------
    function Menu = creategui(~,~)
        % Plotando
        ParteGrafica.TamanhoTela = get(0,'ScreenSize');
        Menu.Figure = figure(1);
        set(Menu.Figure,'DockControls','off','Menubar','none','Toolbar','none','Renderer','OpenGL','resize','off','Position',[200 50 350 600])
        Pos.X = 25;
        Pos.Y = 10;
        
        % Menu de idiomas
        Menu.CKB_LNG = uibuttongroup('visible','on','Position',[0 .95 1 .05],'SelectionChangeFcn',@Language_Select);
        %         Menu.CKB_LNGV(1) = uicontrol('Style','radio','String','Português','Position',[Pos.X+60  5 80 20],'parent',Menu.CKB_LNG,'HandleVisibility','off');
        %         Menu.CKB_LNGV(2) = uicontrol('Style','radio','String','English','Position',  [Pos.X+170 5 80 20],'parent',Menu.CKB_LNG,'HandleVisibility','off');
        % Menu inferior
        Pos.X = 25;
        Pos.Y = 10;
        Menu.CKB_BTT =    uibuttongroup('visible','on','Position',[0 0 1 .07]);
        Menu.Plotar =     uicontrol('Style','pushbutton','String','Plot','Position',[Pos.X-10  Pos.Y 90 25],'parent',Menu.CKB_BTT,'Callback',@GraphsPlot);
        Menu.Salvar =     uicontrol('Style','pushbutton','String','Save','Position',[Pos.X+220 Pos.Y 90 25],'parent',Menu.CKB_BTT,'Callback',@SaveGraphs);
        Menu.CKB_SVtext = uicontrol('Style','edit','String',FlagData.Indentifier,'Position',[Pos.X+100 Pos.Y 100 25],'parent',Menu.CKB_BTT,'HandleVisibility','off');
        
        % Caixa de menus de gráficos
        Menu.Graphs_pnn = uipanel('Title','Graphs','Position',[.03 .60 .47 .34]);
        Pos.Y = 165;
        Pos.X = 5;
        space = 75;
        Menu.Graphs{1} = uicontrol('Style','checkbox','String'  ,'2D Plot',  'parent',Menu.Graphs_pnn,'Position',[Pos.X Pos.Y 100 20],'Enable',FlagData.Enable.DDPlot);
        Menu.Graphs{2} = uicontrol('Style','checkbox','String'  ,'Coord',    'parent',Menu.Graphs_pnn,'Position',[Pos.X Pos.Y-20 100 20],'Enable',FlagData.Enable.Coord);
        Menu.Graphs{3} = uicontrol('Style','checkbox','String'  ,'Vel',      'parent',Menu.Graphs_pnn,'Position',[Pos.X Pos.Y-40 100 20],'Enable',FlagData.Enable.Vel);
        Menu.Graphs{4} = uicontrol('Style','checkbox','String'  ,'XErr',     'parent',Menu.Graphs_pnn,'Position',[Pos.X Pos.Y-60 100 20],'Enable',FlagData.Enable.XErr);
        Menu.Graphs{5} = uicontrol('Style','checkbox','String'  ,'Acc',      'parent',Menu.Graphs_pnn,'Position',[Pos.X Pos.Y-80 100 20],'Enable',FlagData.Enable.Acc);
        Menu.Graphs{6} = uicontrol('Style','checkbox','String'  ,'Pwm',      'parent',Menu.Graphs_pnn,'Position',[Pos.X Pos.Y-100 100 20],'Enable',FlagData.Enable.Pwm);
        Menu.Graphs{7} = uicontrol('Style','checkbox','String'  ,'',        'parent',Menu.Graphs_pnn,'Position',[Pos.X Pos.Y-120 100 20],'Enable','Off');
        Menu.Graphs{8}  = uicontrol('Style','checkbox','String' ,'3D Plot',  'parent',Menu.Graphs_pnn,'Position',[Pos.X+space Pos.Y 100 20],'Enable',FlagData.Enable.DDDPlot);
        Menu.Graphs{9}  = uicontrol('Style','checkbox','String' ,'Angles',   'parent',Menu.Graphs_pnn,'Position',[Pos.X+space Pos.Y-20 100 20],'Enable',FlagData.Enable.Angles);
        Menu.Graphs{10} = uicontrol('Style','checkbox','String' ,'AngVel',   'parent',Menu.Graphs_pnn,'Position',[Pos.X+space Pos.Y-40 100 20],'Enable',FlagData.Enable.AngVel);
        Menu.Graphs{11} = uicontrol('Style','checkbox','String' ,'AngErr',   'parent',Menu.Graphs_pnn,'Position',[Pos.X+space Pos.Y-60 100 20],'Enable',FlagData.Enable.AngErr);
        Menu.Graphs{12} = uicontrol('Style','checkbox','String' ,'Gyro',     'parent',Menu.Graphs_pnn,'Position',[Pos.X+space Pos.Y-80 100 20],'Enable',FlagData.Enable.Gyro);
        Menu.Graphs{13} = uicontrol('Style','checkbox','String' ,'Forces',   'parent',Menu.Graphs_pnn,'Position',[Pos.X+space Pos.Y-100 100 20],'Enable',FlagData.Enable.Forces);
        Menu.Graphs{14} = uicontrol('Style','checkbox','String' ,'XForces',  'parent',Menu.Graphs_pnn,'Position',[Pos.X+space Pos.Y-120 100 20],'Enable',FlagData.Enable.XForces);
        
        Menu.Graphs_Call = uicontrol('Style','pushbutton','String','Select All',     'parent',Menu.Graphs_pnn,'Position',[Pos.X+15 Pos.Y-150 100 20],'Callback',@Select_All);
        
        % Caixa de menu de características do banco de dados
        Pos.Y = Pos.Y-5;
        Menu.CKB_CD = uipanel('Title','DataBase Characteristics','Position',[.53 .60 .44 .34]);
        Menu.CKB_CD1 = uicontrol('Style','text','String','Begins [s]:'  ,'parent',Menu.CKB_CD,'HorizontalAlignment','left','Position',[Pos.X Pos.Y 60 20]);
        Menu.CKB_CD2 = uicontrol('Style','text','String','Ends [s]:'    ,'parent',Menu.CKB_CD,'HorizontalAlignment','left','Position',[Pos.X Pos.Y-20 60 20]);
        Menu.CKB_CD2 = uicontrol('Style','text','String','Begins [n]:'  ,'parent',Menu.CKB_CD,'HorizontalAlignment','left','Position',[Pos.X Pos.Y-40 60 20]);
        Menu.CKB_CD4 = uicontrol('Style','text','String','Ends [n]:'    ,'parent',Menu.CKB_CD,'HorizontalAlignment','left','Position',[Pos.X Pos.Y-60 60 20]);
        Menu.CKB_BeginsS = uicontrol('Style','edit','String',(round(DataBank{1}.Var.t(1,1))),'parent',Menu.CKB_CD,'HorizontalAlignment','left','Position',[75 Pos.Y+2 50 20],'Callback',@BeginsAts);
        Menu.CKB_EndsS   = uicontrol('Style','edit','String',(round(DataBank{1}.Var.t(end,1))),'parent',Menu.CKB_CD,'HorizontalAlignment','left','Position',[75 Pos.Y-18 50 20],'Callback',@EndsAts);
        Menu.CKB_Beginsn = uicontrol('Style','edit','String','1','parent',Menu.CKB_CD,'HorizontalAlignment','left','Position',[75 Pos.Y-40 50 20],'Callback',@BeginsAt);
        Menu.CKB_Endsn   = uicontrol('Style','edit','String','end','parent',Menu.CKB_CD,'HorizontalAlignment','left','Position',[75 Pos.Y-60 50 20],'Callback',@EndsAt);
        
        % Caixa de menu de opções
        Pos.Y = 51;
        Pos.X = 0;
        Menu.CKB_OPT =          uipanel('Title','Options','Position',[.03 .08 .60 .15]);
        Menu.CKB_Legendas =     uicontrol('Style','checkbox','String'  ,'Legend','parent',Menu.CKB_OPT,'Position',[10 Pos.Y 100 20]);
        Menu.CKB_Desired =      uicontrol('Style','checkbox','String'  ,'Desired','parent',Menu.CKB_OPT,'Position',[110 Pos.Y 80 20]);
        Menu.CKB_PlayVideo =    uicontrol('Style','pushbutton','String','Play Video','parent',Menu.CKB_OPT,'Position',[8 Pos.Y-44 80 20],'Callback',@PlayVideo,'Enable',FlagData.Enable.Video);
        Menu.CKB_ShowHeader =   uicontrol('Style','pushbutton','String','Show Header','parent',Menu.CKB_OPT,'Position',[107 Pos.Y-44 85 20],'Callback',@ShowHeader,'Enable','On');
        Menu.CKB_CloseFigures = uicontrol('Style','pushbutton','String','Close Figures','parent',Menu.CKB_OPT,'Position',[107 Pos.Y-21 85 20],'Callback',@CloseAll);
        Menu.CKB_Gains =        uicontrol('Style','pushbutton','String','Gains','parent',Menu.CKB_OPT,'Position',[8 Pos.Y-20 80 20],'Enable',FlagData.Enable.Gains,'Callback',@Show_Gains);
        
        % Caixa de menu de formatos
        Menu.CKB_SVtx = uipanel('Title','Format','Position',[.72 .08 .26 .15]);
        Menu.CKB_SVeps = uicontrol('Style','checkbox','String','.eps','parent',Menu.CKB_SVtx,'Position',[20 Pos.Y 50 20]);
        Menu.CKB_SVjpg = uicontrol('Style','checkbox','String','.jpg','parent',Menu.CKB_SVtx,'Position',[20 Pos.Y-20 50 20]);
        Menu.CKB_SVpng = uicontrol('Style','checkbox','String','.png','parent',Menu.CKB_SVtx,'Position',[20 Pos.Y-40 50 20]);
        
        % Cria a Tabela com as Infromações sobre cada arquivo
        for ki = 1:size(DataBank,2)
            RowNames{ki} = DataBank{ki}.Type;
            DataMatriz(ki,:) = {size(DataBank{ki}.data,1) size(DataBank{ki}.data,2) true DataBank{ki}.Type};
        end
        % Create the uitable
        Menu.Table = uitable('Data',DataMatriz,...
            'ColumnName',{'Rows','Columns','Display','Legend'},...
            'ColumnFormat', {'numeric','numeric','logical','char'},...
            'ColumnEditable', [false false true true true],...
            'Position',[10 150 330 200],...
            'ColumnWidth',{50,60,50,80},...
            'RowName',RowNames);
        set(Menu.CKB_Endsn,'String',DataMatriz{1,1});
        
        %%
    end
%% -------------------------------------------------------------------------------------------------------------
%  -------------------------------------------------------------------------------------------------------------
%  ---------                                   Funções de Ajuste Temporal                              ---------
%  -------------------------------------------------------------------------------------------------------------
%  -------------------------------------------------------------------------------------------------------------
    function BeginsAt(~,~) % Atualiza o valor do tempo utilizando as linhas de entrada
        %%
        if isnan(str2double(get(Menu.CKB_Endsn,'String')))
            errordlg('Input must be a number','Error');
        else
            set(Menu.CKB_BeginsS,'String',round(DataBank{1}.Var.t(str2double(get(Menu.CKB_Beginsn,'String'))),3));
        end
    end
    function EndsAt(~,~) % Atualiza o valor do tempo utilizando as linhas de entrada
        %%
        if isnan(str2double(get(Menu.CKB_Endsn,'String')))
            errordlg('Input must be a number','Error');
        else
            if str2double(get(Menu.CKB_Endsn,'String')) > size(DataBank{1}.Var.t,1)
                set(Menu.CKB_Endsn,'String',size(DataBank{1}.Var.t,1));
            end
            set(Menu.CKB_EndsS,'String',round(DataBank{1}.Var.t(str2double(get(Menu.CKB_Endsn,'String'))),3));
        end
    end
    function EndsAts(~,~) % Atualiza o valor das linhas utilizando o tempo de entrada
        %%
        if isnan(str2double(get(Menu.CKB_EndsS,'String')))
            errordlg('Input must be a number','Error');
        else
            NewLines = find((DataBank{1}.Var.t(:,1)) > str2double(get(Menu.CKB_EndsS,'String')),1,'first');
            if isempty(NewLines)
                NewLines = size(DataBank{1}.Var.t,1);
                set(Menu.CKB_EndsS,'String',round(DataBank{1}.Var.t(end,1)));
            end
            set(Menu.CKB_Endsn,'String',NewLines);
        end
    end
    function BeginsAts(~,~) % Atualiza o valor das linhas utilizando o tempo de entrada
        %%
        if isnan(str2double(get(Menu.CKB_BeginsS,'String')))
            errordlg('Input must be a number','Error');
        else
            NewLines = find((DataBank{1}.Var.t(:,1)) > str2double(get(Menu.CKB_BeginsS,'String')),1,'first');
            if isempty(NewLines)
                NewLines = size(DataBank{1}.Var.t,1);
                set(Menu.CKB_BeginsS,'String',round(DataBank{1}.Var.t(end,1)));
            end
            set(Menu.CKB_Beginsn,'String',NewLines);
        end
    end
%% -------------------------------------------------------------------------------------------------------------
%  -------------------------------------------------------------------------------------------------------------
%  ---------                         Funcao para Plotar os graficos                                    ---------
%  -------------------------------------------------------------------------------------------------------------
%  -------------------------------------------------------------------------------------------------------------
    function GraphsPlot(~,~)
        %% Organiza as legendas
        Menu.Table_Data = get(Menu.Table,'Data');
        Menu.fim = str2double(get(Menu.CKB_Endsn,'String'));
        Menu.ini = str2double(get(Menu.CKB_Beginsn,'String'));
        Menu.inis = str2double(get(Menu.CKB_BeginsS,'String'));
        %% Callback Functions
        Menu = mPlot2D(Menu,DataBank);
        Menu = mPlotX(Menu,DataBank);
        Menu = mPlotXvel(Menu,DataBank);
        Menu = PlotXErr(Menu,DataBank);
        Menu = mPlotAcc(Menu,DataBank);
        Menu = mPlotPwm(Menu,DataBank);
        
        Menu = mPlot3D(Menu,DataBank);
        Menu = mPlotAngles(Menu,DataBank);
        Menu = mPlotAngVel(Menu,DataBank);
        Menu = mPlotAngErr(Menu,DataBank);
        Menu = mPlotGyro(Menu,DataBank);
        Menu = mPlotForces(Menu,DataBank);
        Menu = mPlotXForces(Menu,DataBank);
        %% Menu.CKB_Cres
    end
%%  ------------------------------------------------------------------------------------------------------------
%  -------------------------------------------------------------------------------------------------------------
%  ---------                                  Salvar Gráficos                                          ---------
%  -------------------------------------------------------------------------------------------------------------
%  -------------------------------------------------------------------------------------------------------------
    function SaveGraphs(~,~)
        %%
        try
            identf = get(Menu.CKB_SVtext,'String');
            strin{2} = ['Graph_', identf, '_Coord'];
            strin{3} = ['Graph_', identf, '_Vel'];
            strin{4} = ['Graph_', identf, '_CordError'];
            strin{5} = ['Graph_', identf, '_Acc'];
            strin{6} = ['Graph_', identf, '_Pwm'];
            strin{7} = ['Graph_', identf, '_'];
            strin{8} = ['Graph_', identf, '_3D'];
            strin{9} = ['Graph_', identf, '_Angles'];
            strin{10} = ['Graph_', identf, '_AngVel'];
            strin{11} = ['Graph_', identf, '_AngErr'];
            strin{12} = ['Graph_', identf, '_Gyro'];
            strin{13} = ['Graph_', identf, '_Forces'];
            strin{14} = ['Graph_', identf, '_XForces'];
            
            strin{20} = ['Graph_', identf, '_2Dxz'];
            strin{21} = ['Graph_', identf, '_2Dyz'];
            strin{22} = ['Graph_', identf, '_2Dxy'];
            
            
            
            % Saves all plots if exists
            for idx = 2:14
                try
                    if get(Menu.Graphs{idx},'Value')
                        if get(Menu.CKB_SVeps,'Value')
                            saveas(Menu.PlotFig(idx),strin{idx},'epsc')
                            disp(['File ',strin{idx},'.eps Saved']);
                        end
                        if get(Menu.CKB_SVjpg,'Value')
                            saveas(Menu.PlotFig(idx),strin{idx},'jpg')
                            disp(['File ',strin{idx},'.jpg Saved']);
                        end
                        if get(Menu.CKB_SVpng,'Value')
                            saveas(Menu.PlotFig(idx),strin{idx},'png')
                            disp(['File ',strin{idx},'.png Saved']);
                        end
                        
                    end
                end
            end
            
            % Saves the 2D plots if exists
            for idx = 20:22
                try
                    if get(Menu.Graphs{1},'Value')
                        
                        if get(Menu.CKB_SVeps,'Value')
                            saveas(Menu.PlotFig(idx),strin{idx},'epsc')
                            disp(['File ',strin{idx},'.jpg Saved']);
                        end
                        if get(Menu.CKB_SVjpg,'Value')
                            saveas(Menu.PlotFig(idx),strin{idx},'jpg')
                            disp(['File ',strin{idx},'.jpg Saved']);
                        end
                        if get(Menu.CKB_SVpng,'Value')
                            saveas(Menu.PlotFig(idx),strin{idx},'png')
                            disp(['File ',strin{idx},'.png Saved']);
                        end
                    end
                end
            end
            
            disp('Files Saved.')
        catch
            display('Some Error Occur!')
        end
    end
%%  ------------------------------------------------------------------------------------------------------------
%  -------------------------------------------------------------------------------------------------------------
%  ---------                                  Calbacks Gerais                                          ---------
%  -------------------------------------------------------------------------------------------------------------
%  -------------------------------------------------------------------------------------------------------------
    function Show_Gains(~,~)
        %%
        display(StatusData.Gains.Data);
    end
    function PlayVideo(~,~)
        %%
        system(StatusData.Video.Name);
    end
    function CloseAll(~,~)
        %%
        %         h =  findobj('type','figure');
        %         n = max(h);
        try
            for k = 2:25
                try
                    close(figure(k))
                end
            end
        end
    end
    function ShowHeader(~,~)
        %%
        for idx = 1:size(DataBank,2)
            fprintf([DataBank{idx}.Type,': ',DataBank{idx}.textdata{1}, '\n'])
        end
    end
    function Select_All(~,~)
        %%
        if(get(Menu.Graphs_Call,'Value') == 1)
            if (strcmp(get(Menu.Graphs_Call,'String'),'Select All'))
                set(Menu.Graphs_Call,'String','None')
                Valor = true;
            elseif  (strcmp(get(Menu.Graphs_Call,'String'),'None'))
                set(Menu.Graphs_Call,'String','Select All')
                Valor = false;
            end
            for idx = 1:14
                if strcmp(get(Menu.Graphs{idx},'Enable'),'on')
                    set(Menu.Graphs{idx},'Value',Valor)
                end
            end
        end
        
        
    end
%% -------------------------------------------------------------------------------------------------------------
%  -------------------------------------------------------------------------------------------------------------
%  ---------                               Funções de Plotagem                                         ---------
%  -------------------------------------------------------------------------------------------------------------
%  -------------------------------------------------------------------------------------------------------------
    function Menu = mPlot2D(Menu,DataBank)
        %%
        if get(Menu.Graphs{1},'Value')
            Table_Data = Menu.Table_Data;
            ini = Menu.ini;
            fim = Menu.fim;
            nLabel = 1;
            Menu.PlotFig(20) = figure(20);
            %          figure(Menu.PlotFigD(1));
            set(Menu.PlotFig(20),'units','pix','pos',[5 500 1200 500],'PaperPositionMode','auto')
            for idb = 1:size(DataBank,2)
                DataBank{idb}.Show = Table_Data{1,3};
                if (Table_Data{idb,3})
                    Label(nLabel) = Menu.Table.Data(idb,4); nLabel = nLabel+1;
                    if (get(Menu.CKB_Desired,'Value')) && (DataBank{idb}.Have.Desird == 1)
                        Label(nLabel) = Menu.Table.Data(idb,4); nLabel = nLabel+1;
                    end
                    hold on
                    plot(DataBank{idb}.Var.X(ini:fim,1),DataBank{idb}.Var.X(ini:fim,3),'Color',DataBank{idb}.Color,'LineStyle','-');
                    if (get(Menu.CKB_Desired,'Value')) && (DataBank{idb}.Have.Desird == 1)
                        plot(DataBank{idb}.Var.X(ini:fim,4),DataBank{idb}.Var.X(ini:fim,6),'Color',DataBank{idb}.Color,'LineStyle','--');
                    end
                    hold off
                end
            end
            grid on
            xlabel('x [m]');ylabel('z [m]');
            
            if get(Menu.CKB_Legendas,'Value')
                legend(Label,'Location','southeast')
            end
            
            Menu.PlotFig(21) = figure(21);
            set(Menu.PlotFig(21),'units','pix','pos',[5 500 1200 500],'PaperPositionMode','auto')
            for idb = 1:size(DataBank,2)
                DataBank{idb}.Show = Table_Data{1,3};
                if (Table_Data{idb,3})
                    hold on
                    plot(DataBank{idb}.Var.X(ini:fim,2),DataBank{idb}.Var.X(ini:fim,3),'Color',DataBank{idb}.Color,'LineStyle','-');
                    if (get(Menu.CKB_Desired,'Value')) && (DataBank{idb}.Have.Desird == 1)
                        plot(DataBank{idb}.Var.X(ini:fim,5),DataBank{idb}.Var.X(ini:fim,6),'Color',DataBank{idb}.Color,'LineStyle','--');
                    end
                    hold off
                end
            end
            grid on;
            xlabel('y [m]');ylabel('z [m]');
            
            if get(Menu.CKB_Legendas,'Value')
                legend(Label,'Location','southeast')
            end
            
            Menu.PlotFig(22) = figure(22);
            set(Menu.PlotFig(22),'units','pix','pos',[5 500 1200 500],'PaperPositionMode','auto')
            for idb = 1:size(DataBank,2)
                DataBank{idb}.Show = Table_Data{1,3};
                if (Table_Data{idb,3})
                    hold on
                    plot(DataBank{idb}.Var.X(ini:fim,1),DataBank{idb}.Var.X(ini:fim,2),'Color',DataBank{idb}.Color,'LineStyle','-');
                    if (get(Menu.CKB_Desired,'Value')) && (DataBank{idb}.Have.Desird == 1)
                        plot(DataBank{idb}.Var.X(ini:fim,4),DataBank{idb}.Var.X(ini:fim,5),'Color',DataBank{idb}.Color,'LineStyle','--');
                    end
                    hold off
                end
            end
            grid on;
            xlabel('x [m]'); ylabel('y [m]');
            
            if get(Menu.CKB_Legendas,'Value')
                legend(Label,'Location','southeast')
            end
        end
    end
%%
    function Menu = mPlotX(Menu,DataBank)
        %%
        if get(Menu.Graphs{2},'Value')     % Coordinates
            Table_Data = Menu.Table_Data;
            ini = Menu.ini;
            fim = Menu.fim;
            nLabel = 1;
            Menu.PlotFig(2) = figure(2);
            set(Menu.PlotFig(2),'units','pix','pos',[5 500 1200 500],'PaperPositionMode','auto')
            for idb = 1:size(DataBank,2)
                DataBank{idb}.Show = Table_Data{1,3};
                if (Table_Data{idb,3})
                    Label(nLabel) = Menu.Table.Data(idb,4); nLabel = nLabel+1;
                    if (get(Menu.CKB_Desired,'Value')) && (DataBank{idb}.Have.Desird == 1)
                        Label(nLabel) = Menu.Table.Data(idb,4); nLabel = nLabel+1;
                    end
                    for idxplot = 1:3
                        subplot(3,1,idxplot)
                        hold on
                        plot(DataBank{idb}.Var.t(ini:fim,end)-Menu.inis,DataBank{idb}.Var.X(ini:fim,idxplot),'Color',DataBank{idb}.Color,'LineStyle','-');
                        if (get(Menu.CKB_Desired,'Value')) && (DataBank{idb}.Have.Desird == 1)
                            plot(DataBank{idb}.Var.t(ini:fim,end)-Menu.inis,DataBank{idb}.Var.X(ini:fim,idxplot+3),'Color',DataBank{idb}.Color,'LineStyle','--');
                        end
                        hold off
                    end
                end
            end
            subplot(3,1,1)
            grid on
            ylabel('x [m]')
            subplot(3,1,2)
            ylabel('y [m]')
            grid on
            subplot(3,1,3)
            grid on
            xlabel('time [s]')
            ylabel('z [m]')
            
            if get(Menu.CKB_Legendas,'Value')
                legend(Label,'Location','southeast')
            end
        end
    end
%%
    function Menu = mPlotXvel(Menu,DataBank)
        %%
        if get(Menu.Graphs{3},'Value')     % Coordinate Velocities
            Table_Data = Menu.Table_Data;
            ini = Menu.ini;
            fim = Menu.fim;
            nLabel = 1;
            Menu.PlotFig(3) = figure(4);
            set(Menu.PlotFig(3),'units','pix','pos',[5 500 1200 500],'PaperPositionMode','auto')
            for idb = 1:size(DataBank,2)
                DataBank{idb}.Show = Table_Data{1,3};
                if (Table_Data{idb,3}) && (DataBank{idb}.Have.XVel == 1)
                    Label(nLabel) = Menu.Table.Data(idb,4); nLabel = nLabel+1;
                    if (get(Menu.CKB_Desired,'Value')) && (DataBank{idb}.Have.Desird == 1)
                        Label(nLabel) = Menu.Table.Data(idb,4); nLabel = nLabel+1;
                    end
                    for idxplot = 1:3
                        hold on
                        subplot(3,1,idxplot)
                        plot(DataBank{idb}.Var.t(ini:fim,end)-Menu.inis,DataBank{idb}.Var.X(ini:fim,idxplot+9),'Color',DataBank{idb}.Color,'LineStyle','-');
                    end
                end
            end
            subplot(3,1,1)
            grid on
            ylabel('X [m/s]')
            subplot(3,1,2)
            ylabel('Y [m/s]')
            grid on
            subplot(3,1,3)
            grid on
            ylabel('Z [m/s]')
            xlabel('time [s]')
            if get(Menu.CKB_Legendas,'Value')
                legend(Label,'Location','southeast')
            end
        end
    end
%%
    function Menu = PlotXErr(Menu,DataBank)
        %%
        if get(Menu.Graphs{4},'Value')         % Coordinate Errors
            Table_Data = Menu.Table_Data;
            ini = Menu.ini;
            fim = Menu.fim;
            nLabel = 1;
            Menu.PlotFig(4) = figure(5);
            set(Menu.PlotFig(4),'units','pix','pos',[5 500 1200 500],'PaperPositionMode','auto')
            for idb = 1:size(DataBank,2)
                DataBank{idb}.Show = Table_Data{1,3};
                if (Table_Data{idb,3}) && (DataBank{idb}.Have.Xtil == 1)
                    Label(nLabel) = Menu.Table.Data(idb,4); nLabel = nLabel+1;
                    for idxplot = 1:3
                        hold on
                        subplot(3,1,idxplot)
                        plot(DataBank{idb}.Var.t(ini:fim,end)-Menu.inis,DataBank{idb}.Var.X(ini:fim,idxplot+6),'Color',DataBank{idb}.Color,'LineStyle','-');
                    end
                end
            end
            subplot(3,1,1)
            ylabel('$\tilde{X}$ [m]','Interpreter','latex')
            subplot(3,1,2)
            ylabel('$\tilde{Y}$ [m]','Interpreter','latex')
            subplot(3,1,3)
            ylabel('$\tilde{Z}$ [s]','Interpreter','latex')
            xlabel('t [m]')
            if get(Menu.CKB_Legendas,'Value')
                legend(Label,'Location','southeast')
            end
        end
    end
%%
    function Menu = mPlotAcc(Menu,DataBank)
        %%
        if get(Menu.Graphs{5},'Value')     % Coordinates
            Table_Data = Menu.Table_Data;
            ini = Menu.ini;
            fim = Menu.fim;
            nLabel = 1;
            Menu.PlotFig(5) = figure(6);
            set(Menu.PlotFig(5),'units','pix','pos',[5 500 1200 500],'PaperPositionMode','auto')
            for idb = 1:size(DataBank,2)
                DataBank{idb}.Show = Table_Data{1,3};
                if (Table_Data{idb,3}) && (DataBank{idb}.Have.Acc == 1)
                    Label(nLabel) = Menu.Table.Data(idb,4); nLabel = nLabel+1;
                    for idxplot = 1:3
                        hold on
                        subplot(3,1,idxplot)
                        plot(DataBank{idb}.Var.t(ini:fim,end)-Menu.inis,DataBank{idb}.Var.Sensor(ini:fim,idxplot),'Color',DataBank{idb}.Color,'LineStyle','-');
                    end
                end
            end
            subplot(3,1,1)
            grid on
            ylabel('Acc_x [m]')
            subplot(3,1,2)
            ylabel('Acc_y [m]')
            grid on
            subplot(3,1,3)
            grid on
            ylabel('Acc_z [m]')
            xlabel('time [s]')
            if get(Menu.CKB_Legendas,'Value')
                legend(Label,'Location','southeast')
            end
        end
    end
%%
    function Menu = mPlotPwm(Menu,DataBank)
        %%
        if get(Menu.Graphs{6},'Value')     % Pwm Values
            Table_Data = Menu.Table_Data;
            ini = Menu.ini;
            fim = Menu.fim;
            nLabel = 1;
            Menu.PlotFig(6) = figure(7);
            set(Menu.PlotFig(6),'units','pix','pos',[5 500 1200 500],'PaperPositionMode','auto')
            for idb = 1:size(DataBank,2)
                DataBank{idb}.Show = Table_Data{1,3};
                if (Table_Data{idb,3}) && (DataBank{idb}.Have.Pwm == 1)
                    Label(nLabel) = Menu.Table.Data(idb,4); nLabel = nLabel+1;
                    for idxplot = 1:4
                        hold on
                        subplot(4,1,idxplot)
                        plot(DataBank{idb}.Var.t(ini:fim,end)-Menu.inis,DataBank{idb}.Var.Sensor(ini:fim,idxplot+6),'Color',DataBank{idb}.Color,'LineStyle','-');
                    end
                end
            end
            subplot(4,1,1); grid on
            ylabel('Pwm_1');
            subplot(4,1,2); grid on
            ylabel('Pwm_2');
            subplot(4,1,3); grid on
            ylabel('Pwm_3');
            subplot(4,1,4); grid on
            ylabel('Pwm_4');
            xlabel('Time [s]');
            if get(Menu.CKB_Legendas,'Value')
                legend(Label,'Location','southeast')
            end
        end
    end
%%
    function Menu = mPlot3D(Menu,DataBank)
        %%
        if get(Menu.Graphs{8},'Value')     % 3D
            Table_Data = Menu.Table_Data;
            ini = Menu.ini;
            fim = Menu.fim;
            nLabel = 1;
            Menu.PlotFig(8) = figure(9);
            set(Menu.PlotFig(8),'units','pix','pos',[5 500 1200 500],'PaperPositionMode','auto')
            for idb = 1:size(DataBank,2)
                DataBank{idb}.Show = Table_Data{1,3};
                if (Table_Data{idb,3})
                    Label(nLabel) = Menu.Table.Data(idb,4); nLabel = nLabel+1;
                    if (get(Menu.CKB_Desired,'Value')) && (DataBank{idb}.Have.Desird == 1)
                        Label(nLabel) = Menu.Table.Data(idb,4); nLabel = nLabel+1;
                    end
                    hold on
                    plot3(DataBank{idb}.Var.X(ini:fim,1),DataBank{idb}.Var.X(ini:fim,2),DataBank{idb}.Var.X(ini:fim,3),'Color',DataBank{idb}.Color,'LineStyle','-');
                    if (get(Menu.CKB_Desired,'Value')) && (DataBank{idb}.Have.Desird == 1)
                        plot3(DataBank{idb}.Var.X(ini:fim,4),DataBank{idb}.Var.X(ini:fim,5),DataBank{idb}.Var.X(ini:fim,6),'Color',DataBank{idb}.Color,'LineStyle','--');
                    end
                    hold off
                end
            end
            grid on
            xlabel('x [m]')
            ylabel('y [m]')
            zlabel('z [m]')
            if get(Menu.CKB_Legendas,'Value')
                legend(Label,'Location','southeast')
            end
        end
    end
%%
    function Menu = mPlotAngles(Menu,DataBank)
        %%
        if get(Menu.Graphs{9},'Value')     % Angles
            Table_Data = Menu.Table_Data;
            ini = Menu.ini;
            fim = Menu.fim;
            nLabel = 1;
            Menu.PlotFig(9) = figure(10);
            set(Menu.PlotFig(9),'units','pix','pos',[5 500 1200 500],'PaperPositionMode','auto')
            for idb = 1:size(DataBank,2)
                DataBank{idb}.Show = Table_Data{1,3};
                if (Table_Data{idb,3}) && (DataBank{idb}.Have.Angles == 1)
                    Label(nLabel) = Menu.Table.Data(idb,4); nLabel = nLabel+1;
                    if (get(Menu.CKB_Desired,'Value')) && (DataBank{idb}.Have.AngDesird == 1)
                        Label(nLabel) = Menu.Table.Data(idb,4); nLabel = nLabel+1;
                    end
                    for idxplot = 1:3
                        subplot(3,1,idxplot)
                        hold on
                        plot(DataBank{idb}.Var.t(ini:fim,end)-Menu.inis,DataBank{idb}.Var.Ang(ini:fim,idxplot),'Color',DataBank{idb}.Color,'LineStyle','-');
                        if (get(Menu.CKB_Desired,'Value')) && (DataBank{idb}.Have.AngDesird == 1)
                            plot(DataBank{idb}.Var.t(ini:fim,end)-Menu.inis,DataBank{idb}.Var.Ang(ini:fim,idxplot+3),'Color',DataBank{idb}.Color,'LineStyle','--');
                        end
                        hold off
                    end
                end
            end
            subplot(3,1,1)
            ylabel('\theta [^o]')
            grid on
            subplot(3,1,2)
            ylabel('\phi [^o]')
            grid on
            subplot(3,1,3)
            ylabel('\psi [^o]')
            xlabel('t [t]')
            grid on
            if get(Menu.CKB_Legendas,'Value')
                legend(Label,'Location','southeast')
            end
            
        end
    end
%%
    function Menu = mPlotAngVel(Menu,DataBank)
        %%
        if get(Menu.Graphs{10},'Value')     % Angular Velocities
            Table_Data = Menu.Table_Data;
            ini = Menu.ini;
            fim = Menu.fim;
            nLabel = 1;
            Menu.PlotFig(10) = figure(11);
            set(Menu.PlotFig(10),'units','pix','pos',[5 500 1200 500],'PaperPositionMode','auto')
            for idb = 1:size(DataBank,2)
                DataBank{idb}.Show = Table_Data{1,3};
                if (Table_Data{idb,3}) && (DataBank{idb}.Have.AngVel == 1)
                    Label(nLabel) = Menu.Table.Data(idb,4); nLabel = nLabel+1;
                    for idxplot = 1:3
                        hold on
                        subplot(3,1,idxplot)
                        plot(DataBank{idb}.Var.t(ini:fim,end)-Menu.inis,DataBank{idb}.Var.Ang(ini:fim,idxplot+9),'Color',DataBank{idb}.Color,'LineStyle','-');
                    end
                end
            end
            subplot(3,1,1)
            grid on
            ylabel('X [^o/s]')
            subplot(3,1,2)
            ylabel('Y [^o/s]')
            grid on
            subplot(3,1,3)
            grid on
            ylabel('Z [^o/s]')
            xlabel('time [s]')
            if get(Menu.CKB_Legendas,'Value')
                legend(Label,'Location','southeast')
            end
        end
    end
%%
    function Menu = mPlotAngErr(Menu,DataBank)
        %%
        if get(Menu.Graphs{11},'Value')     % Angular Errors
            Table_Data = Menu.Table_Data;
            ini = Menu.ini;
            fim = Menu.fim;
            nLabel = 1;
            Menu.PlotFig(11) = figure(12);
            set(Menu.PlotFig(11),'units','pix','pos',[5 500 1200 500],'PaperPositionMode','auto')
            for idb = 1:size(DataBank,2)
                DataBank{idb}.Show = Table_Data{1,3};
                if (Table_Data{idb,3}) && (DataBank{idb}.Have.AngErr == 1)
                    Label(nLabel) = Menu.Table.Data(idb,4); nLabel = nLabel+1;
                    for idxplot = 1:3
                        hold on
                        subplot(3,1,idxplot)
                        plot(DataBank{idb}.Var.t(ini:fim,end)-Menu.inis,DataBank{idb}.Var.Angles(ini:fim,idxplot+9),'Color',DataBank{idb}.Color,'LineStyle','-');
                    end
                end
            end
            subplot(3,1,1); grid on
            ylabel('\phi [^o]')
            subplot(3,1,2); grid on
            ylabel('\theta [^o]')
            subplot(3,1,3); grid on
            ylabel('\psi [^o]')
            xlabel('time [s]')
            if get(Menu.CKB_Legendas,'Value')
                legend(Label,'Location','southeast')
            end
        end
    end
%%
    function Menu = mPlotGyro(Menu,DataBank)
        %%
        if get(Menu.Graphs{12},'Value')     % Coordinates
            Table_Data = Menu.Table_Data;
            ini = Menu.ini;
            fim = Menu.fim;
            nLabel = 1;
            Menu.PlotFig(12) = figure(13);
            set(Menu.PlotFig(12),'units','pix','pos',[5 500 1200 500],'PaperPositionMode','auto')
            for idb = 1:size(DataBank,2)
                DataBank{idb}.Show = Table_Data{1,3};
                if (Table_Data{idb,3}) && (DataBank{idb}.Have.Gyro == 1)
                    Label(nLabel) = Menu.Table.Data(idb,4); nLabel = nLabel+1;
                    for idxplot = 1:3
                        hold on
                        subplot(3,1,idxplot)
                        plot(DataBank{idb}.Var.t(ini:fim,end)-Menu.inis,DataBank{idb}.Var.Sensor(ini:fim,idxplot+3),'Color',DataBank{idb}.Color,'LineStyle','-');
                    end
                end
            end
            subplot(3,1,1)
            grid on
            ylabel('Gyro_x []')
            subplot(3,1,2)
            ylabel('Gyro_y []')
            grid on
            subplot(3,1,3)
            grid on
            ylabel('Gyro_z []')
            xlabel('time [s]')
            if get(Menu.CKB_Legendas,'Value')
                legend(Label,'Location','southeast')
            end
        end
    end
%%
    function Menu = mPlotForces(Menu,DataBank)
        %%
        if get(Menu.Graphs{13},'Value')         % Propulsor forces
            Table_Data = Menu.Table_Data;
            ini = Menu.ini;
            fim = Menu.fim;
            nLabel = 1;
            Menu.PlotFig(13) = figure(14);
            set(Menu.PlotFig(13),'units','pix','pos',[5 500 1200 500],'PaperPositionMode','auto')
            for idb = 1:size(DataBank,2)
                DataBank{idb}.Show = Table_Data{1,3};
                if (Table_Data{idb,3}) && (DataBank{idb}.Have.PForces == 1)
                    Label(nLabel) = Menu.Table.Data(idb,4); nLabel = nLabel+1;
                    for idxplot = 1:4
                        hold on
                        subplot(4,1,idxplot)
                        plot(DataBank{idb}.Var.t(ini:fim,end)-Menu.inis,DataBank{idb}.Var.Forces(ini:fim,idxplot),'Color',DataBank{idb}.Color,'LineStyle','-');
                        ylabel(['f_',num2str(idxplot),' [N]'])
                        hold off
                    end
                end
            end
            xlabel('t [m]')
            if get(Menu.CKB_Legendas,'Value')
                legend(Label,'Location','southeast')
            end
        end
    end
%%
    function Menu = mPlotXForces(Menu,DataBank)
        %%
        if get(Menu.Graphs{14},'Value')         % Coordinate forces
            Table_Data = Menu.Table_Data;
            ini = Menu.ini;
            fim = Menu.fim;
            nLabel = 1;
            Menu.PlotFig(14) = figure(15);
            set(Menu.PlotFig(14),'units','pix','pos',[5 500 1200 500],'PaperPositionMode','auto')
            for idb = 1:size(DataBank,2)
                DataBank{idb}.Show = Table_Data{1,3};
                if (Table_Data{idb,3}) && (DataBank{idb}.Have.XForces == 1)
                    Label(nLabel) = Menu.Table.Data(idb,4); nLabel = nLabel+1;
                    for idxplot = 1:3
                        hold on
                        subplot(3,1,idxplot)
                        plot(DataBank{idb}.Var.t(ini:fim,end)-Menu.inis,DataBank{idb}.Var.Forces(ini:fim,+4),'Color',DataBank{idb}.Color,'LineStyle','-');
                    end
                end
            end
            subplot(3,1,1)
            ylabel('f_x [N]')
            subplot(3,1,2)
            ylabel('f_y [N]')
            subplot(3,1,3)
            ylabel('f_z [N]')
            xlabel('t [t]')
            if get(Menu.CKB_Legendas,'Value')
                legend(Label,'Location','southeast')
            end
        end
    end
%%
end