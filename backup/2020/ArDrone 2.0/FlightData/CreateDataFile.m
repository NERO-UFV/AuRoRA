function journal = CreateDataFile(varargin)

itm = 0; % Initialize the item variable.
CommentFlag = 0; % Initialize Comment Flag variable.
TimeTag = datestr(clock,30); % get the actual time in the format yyyymmddThhddss

Mainfolder = pwd;
% cd('..')
cd('DataFiles\')

IndexChar = 0; % Control string flag
DesiredFolderflag = 0;  % Folder input flag

for idx = 1:nargin % Separetes all input data
    if isobject(varargin{idx}) % Detects what is object. i.e., robots
        journal{idx}.Type = class(varargin{idx});
        switch journal{idx}.Type
            case 'ArDrone'
                journal{idx}.Typen = 1;
            case 'Pionner'
                journal{idx}.Typen = 2;
            case 'Load'
                journal{idx}.Typen = 3;
            otherwise
                disp('Unknow vehicle Type.')
                break
        end
        journal{idx}.Index = idx;
        journal{idx}.DataTag = TimeTag;
        
    elseif ischar(varargin{idx}) % Detects what is string
        if idx <= nargin && IndexChar == 0  % First String is the Comment
        Comment = ['_' varargin{idx}];
        CommentFlag = 1;
        IndexChar = 1;
        elseif IndexChar == 1 % Second String is the Desired Folder to save the data
        DesiredFolder = [varargin{idx}];
        DesiredFolderflag = 1;
        IndexChar = 2;
        else % If the is only one string in the input, it is the comment
        Comment = ['_' varargin{idx}];
        CommentFlag = 1;
        end
    else
        disp('Invalid Variable Format.') % Otherwise
        break
    end
end

% Create a blank comment if none is presented
if CommentFlag == 0
    Comment = '';
end

if DesiredFolderflag == 1
    try
        disp(DesiredFolder);
    cd(DesiredFolder);
    catch 
        warning('Nonexistent desired folder. It was used the default one.')
    end
end
% Create the folder Log with the comment
Folder = ['Log','_',TimeTag,Comment,'.txt'];
mkdir(Folder);
cd(Folder);

% Create the files
if size(journal,2) == 1
    journal{1}.DataFile = fopen(['VarData_',journal{1}.Type,'_',TimeTag,'.txt'],'w');
else
    for idx = 1:size(journal,2)
        journal{idx}.DataFile = fopen(['VarData_',journal{idx}.Type,'-',num2str(journal{idx}.Index),'_',TimeTag,'.txt'],'w');
    end
end
cd(Mainfolder)


%         journal{idx}.TypeS = 1; % Sensorial Data Enable.
%         for idx = 1:size(journal,2)
%             switch  journal{idx}.Typen
%                 case 1
%                     fprintf(journal{idx}.DataFile,['t \t X \t Y \t Z \t Phi \t Theta \t Psi \t dx \t dy \t dz \t dPhi \t dTheta \t dPsi \t', ...
%                         'Xd \t Yd \t Zd \t Phid \t Thetad \t Psid \t', ...
%                         'U1 \t U2 \t U3 \t U4 \t', ...
%                         'Ud1 \t Ud2 \t Ud3 \t Ud4 \t', ...
%                         'Batt \t Pitch \t Roll \t Yaw \t Alt \t Vx \t Vy \t Vz \t ',...
%                         'Accx \t Accy \t Accz \t Gyrox \t Gyroy \t Gyroz \t ',...
%                         'pwm1 \t pwm2 \t pwm3 \t pwm4']);
%                     fprintf(journal{idx}.DataFile,'\n');
%                 case 2
%                     journal{idx}.Typen = 2;
%                 case 3
%                     journal{idx}.Typen = 3;
%             end
%         end

        
        for idx = 1:size(journal,2)
            journal{idx}.TypeS = 0; %Nao utiliza dados sensoriais.
            switch  journal{idx}.Typen
                case 1
                    fprintf(journal{idx}.DataFile,['X \t Y \t Z \t Phi \t Theta \t Psi \t Vx \t Vy \t Vz \t dPhi \t dTheta \t dPsi \t', ...
                        'Xd \t Yd \t Zd \t Phid \t Thetad \t Psid \t', ...
                        'U1 \t U2 \t U3 \t U4 \t', ...
                        'Ud1 \t Ud2 \t Ud3 \t Ud4 \t t']);
                    fprintf(journal{idx}.DataFile,'\n');
                case 2
                    fprintf(journal{idx}.DataFile,['X \t Y \t Z \t Phi \t Theta \t Psi \t Vx \t Vy \t Vz \t dPhi \t dTheta \t dPsi \t', ...
                        'Xd \t Yd \t Zd \t Phid \t Thetad \t Psid \t', ...
                        'U1 \t U2 \t U3 \t U4 \t', ...
                        'Ud1 \t Ud2 \t Ud3 \t Ud4 \t t']);
                    fprintf(journal{idx}.DataFile,'\n');
                    journal{idx}.Typen = 2;
                case 3
                    journal{idx}.Typen = 3;
            end
        end












end % function journal = CreateDataFile(varargin) 
