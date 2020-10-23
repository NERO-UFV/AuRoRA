function journal = CreateOptDataFile(varargin)

itm = 0; % Initialize the item variable.
CommentFlag = 0; % Initialize Comment Flag variable.
TimeTag = datestr(clock,30); % get the actual time in the format yyyymmddThhddss

Mainfolder = pwd;
% cd('..')
cd('DataFiles\');

IndexChar = 0; % Control string flag
DesiredFolderflag = 0;  % Folder input flag

for idx = 1:nargin % Separetes all input data
    if isobject(varargin{idx}) % Detects what is object. i.e., robots
        switch class(varargin{idx})
            case 'ArDrone'
                journal{idx}.Type = 'ArDroneOpt';
                journal{idx}.Typen = 1;
            case 'Pioneer3dx'
                journal{idx}.Type = 'Pioneer3dxOpt';
                journal{idx}.Typen = 2;
            case 'Load'
                journal{idx}.Type = 'LoadOpt';
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

% Create a blank comment.
if CommentFlag == 0
    Comment = '';
end

% Enter in the desired folder if was declared.
if DesiredFolderflag == 1
    try
        disp(DesiredFolder);
        cd(DesiredFolder);
    catch 
        warning('Nonexistent desired folder. It was used the default one.')
    end
end

% Create the folder Log with the comment
Folder = ['Log','_',TimeTag,Comment];
mkdir(Folder);
cd(Folder);

% Create the files
if size(journal,2) == 1
    journal{1}.DataFile = fopen(['VarData_',journal{1}.Type,'_','Optitrack','_',TimeTag,'.txt'],'w');
else
    for idx = 1:size(journal,2)
        journal{idx}.DataFile = fopen(['VarData_',journal{idx}.Type,'-',num2str(journal{idx}.Index),'_','Optitrack','_',TimeTag,'.txt'],'w');
    end
end
cd(Mainfolder)

for idx = 1:size(journal,2)
    journal{idx}.TypeS = 0; %Nao utiliza dados sensoriais.
    switch  journal{idx}.Typen
        case 1
            fprintf(journal{idx}.DataFile,['X \t Y \t Z \t Phi \t Theta \t Psi \t Vx \t Vy \t Vz \t dPhi \t dTheta \t dPsi \t', ...
                                           'Xd \t Yd \t Zd \t Phid \t Thetad \t Psid \t  t' ]);
            fprintf(journal{idx}.DataFile,'\n');
        case 2
            fprintf(journal{idx}.DataFile,['X \t Y \t Z \t Phi \t Theta \t Psi \t Vx \t Vy \t Vz \t dPhi \t dTheta \t dPsi \t', ...
                                           'Xd \t Yd \t Zd \t Phid \t Thetad \t Psid \t  t' ]);
            fprintf(journal{idx}.DataFile,'\n');
            journal{idx}.Typen = 2;
        case 3
            fprintf(journal{idx}.DataFile,['X \t Y \t Z \t Phi \t Theta \t Psi \t Vx \t Vy \t Vz \t dPhi \t dTheta \t dPsi \t', ...
                                           'Xd \t Yd \t Zd \t Phid \t Thetad \t Psid \t  t' ]);
            fprintf(journal{idx}.DataFile,'\n');
            journal{idx}.Typen = 3;
    end
end

end % function journal = CreateDataFile(varargin) 
