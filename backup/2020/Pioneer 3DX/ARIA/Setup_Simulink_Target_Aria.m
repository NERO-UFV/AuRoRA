% Add Pioneer files onto the MATLAB path
addpath(genpath(fileparts(mfilename('fullpath'))));
savepath;

% Refresh library Browser and Simulink Customizations. 
Library_Browser = LibraryBrowser.StandaloneBrowser;
Library_Browser.refreshLibraryBrowser;

% Copy DLL's for S-funcitons to MATLAB bin folder
dllfolder = fullfile(fileparts(mfilename('fullpath'))); %,'Library','Includes');
binfolder = fullfile(matlabroot,'bin','win32');
if(exist(fullfile(dllfolder,'AriaVC10.dll'),'file') == 0)
    copyfile(fullfile(dllfolder,'AriaVC10.dll'),binfolder);
end
if(exist(fullfile(dllfolder,'ariac_vc10_i386.dll'),'file') == 0)
    copyfile(fullfile(dllfolder,'ariac_vc10_i386.dll'),binfolder);
end