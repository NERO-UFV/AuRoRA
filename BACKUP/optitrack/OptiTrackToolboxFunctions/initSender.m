function udpS = initSender(port,IP)
% INITSENDER initializes a UDP server for transmitting information to a
% remote client.
%   udpS = initSender(port) creates a UDP Sender tied to the designated
%   port (suggested ports 31000 - 32000) using a user-selected broadcast 
%   IP.
%
%   udpS = initSender(port,IP) creates a UDP Sender tied to the 
%   designated port (suggested ports 31000 - 32000) using a specified IP.
%
%   Note: This function requires the DSP System Toolbox.
%
%   See also initReceiver
%
%   M. Kutzer, 31Oct2016, USNA

% Updates


%% Check inputs
% TODO - improve error handling
narginchk(1,2);

%% Set default IP
if nargin < 2
    % Set to broadcast
    % NOTE: This IP should be tied to your network IP
    [~,IP] = getIPv4;
end

%% Check inputs
% TODO - check port range
% TODO - check for valid IP

%% Create UDP Sender
udpS = dsp.UDPSender(...
    'RemoteIPAddress',IP,...
    'RemoteIPPort',port);