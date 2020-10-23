function [localIP,broadcastIP] = getIPv4(varargin)
% GETIPV4 uses Java functionality to find the local IP and broadcast IP for
% wired connections.
%
%   Note: This function is still a work in progress.
%
%   M. Kutzer, 25Apr2016, USNA

% Updates
%   08Nov2016 - Updated to account for multiple network connections

%% Get local host, networkInterface, and interface addresses using java
% Get the local network host
localhost = java.net.Inet4Address.getLocalHost();
% Get all IPv4 addresses by name
allIPs = java.net.Inet4Address.getAllByName(localhost.getCanonicalHostName());

n = numel(allIPs);
adapterName      = cell(n,1); % Network adapter name
interfaceAddress = cell(n,1); % Network interface address
localIP          = cell(n,1); % Local IP address
broadcastIP      = cell(n,1); % Broadcast IP address
nicInfo          = cell(n,1); % Combine NIC info (to eliminate redundancy)
for i = 1:n
    % Get network interface
    networkInterface(i) = java.net.NetworkInterface.getByInetAddress(allIPs(i));
    
    if ~isempty(networkInterface(i))
        % Get adapter display name
        adapterName{i} = char( networkInterface(i).getDisplayName() );
        % Define interface address
        interfaceAddress{i} = char( networkInterface(i).getInterfaceAddresses() );
        % Parse interface address
        out = sscanf(interfaceAddress{i},'[/%d.%d.%d.%d/%d [/%d.%d.%d.%d]',[1,9]);
        % Define Local IP
        localIP{i} = sprintf('%d.%d.%d.%d',out(1:4));
        % Define Broadcast IP
        broadcastIP{i} = sprintf('%d.%d.%d.%d',out(6:9));
        % Combine NIC info to find unique options
        nicInfo{i} = sprintf('%s,%s,%s',adapterName{i},localIP{i},broadcastIP{i});
    end
end

% Remove redundant adapters
[~,idx] = unique(nicInfo,'Stable');
adapterName = adapterName(idx);
localIP = localIP(idx);
broadcastIP = broadcastIP(idx);

n = numel(adapterName);
listStr = cell(n,1);
for i = 1:n
    % Append list for list dialog
    listStr{i} = sprintf('(%2.0d) %s --- Network IP: [%s] --- Broadcast IP: [%s]',...
        i,...
        adapterName{i},...
        localIP{i},...
        broadcastIP{i});
end

[s,ok] = listdlg('Name','Multiple NIC Found',...
    'PromptString','Select network adapter:',...
    'SelectionMode','single',...
    'ListString',listStr,...
    'ListSize',[900 300]);

% Parse output
if ok
    localIP = localIP{s};
    broadcastIP = broadcastIP{s};
else
    localIP = [];
    broadcastIP = [];
end