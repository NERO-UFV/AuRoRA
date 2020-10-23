% Make sure to have the server side running in V-REP: 
% in a child script of a V-REP scene, add following command
% to be executed just once, at simulation start:
%
% simRemoteApi.start(19999)
%
% then start simulation, and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

%% Conectando o Matlab ao V-Rep via servidor cliente
function vConnect(p3dx)
    disp('Program trying to communicate...');
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    p3dx.vrep.simxFinish(-1); % just in case, close all opened connections
    p3dx.clientID = p3dx.vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (p3dx.clientID>-1)
        disp('Connected to remote API server');
        p3dx.vrep.simxAddStatusbarMessage(p3dx.clientID,'Connected to Matlab',p3dx.vrep.simx_opmode_oneshot);
        disp('Successful communication');
        pause(2);

        % Now try to retrieve data in a blocking fashion (i.e. a service call):
        [erro,objs]= p3dx.vrep.simxGetObjects(p3dx.clientID,p3dx.vrep.sim_handle_all,p3dx.vrep.simx_opmode_blocking);
        if (erro == p3dx.vrep.simx_return_ok)
            fprintf('Number of objects in the V-Rep scene: %d\n',length(objs));
        else
            fprintf('Remote API function call returned with error code: %d\n',erro);
        end

    else
        disp('Failed connecting to remote API server');
        p3dx.vrep.delete(); % call the destructor!
    end

end