function sInvKinematicModel(obj,dXr)

    % Determine the robot velocity, based on the reference velocity
    %        +-------------+
    % dXr -> | InvKinematic|  ->  Ur
    %        | Model       |
    %        +-------------+
    %
    % Verify vector length
    l = length(dXr);
    % Inverse Kinematic Matrix (2D)
    if l==2

        Kinv = [cos(obj.pPos.X(6)),            sin(obj.pPos.X(6));
            -sin(obj.pPos.X(6))/obj.pPar.a,  cos(obj.pPos.X(6))/obj.pPar.a];

        % Inverse Kinematic Matrix (3D)
    elseif l==3

        Kinv = [cos(obj.pPos.X(6)),              sin(obj.pPos.X(6)),          0;
            -sin(obj.pPos.X(6))/obj.pPar.a,  cos(obj.pPos.X(6))/obj.pPar.a, 0;
            0,                                 0,                    0;
            0,                                 0,                    0];

        % Ps.: Kinv(4x3) --> Ur(4,1) ==> same pattern as ArDrone's command signals

    else
        disp('Invalid vector length (please verify dXr).');
        Kinv =0;
    end

    % Reference control signal
    obj.pSC.Ur = Kinv*dXr;
end