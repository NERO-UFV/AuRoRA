%% Triangular Formation 3D

% Q = [  | x    y    z  |    | p   q     beta |    | phi   theta   psi |  ]
% X = [  | x1   y1   z1 |    | x2   y2   z2 |      | x3    y3      z3  |  ]
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  %

% Initial Comands

clear; close all; clc;

try
    fclose(instrfindall);
catch
end

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

% Look for root folder

PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Load Classes

figure;
axis(4*[-1 1 -1 1 0 1]);
view(-21,30);
hold on;
grid on;


T = 60;
w = 2*pi/T;
r = 0.4;

inc = 0.1;
tsim = 60;

t = tic;
tinc = tic;

while toc(t) < tsim
    if tinc > inc
        tinc = tic;
        
        %% Robots position
       
% % %         x1 = 1 + r*cos(w*toc(t));
% % %         y1 = 0 + r*sin(w*toc(t));
% % %         z1 = 1 + sin(w*toc(t));
% % %         
% % %         % Robot 2 (R2)
% % %         x2 = -1 + r*cos(w*toc(t));
% % %         y2 = 0 + r*sin(w*toc(t));
% % %         z2 = 1 + sin(w*toc(t));
% % %         
% % %         % Robot 3 (R3)
% % %         x3 = 0 + r*cos(w*toc(t));
% % %         y3 = 1 + r*sin(w*toc(t));
% % %         z3 = 1 + sin(w*toc(t));

        x1 = 1.25;  
        y1 = 0;
        z1 = 0;
        
        % Robot 2 (R2)
        x2 = -1.25;
        y2 = 0;
        z2 = 1.5;
        
        % Robot 3 (R3)
        x3 = 0;
        y3 = 1.25;
        z3 = 1.5;
        
        %% Direct Transformation

        % Position (P)
        x = (x1 + x2 + x3)/3;
        y = (y1 + y2 + y3)/3;
        z = (z1 + z2 + z3)/3;
        
        % Shape (S)
        p = sqrt((x1 - x2)^2 + (y1-y2)^2 + (z1-z2)^2);
        q = sqrt((x1 - x3)^2 + (y1-y3)^2 + (z1-z3)^2);
        r = sqrt((x2 - x3)^2 + (y2-y3)^2 + (z2-z3)^2);
        beta = acos((p^2 + q^2 - r^2)/(2*p*q));
        
        % Orientation (O)
        phi = atan2((2*z1 - z2 - z3),(2*y1 - y2 - y3)); % Roll (x-axis)
        theta = -atan((2*z1 - z2 - z3)/(2*x1 - x2 - x3)); % Pitch (y-axis)
        psi = atan2((2*y1 - y2 - y3),(2*x1 - x2 - x3)); % Yaw (z-axis)
        
        
        % Position (P)
        x = 0;
        y = 0;
        z = 2;
        
        % Shape (S)
        p = 1;
        q = 1;
        r = 1;
        beta = pi/3;
        
        % Orientation (O)
        phi = pi; % Roll (x-axis)
        theta = 0; % Pitch (y-axis)
        psi = 0; % Yaw (z-axis)
        
        %% Inverse Transformation
        
        r = sqrt(p^2 + q^2 - 2*p*q*cos(beta));
        h1 = sqrt(0.5*(p^2 + q^2 - 0.5*r^2));
        h2 = sqrt(0.5*(r^2 + p^2 - 0.5*q^2));
        h3 = sqrt(0.5*(q^2 + r^2 - 0.5*p^2));
        alpha1 = acos((4*(h1^2 + h2^2) - 9*p^2)/(8*h1*h2));
        alpha2 = acos((4*(h1^2 + h3^2) - 9*q^2)/(8*h1*h3));
        
        % Robot 1 (R1)
        x1d = 2/3*h1*cos(theta)*cos(psi) + x;
        y1d = 2/3*h1*cos(theta)*sin(psi) + y;
        z1d = - 2/3*h1*sin(theta) + z;
        
        % Robot 2 (R2)
        x2d =  2/3*h2*cos(alpha1)*cos(theta)*cos(psi) ...
            + 2/3*h2*sin(alpha1)*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) ...
            + x;
        y2d =  2/3*h2*cos(alpha1)*cos(theta)*sin(psi) ...
            + 2/3*h2*sin(alpha1)*(sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) ...
            + y;
        z2d = - 2/3*h2*cos(alpha1)*sin(theta) ...
            + 2/3*h2*sin(alpha1)*sin(phi)*cos(theta)...
            + z;

        % Robot 3 (R3)
        x3d =  2/3*h3*cos(alpha2)*cos(theta)*cos(psi) ...
            - 2/3*h3*sin(alpha2)*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) ...
            + x;
        y3d =   2/3*h3*cos(alpha2)*cos(theta)*sin(psi) ...
            - 2/3*h3*sin(alpha2)*(sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) ...
            + y;
        z3d = - 2/3*h3*cos(alpha2)*sin(theta) ...
            - 2/3*h3*sin(alpha2)*sin(phi)*cos(theta)...
            + z;
        
        
        % Position (P)
        x = (x1d + x2d + x3d)/3;
        y = (y1d + y2d + y3d)/3;
        z = (z1d + z2d + z3d)/3;
        
        % Shape (S)
        p = sqrt((x1d - x2d)^2 + (y1d-y2d)^2 + (z1d-z2d)^2);
        q = sqrt((x1d - x3d)^2 + (y1d-y3d)^2 + (z1d-z3d)^2);
        r = sqrt((x2d - x3d)^2 + (y2d-y3d)^2 + (z2d-z3d)^2);
        beta = acos((p^2 + q^2 - r^2)/(2*p*q));
        
        % Orientation (O)
        phi = atan2((2*z1d - z2d - z3d),(2*y1d - y2d - y3d)); % Roll (x-axis)
        theta = -atan((2*z1d - z2d - z3d)/(2*x1d - x2d - x3d)); % Pitch (y-axis)
        psi = atan2((2*y1d - y2d - y3d),(2*x1d - x2d - x3d)); % Yaw (z-axis)
        
        
        r = sqrt(p^2 + q^2 - 2*p*q*cos(beta));
        h1 = sqrt(0.5*(p^2 + q^2 - 0.5*r^2));
        h2 = sqrt(0.5*(r^2 + p^2 - 0.5*q^2));
        h3 = sqrt(0.5*(q^2 + r^2 - 0.5*p^2));
        alpha1 = acos((4*(h1^2 + h2^2) - 9*p^2)/(8*h1*h2));
        alpha2 = acos((4*(h1^2 + h3^2) - 9*q^2)/(8*h1*h3));
        
        % Robot 1 (R1)
        x1 = 2/3*h1*cos(theta)*cos(psi) + x;
        y1 = 2/3*h1*cos(theta)*sin(psi) + y;
        z1 = - 2/3*h1*sin(theta) + z;
        
        % Robot 2 (R2)
        x2 =  2/3*h2*cos(alpha1)*cos(theta)*cos(psi) ...
            + 2/3*h2*sin(alpha1)*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) ...
            + x;
        y2 =  2/3*h2*cos(alpha1)*cos(theta)*sin(psi) ...
            + 2/3*h2*sin(alpha1)*(sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) ...
            + y;
        z2 = - 2/3*h2*cos(alpha1)*sin(theta) ...
            + 2/3*h2*sin(alpha1)*sin(phi)*cos(theta)...
            + z;

        % Robot 3 (R3)
        x3 =  2/3*h3*cos(alpha2)*cos(theta)*cos(psi) ...
            - 2/3*h3*sin(alpha2)*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) ...
            + x;
        y3 =   2/3*h3*cos(alpha2)*cos(theta)*sin(psi) ...
            - 2/3*h3*sin(alpha2)*(sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) ...
            + y;
        z3 = - 2/3*h3*cos(alpha2)*sin(theta) ...
            - 2/3*h3*sin(alpha2)*sin(phi)*cos(theta)...
            + z;
        
        
        try
            delete(fig(1));
            delete(fig(2));
            delete(triangle);
            delete(triangled);
        catch
        end
        
        fig(1) = plot3([x1 x2 x3],[y1 y2 y3],[z1 z2 z3],'b*');
        fig(2) = plot3([x1d x2d x3d],[y1d y2d y3d],[z1d z2d z3d],'mo');
        
        triangle = plot3([x1 x2 x3 x1],[y1 y2 y3 y1],[z1 z2 z3 z1], '-b','LineWidth',1.5);
        triangled = plot3([x1d x2d x3d x1d],[y1d y2d y3d y1d],[z1d z2d z3d z1d], '--m','LineWidth',1.5);
        
        drawnow;
        
    end
end

