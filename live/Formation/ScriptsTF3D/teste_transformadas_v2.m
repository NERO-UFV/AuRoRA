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
axis(5*[-1 1 -1 1 0 1]);
view(-21,30);
hold on;
grid on;

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

T = 60;
w = 2*pi/T;
r = 2;

inc = 0.1;
tsim = 60;

t = tic;
tinc = tic;

while toc(t) < tsim
    if tinc > inc
        tinc = tic;
        
        %% Robots position
        
        % Robots
        x1 = 1 + r*cos(w*toc(t));
        y1 = 0 + r*sin(w*toc(t));
        z1 = 2 + sin(w*toc(t));
        
        % Robot 2 (R2)
        x2 = 1 + r*cos(w*toc(t));
        y2 = 1 + r*sin(w*toc(t));
        z2 = 2 + sin(w*toc(t));
        
        % Robot 3 (R3)
        x3 = 0 + r*cos(w*toc(t));
        y3 = 1 + r*sin(w*toc(t));
        z3 = 2 + sin(w*toc(t));
        
        x1 = 1;
        y1 = 0;
        z1 = 0;
        
        % Robot 2 (R2)
        x2 = 0;
        y2 = 0;
        z2 = 0;
        
        % Robot 3 (R3)
        x3 = 0;
        y3 = 1;
        z3 = 0;
        
        
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
        Fx = [x1 y1 z1]' - [x y z]';
        Fz = cross(([x1 y1 z1]' - [x2 y2 z2]'),([x1 y1 z1]' - [x3 y3 z3]'));
        Fy = cross(Fz,Fx);
        
        psi = atan2(Fx(2),Fx(1)); % Roll (x-axis)
        theta = -asin(Fx(3));%,(2*x1 - x2 - x3)); % Pitch (y-axis)
        phi = atan2(Fy(3),Fz(3)); % Yaw (z-axis)      

        
        %% Inverse Transformation
        
        r = sqrt((x2-x3)^2 + (y2-y3)^2 + (z2-z3)^2);
        h = sqrt(0.5*(p^2 + q^2 - 0.5*r^2));
        alpha = acos((p^2 + q^2 - 0.25*r^2)/(2*p*q));
        
        % Robot 1 (R1)
        x1d = x + 2/3*h*cos(theta)*cos(psi);
        y1d = y + 2/3*h*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi));
        z1d = z - 2/3*h*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
        
        % Robot 2 (R2)
        x2d = x1 - p*cos(theta)*cos(alpha + psi);
        y2d = y1 - p*cos(phi)*sin(alpha + psi) - q*sin(phi)*sin(theta)*cos(alpha + psi);
        z2d = z1 + p*sin(phi)*sin(alpha + psi) + q*cos(phi)*sin(theta)*cos(alpha + psi);
        
        % Robot 3 (R3)
        x3d = x1 - q*cos(theta)*cos(beta - alpha - psi);
        y3d = y1 + q*cos(phi)*sin(beta - alpha - psi) - q*sin(phi)*sin(theta)*cos(beta - alpha - psi);
        z3d = z1 - q*sin(phi)*sin(beta - alpha - psi) + q*cos(phi)*sin(theta)*cos(beta - alpha - psi);
        
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

