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
axis(3*[-1 1 -1 1 0 1]);
view(-21,30);
hold on;
grid on;


T = 60;
w = 2*pi/T;
r = 2;

inc = 0.1;
tsim = 60;

t = tic;
tinc = tic;

% while toc(t) < tsim
%     if tinc > inc
%         tinc = tic;
        
        % Robots
%         x1 = 1 + r*cos(w*toc(t));
%         y1 = 1 + r*sin(w*toc(t));
%         z1 = 2 + sin(w*toc(t));
%         
%         % Robot 2 (R2)
%         x2 = 0 + r*cos(w*toc(t));
%         y2 = 1 + r*sin(w*toc(t));
%         z2 = 2 + sin(w*toc(t));
%         
%         % Robot 3 (R3)
%         x3 = 1 + r*cos(w*toc(t));
%         y3 = 0 + r*sin(w*toc(t));
%         z3 = 2 + sin(w*toc(t));
        
        x1 = 0;
        y1 = 0;
        z1 = 2 + sin(w*toc(t));
        
        % Robot 2 (R2)
        x2 = 0;
        y2 = 1;
        z2 = 2;
        
        % Robot 3 (R3)
        x3 = 1;
        y3 = 0;
        z3 = 2;
        
        % Calculating the transformation
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
        
        %%
        
        r = sqrt(p^2 + q^2 - 2*p*q*cos(beta));
        h = sqrt(0.5*(p^2 + q^2 - 0.5*r^2));
        alpha = acos((p^2 + q^2 - 0.25*r^2)/(2*p*q));
        
        %%  Calculating the transformation
        
        % Robot 1 (R1)
        x1d = 2/3*h*cos(theta)*cos(psi) + x;
        y1d = 2/3*h*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) + y;
        z1d = - 2/3*h*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) + z;
        
        % Robot 2 (R2)
        x2d = x1d - p*cos(theta)*cos(alpha + psi);
        y2d = y1d - p*cos(phi)*sin(alpha + psi) - q*sin(phi)*sin(theta)*cos(alpha + psi);
        z2d = z1d + p*sin(phi)*sin(alpha + psi) + q*cos(phi)*sin(theta)*cos(alpha + psi);
        
        
        % Robot 3 (R3)
        x3d =  2/3*h*cos(alpha)*cos(theta)*cos(psi) ...
            - 2/3*h*sin(alpha)*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) ...
            + x;
        y3d =   2/3*h*cos(alpha)*cos(theta)*sin(psi) ...
            - 2/3*h*sin(alpha)*(sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) ...
            + y;
        z3d = - 2/3*h*cos(alpha)*sin(theta) ...
            - 2/3*h*sin(alpha)*sin(phi)*cos(theta)...
            + z;
        
%         try
%             delete(fig(1));
%             delete(fig(2));
%             delete(triangle);
%             delete(triangled);
%         catch
%         end
%         
        fig(1) = plot3([x1 x2 x3],[y1 y2 y3],[z1 z2 z3],'b*');
        fig(2) = plot3([x1d x2d x3d],[y1d y2d y3d],[z1d z2d z3d],'mo');
        
        triangle = plot3([x1 x2 x3 x1],[y1 y2 y3 y1],[z1 z2 z3 z1], '-b','LineWidth',1.5);
        triangled = plot3([x1d x2d x3d x1d],[y1d y2d y3d y1d],[z1d z2d z3d z1d], '--m','LineWidth',1.5);
%         
%         drawnow;
%         
%     end
% end
% 
