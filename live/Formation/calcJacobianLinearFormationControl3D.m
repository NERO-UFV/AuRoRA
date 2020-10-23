%% Calculating the direct and inverse Jacobian matrix

% This code calculates the Jacobian matrix from the relationships between 
% the robot variables and the formation variables. Taking the first time
% derivative of the forward and the inverse kinematics transformations we
% can obtain the relationship between the dX and dQ, represented by the
% Jacobian matrix, which is given by dQ = J(X)dX in the forward way, and by
% dX = J^-1(Q)dQ in the inverse way, where J(X) = dQ_nx1 / dX_mx1 and the 
% inverse Jacobian is J^-1(Q) = dX_mx1 / dQ_nx1, for m,n = 1,2,...,9.

%% Initial commands
close all; 
clearvars; 
clc;
syms x1 y1 z1    x2 y2 z2    x3 y3 z3    real

%% Jacobian Matrix J(X)
% Definign equations in the direct way ( X --->> Q )

% Position (P)
xF = x1;  % xF
yF = y1;  % yF
zF = z1;  % zF

% Shape (S)                                          
rho = sqrt((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2);       % rho
alpha = asin((x2-x1)/rho) ;                          % alpha
beta = asin((y2-y1)/rho);                            % beta   

% Orientation (O)
phi = atan2((2*y1 - y2 - y3),(2*z1 - z2 - z3));
theta = -atan2((2*x1 - x2 - x3),(2*z1 - z2 - z3));
psi = atan2((2*x1 - x2 - x3),(2*y1 - y2 - y3));

J = jacobian...
    ([  xF   yF   zF         rho  alpha   beta  ],...
     [  x1   y1   z1         x2    y2      z2    ]);
            
J = simplify(J);

%% Inverse Jacobian Matrix J^-1(Q)
% Definign equations in the inverse way ( Q --->> X )

% Clearing unnecessary variables and defining new ones
clearvars -except J; 
syms xF yF zF    rho alpha beta real

% Robot 1 (R1)
x1 = xF;
y1 = yF;
z1 = zF;

% Robot 2 (R2)
x2 = xF + rho*sin(alpha);
y2 = yF + rho*sin(beta);
z2 = zF + rho*sqrt(1 - (sin(alpha))^2 - (sin(beta))^2);

Jinv = jacobian...
    ([  x1   y1   z1         x2   y2      z2       ],...
     [  xF   yF   zF         rho  alpha   beta     ]);
            
Jinv = simplify(Jinv);

clearvars -except   J    Jinv; 

%% Printing in Command Window the results

% Jacobian
clc;
fprintf('___________________________________________________________________________________\n');
fprintf('\n                           Jacobian Matrix   | J(X) | \n');
fprintf('___________________________________________________________________________________\n\n');


for i = 1:size(J,1)
    fprintf(['\n\n _______________________  Line ' num2str(i) ':  _______________________\n\n']);
    for j = 1:size(J,1)
        fprintf(['\n [' num2str(j) '] ---> J(' num2str(i) ',' num2str(j) '):  ']);
        disp(J(i,j));
    end
    fprintf('__________________________________________________________\n\n');
end

% Inverse Jacobian
fprintf('__________________________________________________________\n');
fprintf('\n         Inverse Jacobian Matrix   | J^-1(Q) | \n');
fprintf('__________________________________________________________\n');
for i = 1:size(Jinv,1)
    fprintf(['\n\n _______________________  Line ' num2str(i) ':  _______________________\n\n']);
    for j = 1:size(Jinv,1)
        fprintf(['\n [' num2str(j) '] ---> J^-1(' num2str(i) ',' num2str(j) '):  ']);
        disp(Jinv(i,j));
    end
    fprintf('__________________________________________________________\n\n');
end

%% Creating the Jacobian Matrix
Jaut = char([newline newline 'J = [ ']);
for i = 1:size(J,1)
    if i ~= 1
        Jaut = [ Jaut newline '        '];
    end
    for j = 1:size(J,1)
        if j ~= 6
            Jaut = [ Jaut , char(J(i,j)) ' , '];
        elseif j == 6 && i == 7
            Jaut = [ Jaut , char(J(i,j)) ''];
        else
            Jaut = [ Jaut , char(J(i,j)) ' ; '];
        end
    end
end
Jaut = [ Jaut ' ];' newline newline];

%% Test Jacobian Matrix
% Random values
x1 = random('Normal',0,3);
y1 = random('Normal',0,3);
z1 = random('Normal',0,3);

x2 = random('Normal',0,3);
y2 = random('Normal',0,3); 
z2 = random('Normal',0,3);

fprintf('\n\n___________________________________________________________________________________\n\n');
fprintf('                                      FINAL RESULTS                                    ');
fprintf('\n___________________________________________________________________________________\n\n');


fprintf('\n\n(i) Jacobian Matrix obtained: \n\n');
fprintf('J = ');
disp(J);
fprintf('        - Copy this equation and paste inside the code: J = [];');
fprintf('\n\n        - Do the necessary changes.\n\n');

fprintf('___________________________________________________________________________________\n\n');

fprintf('\n(ii) Jacobian Matrix with the correct syntax to Matlab code');
disp(Jaut);
fprintf('          - Copy this equation and paste inside the code: Jsynt = [];\n');
fprintf('\n          - No changes are necessary, just copy and paste.');
fprintf('\n\n          ---> Then run the rest of the code.\n\n');

fprintf('___________________________________________________________________________________\n\n');

fprintf('\n          ... Applying random values in robots variables:  X = [ x1 y1 z1 ... x3 y3 z3 ];\n');


%% Change this
% J_begin
J = [                                                                                                                                                                    1,                                                                                                                                                                    0,                                                                                                                                               0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                0;
                                                                                                                                                                         0,                                                                                                                                                                    1,                                                                                                                                               0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                0;
                                                                                                                                                                         0,                                                                                                                                                                    0,                                                                                                                                               1,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                0;
                                                                                                                 (x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                                                                                                            (y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                                                                                       (z1 - z2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                                                                                                          -(x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                                                                                                          -(y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                                                                                       -(z1 - z2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2);
      -(y1^2 - 2*y1*y2 + y2^2 + z1^2 - 2*z1*z2 + z2^2)/((1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)),                      ((2*y1 - 2*y2)*(x1 - x2))/(2*(1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), ((2*z1 - 2*z2)*(x1 - x2))/(2*(1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), (y1^2 - 2*y1*y2 + y2^2 + z1^2 - 2*z1*z2 + z2^2)/((1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)),                    -((2*y1 - 2*y2)*(x1 - x2))/(2*(1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), -((2*z1 - 2*z2)*(x1 - x2))/(2*(1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2));
                           ((2*x1 - 2*x2)*(y1 - y2))/(2*(1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), -(x1^2 - 2*x1*x2 + x2^2 + z1^2 - 2*z1*z2 + z2^2)/((1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), ((2*z1 - 2*z2)*(y1 - y2))/(2*(1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)),                    -((2*x1 - 2*x2)*(y1 - y2))/(2*(1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), (x1^2 - 2*x1*x2 + x2^2 + z1^2 - 2*z1*z2 + z2^2)/((1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), -((2*z1 - 2*z2)*(y1 - y2))/(2*(1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2))];
 
% J_end

% Jaut_begin
Jaut = [ 1 , 0 , 0 , 0 , 0 , 0 ; 
        0 , 1 , 0 , 0 , 0 , 0 ; 
        0 , 0 , 1 , 0 , 0 , 0 ; 
        (x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2) , (y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2) , (z1 - z2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2) , -(x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2) , -(y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2) , -(z1 - z2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2) ; 
        -(y1^2 - 2*z1*z2 - 2*y1*y2 + y2^2 + z1^2 + z2^2)/((1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)) , ((2*y1 - 2*y2)*(x1 - x2))/(2*(1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)) , ((2*z1 - 2*z2)*(x1 - x2))/(2*(1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)) , (y1^2 - 2*z1*z2 - 2*y1*y2 + y2^2 + z1^2 + z2^2)/((1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)) , -((2*y1 - 2*y2)*(x1 - x2))/(2*(1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)) , -((2*z1 - 2*z2)*(x1 - x2))/(2*(1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)) ; 
        ((2*x1 - 2*x2)*(y1 - y2))/(2*(1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)) , -(x1^2 - 2*z1*z2 - 2*x1*x2 + x2^2 + z1^2 + z2^2)/((1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)) , ((2*z1 - 2*z2)*(y1 - y2))/(2*(1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)) , -((2*x1 - 2*x2)*(y1 - y2))/(2*(1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)) , (x1^2 - 2*z1*z2 - 2*x1*x2 + x2^2 + z1^2 + z2^2)/((1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)) , -((2*z1 - 2*z2)*(y1 - y2))/(2*(1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)) ;  ];
    
% Jaut_end

fprintf('\n\n(iii) Jacobian Matrix with numeric values\n');
fprintf('\n\n          (J) Jacobian Matrix with manual correction after copy and paste:\n\n');
disp(real(J));
fprintf('\n\n          (Jaut) Jacobian Matrix with automatic syntax correction:\n\n');
disp(real(Jaut));

fprintf('___________________________________________________________________________________\n\n');

fprintf('\n(iv) The final question: ');
fprintf('\n\n          ... J = Jaut?\n\n');
C = J == Jaut;
disp(C);


fprintf('___________________________________________________________________________________\n\n');

fprintf('\n(v) Inverse Jacobian Matrix with the correct syntax to Matlab code\n\n');

Jinvaut = char([newline newline 'Jinvaut = [ ']);
for i = 1:size(Jinv,1)
    if i ~= 1
        Jinvaut = [ Jinvaut newline '        '];
    end
    for j = 1:size(Jinv,1)
        if j ~= 6
            Jinvaut = [ Jinvaut , char(Jinv(i,j)) ' , '];
        elseif j == 6 && i == 7
            Jinvaut = [ Jinvaut , char(Jinv(i,j)) ''];
        else
            Jinvaut = [ Jinvaut , char(Jinv(i,j)) ' ; '];
        end
    end
end
Jinvaut = [ Jinvaut ' ];' newline newline];
disp(Jinvaut);
fprintf('\n          - No changes are necessary, just copy and paste.');

fprintf('___________________________________________________________________________________\n\n');

fprintf('\n          ... Applying random values in formation variables:  Q = [ xF yF zF rho alpha beta ];\n');

% Jaut_begin
Jinvaut = [ 1 , 0 , 0 , 0 , 0 , 0 ; 
        0 , 1 , 0 , 0 , 0 , 0 ; 
        0 , 0 , 1 , 0 , 0 , 0 ; 
        1 , 0 , 0 , sin(alpha) , rho*cos(alpha) , 0 ; 
        0 , 1 , 0 , sin(beta) , 0 , rho*cos(beta) ; 
        0 , 0 , 1 , (1 - sin(beta)^2 - sin(alpha)^2)^(1/2) , -(2^(1/2)*rho*sin(2*alpha))/(2*(cos(2*alpha) + cos(2*beta))^(1/2)) , -(2^(1/2)*rho*sin(2*beta))/(2*(cos(2*alpha) + cos(2*beta))^(1/2)) ;  ];

% Jaut_end

fprintf('\n\n(iii) Jacobian Matrix with numeric values\n');
fprintf('\n\n          (J) Jacobian Matrix with manual correction after copy and paste:\n\n');
disp(real(Jinvaut));


fprintf('___________________________________________________________________________________\n\n');

