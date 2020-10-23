%%

close all; clear all; clc;

syms x1 y1 z1 x2 y2 z2 real

xF = x1;                                             % xF
yF = y1;                                             % yF
zF = z1;                                             % zF
rho = sqrt((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2);       % rho
alpha = asin((x2-x1)/rho) ;                          % alpha
beta = asin((y2-y1)/rho);                            % beta              

Jdir = jacobian([   xF     yF     zF     rho     alpha   beta   ],...
                [   x1     y1     z1     x2      y2      z2     ]);
            
Jdir = simplify(Jdir);

%% 

% % % % close all; clear all; clc;
% % % 
% % % syms xF yF zF rho alpha beta
% % % 
x1 = xF;
y1 = yF;
z1 = zF;
x2 = xF + rho*sin(alpha);
y2 = yF + rho*sin(beta);
z2 = zF + rho*sqrt(1 - (sin(alpha))^2 - (sin(beta))^2);

Jinv = jacobian([   x1     y1     z1     x2      y2      z2     ],...
                [   xF     yF     zF     rho     alpha   beta   ]);        
         
%%

Jinv = simplify(Jinv)

% % % fprintf('J(1,1): ');
% % % disp(J(1,1));
% % % 
% % % fprintf('J(1,2): ');
% % % disp(J(1,2));
% % % 
% % % fprintf('J(1,3): ');
% % % disp(J(1,3));
% % % 
% % % fprintf('J(1,4): ');
% % % disp(J(1,4));
% % % 
% % % fprintf('J(1,5): ');
% % % disp(J(1,5));
% % % 
% % % fprintf('J(1,6): ');
% % % disp(J(1,6));
% % % 
% % % fprintf('J(2,1): ');
% % % disp(J(2,1));
% % % 
% % % fprintf('J(2,2): ');
% % % disp(J(2,2));
% % % 
% % % fprintf('J(2,3): ');
% % % disp(J(2,3));
% % % 
% % % fprintf('J(2,4): ');
% % % disp(J(2,4));
% % % 
% % % fprintf('J(2,5): ');
% % % disp(J(1,5));
% % % 
% % % fprintf('J(2,6): ');
% % % disp(J(2,6));




