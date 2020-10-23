% Calcular Modelo Euler-Lagrange
clearvars
clc

tic

% Criar variaveis
syms x(t) dx(t) y(t) dy(t) z(t) dz(t) 
syms ph(t) dph(t) th(t) dth(t) ps(t) dps(t) 
syms a(t) da(t) b(t) db(t) 
syms m mc Ix Iy Iz I real 
syms r g real

W = [1,       0,        -sin(th);  ...
     0,  cos(ph), -sin(ph)*cos(th);  ...
     0, -sin(ph),  cos(th)*cos(ph)];
   
J = diag([Ix, Iy, Iz]);

deta = [dph dth dps]';

D = sqrt(1-sin(a)^2-sin(b)^2);

dxc = dx + r*cos(a)*da;
dyc = dy + r*cos(b)*db;
dzc = dz - r/D*(cos(a)*da + cos(b)*b);


% Lagrangiano
K = 1/2*m*(dx^2+dy^2+dz^2) + 1/2*mc*(dxc^2+dyc^2+dzc^2) + ...
    1/2*transpose(deta)*transpose(W)*J*W*deta +1/2*I*(da^2+db^2);
U = m*g*z + mc*g*(z+r*D);

L = K-U;

% Aplicar restricao de Euler-Lagrange
q = {a b ph th ps x y z };
dqp = {da db dph dth dps dx dy dz};
for ii = 1:length(dqp)
    dLdqp(ii,1) = functionalDerivative(L,dqp{ii});
end

ddLdqpdt = diff(dLdqp);

syms ddx ddy ddz ddph ddth ddps dda ddb

dvar = {diff(a(t), t) ...
    diff(b(t), t)...
    diff(ph(t), t) ...
    diff(th(t), t) ...
    diff(ps(t), t) ...
    diff(x(t), t) ...
    diff(y(t), t) ...
    diff(z(t), t)};

ddvar = {diff(da(t), t) ...
    diff(db(t), t) ...
    diff(dph(t), t) ...
    diff(dth(t), t) ...
    diff(dps(t), t) ...
    diff(dx(t), t) ...
    diff(dy(t), t) ...
    diff(dz(t), t)       
};
    
ddX = {dda ddb ddph ddth ddps ddx ddy ddz};

dd = diff(dLdqp);

%% 
% Matriz de Inercia
for ii = 1:8
    M(:,ii)  = simplify(diff(subs(dd,ddvar{ii},ddX{ii}),ddX{ii}));
    dM(:,ii) = simplify(diff(M(:,ii)));
end

for ii = 1:8
    dM = subs(dM,dvar{ii},dqp{ii});
end

% Calcular os simbolos de Christoffel
for ii = 1:8
    for jj = ii:8
        for kk = 1:8
            c(ii,jj,kk) = 1/2*(...
                functionalDerivative(M(kk,jj),q{ii}) + ...
                functionalDerivative(M(kk,ii),q{jj}) - ...
                functionalDerivative(M(ii,jj),q{kk}));
            c(jj,ii,kk) = c(ii,jj,kk);
        end
    end
end 
%% Armando matriz de Coriolis    

% syms C
for kk = 1:8
    for jj = 1:8
        Cs = 0;
        for ii = 1:8
             Cs = Cs + c(ii,jj,kk)*dqp{ii};
        end
        C(kk,jj) = Cs;
    end
end

C = simplify(C);

syms G
for ii = 1:8
    G(ii,1) = simplify(functionalDerivative(U,q{ii}));
end

%% Substituicoes
% dq
N = simplify(dM - 2*C);
diag(N)
simplify(N+N.')
toc

%% editado em 19/02/19
%Substituindo variáveis para sistema AuRoRA
% q_a1 = {'q1'    'q2'    'q3'    'q4'    'q5'    'q6'    'q7'    'q8'   };
% q_a2 = {'dq1'   'dq2'   'dq3'   'dq4'   'dq5'   'dq6'   'dq7'   'dq8'  };
% q_a3 = {'ddq1'  'ddq2'  'ddq3'  'ddq4'  'ddq5'  'ddq6'  'ddq7'  'ddq8' };
% M = subs(M,{q dqp ddX},{q_a1 q_a2 q_a3});
% 
% M = subs(M,q,q_a1);
% M = subs(M,dqp,q_a2);
% M = subs(M,ddX,q_a3);
% 
% C = subs(C,q,q_a1);
% C = subs(C,dqp,q_a2);
% C = subs(C,ddX,q_a3);
% 
% G = subs(G,q,q_a1);
% G = subs(G,dqp,q_a2);
% G = subs(G,ddX,q_a3);
% 

dq  = [da db dph dth dps dx dy dz];
ddq = [dda ddb ddph ddth ddps ddx ddy ddz];
syms Ta Tb Tph Tth Tps Fx Fy Fz
F = [Ta Tb Tph Tth Tps Fx Fy Fz];
for ii = 1:8
    EqM(ii) = M(ii,:)*ddq.' + C(ii,:)*dq.' + G(ii) == F(ii);
end

for ii = 1:8
    for jj = 1:8
        try
            EqMi(ii,jj) = isolate(EqM(ii), ddq(jj));
        catch
            EqMi(ii,jj) = 'NaN';
            fprintf('Não existe %s na equação %i \n',[ddq(jj)  ii])
            continue
        end
    end
end



