load('EOMs.mat');
M = symEOM.M;
C = symEOM.C;
N = symEOM.N;

tau = symEOM.tau;

syms q1 q2 q1d q2d tau1 real;

q=[q1;q2];
qdot=[q1d;q2d];
variables = [q;qdot;tau1];
%Equation => M*qddot + C*qdot + N = tau
qddot=M^(-1)*(-C*qdot-N+tau);

%state matrix
m = [q;qdot];
mdot = [qdot;qddot];

%Input
n = tau1;

%Output
o = [q2];


%{
finding equilibrium values
q_e = [1.8;0.63];
q1_e = q_e(1);
q2_e = q_e(2);
v1_e = 0;
v2_e = 0;
m_e = [q1_e; q2_e; v1_e; v2_e];
m_estr = mat2str(m_e);
%}


Qddot= subs(qddot,[q1,q1d,q2d],[1.09*pi,0,0]);


EoM_function = matlabFunction(Qddot,'vars',{variables});
var_guess = [1.09*pi;0;0;0;0];
var_sol = fsolve(EoM_function,var_guess);
var_equilibrium = var_sol;


Asym = jacobian(mdot,m);
Bsym = jacobian(mdot,n);
Csym = jacobian(o,m);
Dsym = jacobian(o,n);

%finding Torque
n_e = double(subs((M*[0;0]+C*qdot+N),[variables],[var_equilibrium]));
o_estr = mat2str(var_equilibrium(2));


A = simplify(subs(Asym,[m;n],[var_equilibrium]));
A = double(A);
B = subs(Bsym,[m;n],[var_equilibrium]);
B  =  double(B);
C = subs(Csym,[m;n],[var_equilibrium]);
C = double(C);
%D = double(subs(Dsym,[m;n],[var_equilibrium]));


%Observer Design
Qo = [0.50];
Ro = [3.30 -0.10 -0.15 -0.25; -0.10 4.70 0.35 -0.35; -0.15 0.35 4.90 0.40; -0.25 -0.35 0.40 4.70];

L = lqr(A',C',Ro^(-1),Qo^(-1))';

%Controller Design
Qc = [3.70 0.05 0.40 -0.05; 0.05 4.80 0.25 -0.15; 0.40 0.25 3.80 0.00; -0.05 -0.15 0.00 3.50];
Rc = [1.20];

K = lqr(A,B,Qc,Rc);
Kref= -1/(C*inv(A-B*K)*B);

%{
Analysis of Time delay
syms s tau
Td = (2-s*tau)/(2+s*tau);
G = K*inv(s*eye(size(A))-(A-B*K-L*C))*L;
H = C*inv(s*eye(size(A))-A)*B;
F = (-K*inv(s*eye(size(A))-(A-B*K-L*C))*B*Kref+Kref)/G;

T = Td*H*G*F/(1+Td*H*G);
X = simplify(T);

[n,d] = numden(X);
tmax = 0;

while (roots(sym2poly(subs(d,tau,tmax)))<0)
    tmax = tmax +0.0001;
end
fprintf('\nTau_max = %f\n', tmax)
%}