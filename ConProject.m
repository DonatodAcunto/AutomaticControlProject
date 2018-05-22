%% Automatic Control Project 2018
% Exercise 1 - The Vibration Absorber

%% Definition of the system
% The vibration absorber, also called dynamic vibration 
% absorber, is a mechanical device which is used to reduce or eliminate 
% unwanted vibration. 
% It consists of a mass and a spring attached to the main (or original) 
% mass that needs to be protected from vibration. 
% Vibration absorbers are commonly used in a variety of applications 
% which include sanders, saws, internal combustion engines and 
% high-voltage transmission lines.

%% Equation of Motion
% Definition of the variables and constants
  m1 = 100.0;   % kg
  m2 = 15.0;    % kg 
  c2 = 30.0;    % Ns/m
  k1 = 15.0;    % kN/m
  k2 = 2.0;     % kN/m
  omega1 = sqrt(k1/m1);
  omega2 = sqrt(k2/m2);
  t = 0:1:1000;
% Equation
% Mass Matrix
  M = [ m1, 0;...
        0, m2];
   
% Stiffness spring Matrix
  K = [ k1+k2, -k2;...
       -k2,   k2];
   
% Viscous Damping Coefficients Matrix  
  C = [ c2, -c2;...
       -c2,  c2];
  
% Motion of the system Matrix
 % Q   = [ q1(t);...
  %        q2(t)];
 % dQ  = [ diff(q1,t);...
  %        diff(q2,t)];
 % dQ2 = [ diff(diff(q1,t));...
  %        diff(diff(q2,t))];
      
% Input Matrix
 % U = [ u;...
  %      0];
    
% Final Equation of Motion    
 % U(q1, q2) = M*dQ2 + C*dQ + K*Q;
  
   
%% State Space Representation
% d^2q1/dt + omega1^2*q1 = 1/m1*u
% d^2q2/dt = 0
% What are A, B, C
% R^(4*4)----> A
A = [       0,          1,      0,      0;...
      -(k1+k2)/m1,  -c2/m1,   k2/m1, +c2/m1;...
            0,         0,       0,      1;...
          k2/m2,     c2/m2,  -k2/m2, -c2/m2];
% R^(1*4)----> B
B = [m1^(-1); 0; 0; 0];
C = [1, 0, 0, 0];
%Q1 = [q1, q2, q1', q2'];
%sys = ss(A, B, C, 0);

%% Eigenvalue test for stability

% check that the eigenvalues of matrix A have strictly negative real part.
% we conclude that the system is Globally Exponentially Stable (GES) 
eig(A)



%% Lyapunot stability as an LMI problem 
% clear the internal memory of YALMIP
yalmip('clear')

% set the parameters of the LMI solver
% set the solver settings
opts = sdpsettings;
% chose the solver
opts.solver = 'sdpt3';
% show iterations 0=do not show, 1 = show
opts.verbose = 1;

% set the variable to optimize
% sdpvar stands for semi-definite-programming-variable
% in this case P2 is 2x2 symmetric matrix, 3 unknowns (not 4 due to
% symmetry)
P = sdpvar(4,4,'symmetric');
 
% set the objective function
obj = trace(P);

% set the constraints
constr = [  A'*P + P*A<0;
            P>eye(4)];

% solve the problem
yalmipdiagnostics = solvesdp(constr,obj,opts);

% display the results during iteration
msg = ['feasibility = ', num2str(yalmipdiagnostics.problem)];
disp(msg);

% extract the result
P = double(P);

% check that is a proper Lyapunov function 

eig(A'*P+P*A)

%% LMI Formulation

% compute the size of the matrices
n = length(A);
F=zeros(1, 1);
[ny,nu] = size(F);
% decision variables
P=sdpvar(n); % symmetric n-x-n
gamma=sdpvar(1); % scalar
% define the inequality constraints
M = [ P*A P*B zeros(n,ny);
     zeros(nu,n) -gamma/2*eye(nu) 0;
        C F -gamma/2*eye(ny)];
    
%He function
%constr = set(M+M'<0) + set(P>0); %This command does not work in Matlab R2017b;
constr = [M+M'<0,P>0]
opts=sdpsettings;
opts.solver='sdpt3';
% solve the LMI minimizing gamma
yalmipdiagnostics = solvesdp(constr,gamma,opts)
% evaluate solution variables (if any)
Psol=double(P);
gammasol=double(gamma);
% compare to alternative Hinf norm computation
sys = pck(A,B,C,F);
out = hinfnorm(sys);
disp([out(2) gammasol])

%% Bode Diagram
%G(s) = C(sI-A)^(-1)*B
G = @(s)(C(s*eye(size(A))-A)^(-1)*B);
