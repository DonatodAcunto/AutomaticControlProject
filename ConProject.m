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
  m1 = 100;   % kg
  m2 = 15;    % kg 
  c2 = 30;    % Ns/m
  k1 = 15;    % kN/m
  k2 = 2;     % kN/m
  omega1 = sqrt(k1/m1);
  omega2 = sqrt(k2/m2);
  syms t;
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
  Q   = [ q1(t);...
          q2(t)];
  dQ  = [ diff(q1,t);...
          diff(q2,t)];
  dQ2 = [ diff(diff(q1,t));...
          diff(diff(q2,t))];
      
% Input Matrix
  U = [ u;...
        0];
    
% Final Equation of Motion    
  U(q1, q2) = M*dQ2 + C*dQ + K*Q;
  
   
%% State Space Representation

sys = ss(A, B, C, 0);

   
   
