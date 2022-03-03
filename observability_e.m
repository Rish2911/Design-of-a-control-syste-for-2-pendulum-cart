%% E. Observabilty of the linearized state 

clc
clear all
close all

%% Parameters and their values

M=1000;     % Cart
m1=100;     % Bob 1
m2=100;     % Bob 2
l1=20;      % Bob 1 string length
l2=10;      % Bob 2 string length
g=9.81;     % Acceleration due to gravity

%% A and B matrices for the state space model

A = [0 1 0 0 0 0;
    0 0 -(m1*g)/M 0 -(m2*g)/M 0;
    0 0 0 1 0 0;
    0 0 -((M+m1)*g)/(M*l1) 0 -(m2*g)/(M*l1) 0;
    0 0 0 0 0 1;
    0 0 -(m1*g)/(M*l2) 0 -(g*(M+m2))/(M*l2) 0];

%B = [0; 1/M; 0; 1/(M*l1); 0; 1/(M*l2)];

% Assuming the outputs the same as the states, the C and D matrix of the
% state space model turn out to be :

% Defining state space model for given A,B,C,D
%sys = ss(A,B,C,D);

%% Observability of the system

% Only x(t)
C1 = [1 0 0 0 0 0];

disp('Number of unobservable states when only x(t) is observed: ')

num_unobs_states_cond1 = length(A) - rank(obsv(A,C1))

% Only theta1(t) and theta2(t)
C2 = [0 0 1 0 0 0; 0 0 0 1 0 0];

disp('Number of unobservable states when only theta1(t) and theta2(t) are observed: ')

num_unobs_states_cond2 = length(A) - rank(obsv(A,C2))

% Only x(t) and theta2(t)
C3 = [0 1 0 0 0 0; 0 0 0 1 0 0];

disp('Number of unobservable states when only x(t) and theta2(t) are observed: ')
num_unobs_states_cond3 = length(A) - rank(obsv(A,C3))



% Only x(t), theta1(t) and theta2(t)
C4 = [1 0 0 0 0 0; 0 0 1 0 0 0;0 0 0 0 1 0];

disp('Number of unobservable states when only x(t), theta1(t) and theta2(t) are observed: ')

num_unobs_states_cond4 = length(A) - rank(obsv(A,C4))

C5 = eye(size(A,1));

disp('Number of unobservable states when all states are observed: ')

num_unobs_states_cond5 = length(A) - rank(obsv(A,C5))








