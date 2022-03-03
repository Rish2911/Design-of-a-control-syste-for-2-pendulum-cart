%% D. Design of a LQR controller

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

B = [0; 1/M; 0; 1/(M*l1); 0; 1/(M*l2)];

% Assuming the outputs the same as the states, the C and D matrix of the
% state space model turn out to be :

C = eye(size(A,1));
D = zeros(size(A,1),1);

% Defining state space model for given A,B,C,D
sys = ss(A,B,C,D);

%% Controllability of the system

num_uncon_states = length(A) - rank(ctrb(A,B))

%% LQR controller design

% states : Y = [X, X_dot, theta1, theta1_dot, theta2, theta2_dot

% Initial values for the states
Y_initial = [5;0;20;0;40;0];


%% Case 1 : weights = 1
% Initializing the initial weights in Q and R as 1
Q=1*eye(size(A,1));
R=1;

figure
initial(sys,Y_initial)
grid on   %grid lines visible

[K_val, P_mat, Poles] = lqr(A,B,Q,R);
K_val %computes the K matrix and displays
P_mat %positive definite matrix calculated for the same
Poles %To see the poles of the given equation
sys2 = ss(A-(B*K_val),B,C,D); %Using the K matrix to define ss
figure
initial(sys2,Y_initial)
grid on

%% Case 1 : weights = 1
% Initializing the initial weights in Q and R as 1
Q=1*eye(size(A,1));
R=1;

figure
initial(sys,Y_initial)
grid on   %grid lines visible

[K_val, P_mat, Poles] = lqr(A,B,Q,R);
K_val %computes the K matrix and displays
P_mat %positive definite matrix calculated for the same
Poles %To see the poles of the given equation
sys2 = ss(A-(B*K_val),B,C,D); %Using the K matrix to define ss
figure
initial(sys2,Y_initial)
grid on
