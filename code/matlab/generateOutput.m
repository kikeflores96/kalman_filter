%% ------------------------------------------------------------------------ 
% Controladores en modelo no lineal
%--------------------------------------------------------------------------
clear all
close all
% Importamos el controlador previamente ajustado
load drone_multiloop.mat 'K'
controller = ss(K);

filename = 'input.csv';
input = csvread(filename)';


dt = 60e-3;
u_ekf = input(1:3,:);
u_tcf = input(4:6,:);
t = dt*[0:1:length(input)-1];

u = u_ekf;

t = t(360:420) - t(360);
u = u(:, 360:420);

u(3,:) = u(3,:) - u(3,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Simulamos la respuesta del controlador
% La entrada del controlador es el yaw_error = yaw_ref - yaw_measured
% Para una ref yaw, pitch, roll = 0 el error = -medicion

y = lsim(controller, -u, t);

% Torque de roll
tau_roll = y(:,1);
% Torque de pitch
tau_pitch = y(:,2);
% Torque de yaw
tau_yaw = y(:,3);

% Constante de proporcionalidad entre la señal de entrada y la fuerza del
% motor en N
k1      = 10;        % [N/input^2]         
k2      = 2;        % [N*m/input^2]
l       = 0.15;     % [m] brazo del par de roll/pitch
m       = 1;        % Masa del drone en [kg]
g       = 9.91;     % [m/s^2]

% Matriz de mezclado

M = [  k1,    k1,    k1,   k1; 
    -l*k1, -l*k1,  l*k1, l*k1; 
     l*k1, -l*k1, -l*k1, l*k1;
       k2,   -k2,    k2,  -k2];

% La fuerza total a lo largo del tiempo debe ser igual a la masa del drone
F = m*g*ones(length(tau_yaw),1);
% Matriz con los valores de torque y fuerza (salida del controlador a lo
% largo del tiempo
tau = [F, tau_roll, tau_pitch, tau_yaw];
% Evolución temporal del cuadrado de la input
Rotor2 = tau*(inv(M))';
% Evolución temporal de la consigna [0, 1] de cada rotor a lo largo del
% tiempo
Rotor = Rotor2.^(0.5);

figure
plot(t, u'*180/pi)

figure
plot(t, Rotor')