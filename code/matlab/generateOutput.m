%% ------------------------------------------------------------------------ 
% Controladores en modelo no lineal
%--------------------------------------------------------------------------
clear all

% Importamos el controlador previamente ajustado
load drone_multiloop.mat 'K'
controller = ss(K);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generamos una signal de entrada y un vector de tiempos
% Esta parte será remplazada por la lectura de un archivo csv de generado
% con python

omega=1;
t = linspace(0, 10, 1001);
u = [zeros(1,length(t)); zeros(1,length(t)); sin(1*t)];
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
k1      = 5;        % [N/input^2]         
k2      = 1;        % [N*m/input^2]
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
plot(t, tau_roll, t, u(1)); hold on;

figure

plot(t, Rotor2')