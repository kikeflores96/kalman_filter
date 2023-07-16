%% ------------------------------------------------------------------------
% Controladores en modelo no lineal
%--------------------------------------------------------------------------
clear all
close all
% Importamos el controlador previamente ajustado
load drone_multiloop.mat 'K'
controller = ss(K);


filename = 'input.csv';
filename = 'pitch.csv';
filename = 'roll.csv';
filename = 'yaw.csv';
input = csvread(filename)';


dt = 60e-3;
u_ekf = input(1:3,:);
u_tcf = input(4:6,:);
t = dt*[0:1:length(input)-1];

fc = 0.2;

u = u_ekf;

yaw = u(3,:);
yaw_dot = zeros(length(u), 1);
yaw_dot_raw = zeros(length(u), 1);

for i=2:length(u)
    yaw_dot_raw(i) = (yaw(i) - yaw(i-1))/dt;
    yaw_dot(i) = yaw_dot(i-1) + dt/(fc + dt)*(yaw_dot_raw(i) - yaw_dot(i-1));
end


% plot(t, yaw_dot); hold on;
% plot(t, yaw_dot_raw)


u(3,:) = yaw_dot;
% u(3,:) = 0;

% t = t(360:420) - t(360);
% u = u(:, 360:420);
%
% u(3,:) = u(3,:) - u(3,1);
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
k2      = 0.5;      % [N*m/input^2]
l       = 0.10;     % [m] brazo del par de roll/pitch
m       = 1.0;        % Masa del drone en [kg]
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



f1 = figure(1);
f1.Color = 'w';

subplot(3,1,1)
ax=gca;
ax.FontSize = 14;
yyaxis left;
plot(t, u(1:2,:)'*180/pi)
ylabel('Angle [\circ]', 'Fontsize', 20)

yyaxis right;
plot(t, u(3,:)*180/pi);
ylabel('$\dot{\psi} [^\circ/s]$', 'Fontsize', 20, 'interpreter', 'latex')
xlim([0, 10]);
legend('\phi', '\theta', 'Fontsize', 16);

grid on
subplot(3,1,2)

plot(t, tau_roll), hold on
plot(t, tau_pitch), hold on
plot(t, tau_yaw)
ax=gca;
ax.FontSize = 14;

xlim([0, 10]);
legend('\tau_\phi', '\tau_\theta', '\tau_\psi', 'Fontsize', 16);
ylabel('Torque [N m]', 'Fontsize', 20)
grid on
subplot(3,1,3)

plot(t, Rotor')
ax=gca;
ax.FontSize = 14;
xlim([0, 10]);
legend('U_1', 'U_2', 'U_3', 'U_4', 'Fontsize', 14);
ylabel('Input [-]', 'Fontsize', 20)
xlabel('t[s]', 'Fontsize', 20)
grid on

f1.Position(3:4)=[1200,800];
