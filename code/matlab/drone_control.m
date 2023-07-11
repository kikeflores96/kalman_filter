%% ------------------------------------------------------------------------ 
% Analisis del modelo CSTR
%--------------------------------------------------------------------------
clear all

% Definicion de las constantes del sistema


% Valores iniciales para las variables de estado

roll_0  = 0;
pitch_0 = 0;
yaw_0   = 0;
omegaX_0 = 0;
omegaY_0 = 0;
omegaZ_0 = 0;


% Almacena los valores iniciales en el vector de estados x0
x0 = [roll_0;  pitch_0; yaw_0; omegaX_0; omegaY_0; omegaZ_0];

% Almacena los valores de las entradas en el vector de entradas u0
u0 = zeros(3,1);


% y = x --> estado = salida
y0 = [roll_0;  pitch_0; yaw_0];
ix = [1;2;3];
iu = [1; 2];
iy = [1;2];

% Determina el punto de equilibrio del sistema para u = u0
[xs,us,ys,dx] = trim('attitude_drone', x0, u0, [], [], iu, [])


% Linealizacion de la dinamica en torno al punto de equilibrio x = xs
[A,B,C,D] = linmod('attitude_drone', xs, us)

% Define un sistema lineal a partir de las matrices A, B, C , D
% dxdt  = Ax + Bu
% y     = Cx + Du
system = ss(A,B,C,D);
% Evaluar la controlabilidad del sistema a partir de las matrices A y B
Co = ctrb(A, B);
rank(Co)
% Determina la funcion de transferencia del sistema lineal
G = tf(system)


% ------------------------------------------------------------------------
% Diseño del controlador descentralizado
% -------------------------------------------------------------------------
close all

G               = system;
G.InputName     = {'\tau_\phi','\tau_\theta', '\tau_\psi'};      % Renombra las entradas de la FT
G.OutputName    = {'\phi', '\theta', '\psi'};      % Renombra las salidas de la FT

% PID lazo de control tau_roll --> roll
PID_roll              = tunablePID('PID_roll','pid');       % Define un PID ajustable
PID_roll.InputName    = '\Delta\phi';                             % Entrada PID
PID_roll.OutputName   = '\tau_\phi';                        % Salida PID



% PID lazo de control tau_pitch --> pitch
PID_pitch              = tunablePID('PID_pitch','pid');       % Define un PID ajustable
PID_pitch.InputName    = '\Delta\theta';                            % Entrada PID
PID_pitch.OutputName   = '\tau_\theta';                       % Salida PID



% PID lazo de control tau_yaw --> yaw
PID_yaw              = tunablePID('PID_yaw','pid');       % Define un PID ajustable
PID_yaw.InputName    = '\Delta\psi';                             % Entrada PID
PID_yaw.OutputName   = '\tau_\psi';                        % Salida PID


% Error = Referencia - salida
sum1 = sumblk('\Delta\phi = \phi_r - \phi',1);
sum2 = sumblk('\Delta\theta = \theta_r - \theta',1);
sum3 = sumblk('\Delta\psi = \psi_r - \psi',1);
% Define el subsistema a ajustar (Controlador C0) con los bloques ajustables
% correspondientes
C0 = connect(PID_roll, PID_pitch, PID_yaw, sum1, sum2, sum3,...
    {'\phi_r', '\theta_r', '\psi_r', '\phi', '\theta', '\psi'}, {'\tau_\phi', '\tau_\theta', '\tau_\psi'});
 

% Define el rango de frecuencias
wc = [1e-1, 1e3];
% Define las opciones del algoritmo de ajuste
options = looptuneOptions('RandomStart',20);

% Calcula el controlador C ajustado
[G, C, gam, Info] = looptune(G, C0, wc, options);

% Imprime por pantalla las propiedades del controlador ajustado
showTunable(C)

%  Asigna los valores de los PIDS ajustados a los bloques iniciales
PID_roll.Kp = C.Blocks.PID_roll.Kp;
PID_roll.Ki = C.Blocks.PID_roll.Ki;
PID_roll.Kd = C.Blocks.PID_roll.Kd;
PID_roll.Tf = C.Blocks.PID_roll.Tf;

PID_pitch.Kp = C.Blocks.PID_pitch.Kp;
PID_pitch.Ki = C.Blocks.PID_pitch.Ki;
PID_pitch.Kd = C.Blocks.PID_pitch.Kd;
PID_pitch.Tf = C.Blocks.PID_pitch.Tf;

PID_yaw.Kp = C.Blocks.PID_yaw.Kp;
PID_yaw.Ki = C.Blocks.PID_yaw.Ki;
PID_yaw.Kd = C.Blocks.PID_yaw.Kd;
PID_yaw.Tf = C.Blocks.PID_yaw.Tf;

% Conecta los PIDs para format el controlador diagonal
K = connect(PID_roll, PID_pitch, PID_yaw,  {'\Delta\phi', '\Delta\theta', '\Delta\psi'}, {'\tau_\phi', '\tau_\theta', '\tau_\psi'});

% -----------------------------------------------------------------------
% Diagnosis del controlador
%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% Matrices de Transferencia L, S, T
%-------------------------------------------------------------------------

% Matriz de transferencia en lazo abierto L = GK
L = connect(G, PID_roll, PID_pitch, PID_yaw, {'\Delta\phi', '\Delta\theta', '\Delta\psi'}, {'\phi', '\theta', '\psi'});
L_tf = tf(L)
L_zp = zpk(L);
pole(L)
tzero(L)
% Matriz de sensibilidad S = (1 + GK)^-1
S = inv(eye(3)  + L_tf);
pole(S)
tzero(S)
% Matriz de sensibilidad complementaria T(s) = (1 + GK)^-1 GK
T = connect(G,C,{'\phi_r', '\theta_r', '\psi_r'},{'\phi', '\theta', '\psi'});
T_tf = minreal(zpk(T), 1e-2);
pole(T)
tzero(T)


% Calcula la ganancia en estado estacionario REF-->OUT
Tss = evalfr(T,0)  ;             % Steady State Gain Matrix SSGM

% ------------------------------------------------------------------------
% Diagrama de Nyquist Directo
% ------------------------------------------------------------------------
% Rango de frecuencias

wout = logspace(-1,5,256);
[re, im, wout] = nyquist(L, wout);

close all

f = figure(1);
f.Color = 'w';
f.Position(3:4) = [700, 600];
ax=gca;
ax.FontSize = 15;

subplot(2,2,1)
plot(reshape(re(1,1,:), [length(wout),1]), ...
    reshape(im(1,1,:), [length(wout),1]), 'LineWidth',1,'Color','b')
hold on
plot(reshape(re(1,1,:), [length(wout),1]), ...
    -reshape(im(1,1,:), [length(wout),1]), 'LineWidth',1,'Color','b')
hold on
plot(-1, 0, 'kx', 'Markersize', 10)
hold on
plot([-200, 200], [0, 0], 'k:')
hold on
plot([0, 0], [-200, 200], 'k:')
hold on
xlabel('Re[L_{11}(j\omega)]',  'FontSize', 15);
ylabel('Im[L_{11}(j\omega)]',  'FontSize', 15);
xlim([-10, 10]);
ylim([-10, 10]);
grid on
subplot(2,2,2)
plot(reshape(re(1,2,:), [length(wout),1]), ...
    reshape(im(1,2,:), [length(wout),1]), 'LineWidth',1,'Color','b')
hold on
plot(reshape(re(1,2,:), [length(wout),1]), ...
    -reshape(im(1,2,:), [length(wout),1]), 'LineWidth',1,'Color','b')
hold on
plot(-1, 0, 'kx', 'Markersize', 10)
hold on
plot([-200, 200], [0, 0], 'k:')
hold on
plot([0, 0], [-200, 200], 'k:')
hold on
xlim([-10, 10]);
ylim([-10, 10]);
grid on
xlabel('Re[L_{12}(j\omega)]',  'FontSize', 15);
ylabel('Im[L_{12}(j\omega)]',  'FontSize', 15);

subplot(2,2,3)
plot(reshape(re(2,1,:), [length(wout),1]), ...
    reshape(im(2,1,:), [length(wout),1]), 'LineWidth',1,'Color','b')
hold on
plot(reshape(re(2,1,:), [length(wout),1]), ...
    -reshape(im(2,1,:), [length(wout),1]), 'LineWidth',1,'Color','b')
hold on
plot(-1, 0, 'kx', 'Markersize', 10)
hold on
plot([-200, 200], [0, 0], 'k:')
hold on
plot([0, 0], [-200, 200], 'k:')
hold on
xlim([-10, 10]);
ylim([-10, 10]);
grid on
xlabel('Re[L_{21}(j\omega)]',  'FontSize', 15);
ylabel('Im[L_{21}(j\omega)]',  'FontSize', 15);
subplot(2,2,4)
plot(reshape(re(2,2,:), [length(wout),1]), ...
    reshape(im(2,2,:), [length(wout),1]), 'LineWidth',1,'Color','b')
hold on
plot(reshape(re(2,2,:), [length(wout),1]), ...
    -reshape(im(2,2,:), [length(wout),1]), 'LineWidth',1,'Color','b')
hold on
plot(-1, 0, 'kx', 'Markersize', 10)
hold on
plot([-200, 200], [0, 0], 'k:')
hold on
plot([0, 0], [-200, 200], 'k:')
hold on
xlim([-10, 10]);
ylim([-10, 10]);
grid on
xlabel('Re[L_{22}(j\omega)]',  'FontSize', 15);
ylabel('Im[L_{22}(j\omega)]',  'FontSize', 15);


% ------------------------------------------------------------------------
% Diagrama de Nyquist Generalizado
% ------------------------------------------------------------------------


wout1   = logspace(-1,5,256);
wout2   = -logspace(5,-1, 256);

wout = [wout2 wout1];

RE      = zeros(length(wout), 1);
IM      = zeros(length(wout), 1);

for i = 1:length(wout)
    Lw = evalfr(L, 1i*wout(i));
    determinant = det(eye(3) + Lw);
    RE(i) = real(determinant);
    IM(i) = imag(determinant);
end

f = figure(2);
f.Color = 'w';
ax=gca;
ax.FontSize = 14;
title('Diagrama de Nyquist Generalizado' ,'FontSize', 20)
hold on 
plot(RE -1, IM, 'LineWidth',1, 'Color', 'b')
hold on
plot(-1, 0, 'kx', 'Markersize', 10)
hold on
plot([-200, 200], [0, 0], 'k:')
hold on
plot([0, 0], [-200, 200], 'k:')
hold on
xlabel('Re[det(I + L(j\omega))] - 1',  'FontSize', 20);
ylabel('Im[det(I + L(j\omega))]',  'FontSize', 20);
xlim([-40, 40]);
ylim([-40, 40]);
grid on

% ------------------------------------------------------------------------
% Step Response
% -------------------------------------------------------------------------
f3 = figure(3);
f3.Color = 'w';
step(T)
grid on
h1=findall(f3);
% To find the line object handle from the list of graphic object handles
hline=findobj(h1,'Type','line','Tag','Curves'); 
% To set the line width
hline(1).LineWidth=1;
hline(2).LineWidth=1;
hline(3).LineWidth=1;
hline(4).LineWidth=1;
% -----------------------------------------------------------------------
% Diagrama de Bode
% ------------------------------------------------------------------------
f4 = figure(4);
bode(T)
f4.Color = 'w';
grid on
f4.Position(3:4)=[600,600];

figure('Position',[100,100,520,1000])
loopview(G,C,Info)

save('drone_multiloop.mat', 'L', 'L_zp', 'T_tf', 'T', 'K')


