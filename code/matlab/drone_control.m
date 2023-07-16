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
[xs,us,ys,dx] = trim('attitude_drone_2020b', x0, u0, [], [], iu, [])


% Linealizacion de la dinamica en torno al punto de equilibrio x = xs
[A,B,C,D] = linmod('attitude_drone_2020b', xs, us)

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
G.OutputName    = {'\phi', '\theta', '\psi^t'};      % Renombra las salidas de la FT

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
PID_yaw.InputName    = '\Delta\psi^t';                             % Entrada PID
PID_yaw.OutputName   = '\tau_\psi';                        % Salida PID


% Error = Referencia - salida
sum1 = sumblk('\Delta\phi = \phi_r - \phi',1);
sum2 = sumblk('\Delta\theta = \theta_r - \theta',1);
sum3 = sumblk('\Delta\psi^t = \psi^t_r - \psi^t',1);
% Define el subsistema a ajustar (Controlador C0) con los bloques ajustables
% correspondientes
C0 = connect(PID_roll, PID_pitch, PID_yaw, sum1, sum2, sum3,...
    {'\phi_r', '\theta_r', '\psi^t_r', '\phi', '\theta', '\psi^t'}, {'\tau_\phi', '\tau_\theta', '\tau_\psi'});
 

% Define el rango de frecuencias
wc = [1e1, 1e3];

% Define las opciones del algoritmo de ajuste
% options = looptuneOptions('MinDecay', 0.1);
options = looptuneOptions();
% options = looptuneOptions('RandomStart',20);
% Calcula el controlador C ajustado
[G, C, gam, Info] = looptune(G, C0, wc, options);


% Imprime por pantalla las propiedades del controlador ajustado


%  Asigna los valores de los PIDS ajustados a los bloques iniciales

C.Blocks.PID_roll.Kp = C.Blocks.PID_pitch.Kp;
C.Blocks.PID_roll.Ki = C.Blocks.PID_pitch.Ki;
C.Blocks.PID_roll.Kd = C.Blocks.PID_pitch.Kd;
C.Blocks.PID_roll.Tf = C.Blocks.PID_pitch.Tf;


C.Blocks.PID_yaw.Kp.Value = C.Blocks.PID_yaw.Kp.Value/20;
C.Blocks.PID_yaw.Ki.Value = C.Blocks.PID_yaw.Ki.Value/20;
C.Blocks.PID_yaw.Kd.Value = C.Blocks.PID_yaw.Kd.Value/20;


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

showTunable(C)



% Conecta los PIDs para format el controlador diagonal
K = connect(PID_roll, PID_pitch, PID_yaw,  {'\Delta\phi', '\Delta\theta', '\Delta\psi^t'}, {'\tau_\phi', '\tau_\theta', '\tau_\psi'});

% -----------------------------------------------------------------------
% Diagnosis del controlador
%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% Matrices de Transferencia L, S, T
%-------------------------------------------------------------------------

% Matriz de transferencia en lazo abierto L = GK
L = connect(G, PID_roll, PID_pitch, PID_yaw, {'\Delta\phi', '\Delta\theta', '\Delta\psi^t'}, {'\phi', '\theta', '\psi^t'});
L_tf = minreal(tf(L), 1e-2);
L_zp = minreal(zpk(L), 1e-2);
pole(L_tf)
tzero(L_tf)
% Matriz de sensibilidad S = (1 + GK)^-1
S = inv(eye(3)  + L_tf);
pole(S)
tzero(S)
% Matriz de sensibilidad complementaria T(s) = (1 + GK)^-1 GK
T = connect(G,C,{'\phi_r', '\theta_r', '\psi^t_r'},{'\phi', '\theta', '\psi^t'});
T_tf = minreal(zpk(T), 1e-2);
pole(T_tf)
tzero(T_tf)


% Calcula la ganancia en estado estacionario REF-->OUT
Tss = evalfr(T,0)  ;             % Steady State Gain Matrix SSGM



%% ------------------------------------------------------------------------
% Diagrama de Nyquist Generalizado
% ------------------------------------------------------------------------
close all

wout1   = logspace(-1,5,256);
wout2   = -logspace(5,-1, 256);

wout = [wout2 wout1];

wout= wout1;

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
xlim([-15, 5]);
ylim([-10, 10]);

% xlim([-50, 150]);
% ylim([-100, 100]);
grid on

% ------------------------------------------------------------------------
% Step Response
% -------------------------------------------------------------------------


f3 = figure(3);
f3.Color = 'w';
subplot(1,3,1)
step(T(1,1))
grid on
subplot(1,3,2)
step(T(2,2))
grid on
subplot(1,3,3)
step(T(3,3))
grid on
h1=findall(f3);
% To find the line object handle from the list of graphic object handles
hline=findobj(h1,'Type','line','Tag','Curves'); 
% To set the line width
hline(1).LineWidth=1;
hline(2).LineWidth=1;
hline(3).LineWidth=1;
hline(4).LineWidth=1;

f3.Position(3:4)=[1200,400];

% -----------------------------------------------------------------------
% Diagrama de Bode
% ------------------------------------------------------------------------
f4 = figure(4);
subplot(1,3,1)
bode(T(1,1))
grid on
subplot(1,3,2)
bode(T(2,2))
grid on
subplot(1,3,3)
bode(T(3,3))
f4.Color = 'w';
grid on
f4.Position(3:4)=[1200,600];
h1=findall(f4);
% To find the line object handle from the list of graphic object handles


save('drone_multiloop.mat', 'L', 'L_zp', 'T_tf', 'T', 'K')


