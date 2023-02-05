%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D orientation estimation template
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CONFIGURATION and INITIALISATION
dt=0.01;
COM='COM5';
useSimulation = true;
useSimulation_visuals = useSimulation;
useGyro = 2; %0: none; 1: control; 2: full
%% CALIBRATION
% To re-run calibration, just clear MAG0 or run the calibration script
% separately:
if ~exist('MAG0', 'var')
    runcalibration;
end
%% affichage du parallélépipède
launch3D;

%% estimation attitude
qa = [0.0001]; %Bruit d'état orientation
qg = [1]; %Bruit d'état vitesse angulaire
ra = 1*ACCVAR(2:3); %Bruit de mesure accéléro
rg = 1*GYRVAR(1); %Bruit de mesure gyro

if useGyro == 2
    Q = diag([qa;qg]);
    R = diag([ra;rg]);
    X=[0 0]'; %Etat : rotation selon x (rad), vitesse autour de x   
else
    Q = diag([qa]);
    R = diag([ra]);
    X=[0]'; %Etat : rotation selon x (rad)
end

P = deg2rad(10)^2;

tp=0;
ii=0;
obs = [];
xtrue = [];
xhat = [];
if useSimulation
    imu411('open',COM, 'simimu_2Dmaneuver');
else
    imu411('open',COM);
end
while(1)
    ii=ii+1;
    % Read sensor
    [d, xt] = imu411('read'); %cette lecture est bloquante et cadence la boucle a 0.01s
    obs(:, end+1) = d;
    xtrue(:, end+1) = xt;
    % Rappel :
    % d(1)    = Time                (s)
    % d(2:4)  = Gyroscope     X,Y,Z (°/s)
    % d(5:7)  = Accelerometer X,Y,Z (g)
    
    t=d(1);
   
 
    % Predict
    if useGyro == 2
        F = [1 dt;0 1];
        X = F*X;
    elseif useGyro == 1
        F = [1];
        X = F*X + dt*d(2)*pi/180;
    else
        F = [1];
        X = F*X;
    end
    P = F*P*F'+Q;
    % Update
    if ~isnan(t)
        if useGyro == 2
            Y = [d(6) d(7) d(2)*pi/180]';
            Yhat = [sin(X(1)) cos(X(1)) X(2)]';
            H =   [cos(X(1)) -sin(X(1)) 0;0 0 1]';
        else
            Y = [d(6) d(7)]';
            Yhat = [sin(X(1)) cos(X(1))]';
            H =   [cos(X(1)) -sin(X(1))]';
        end
        S = H*P*H'+R;
        K = P*H'/S;
        X = X + K*(Y-Yhat);
        P = P - K*S*K';
    end
    
    % Update Visualisation:
    xhat(:, end+1) = [angle2quat(0, 0, X(1))'; zeros(3,1)];
    DCM_k = angle2dcm(0, 0, X(1), 'ZYX');
    update3D;
end