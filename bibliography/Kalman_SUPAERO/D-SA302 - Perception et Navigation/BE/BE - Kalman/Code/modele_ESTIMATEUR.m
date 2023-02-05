function [paramEST,initEST] = modele_ESTIMATEUR(nom,paramENG,paramODO,paramIMU,paramGPS)

% [paramEST,initEST] = modele_ESTIMATEUR(nom,paramENG,paramODO,paramIMU,paramGPS);
%--------------------------------------------------------------------------
% Mod?le de l'ESTIMATEUR
%--------------------------------------------------------------------------
%   nom :       Nom du mod?le ? utiliser
%   paramENG :  param?tres de l'engin
%   paramODO :  param?tres des odom?tres
%   paramIMU :  param?tres de l'IMU
%   paramGPS :  param?tres du GPS
%
%	paramEST :	structure contenant les param?tres de l'estimateur
%	initEST :   structure contenant les initialisations de l'estimateur
%--------------------------------------------------------------------------

if nargin==0
    nom = 'model_EST_1';
end;

%--------------------------------------------------------------------------
% R?cup?ration des param?tres des ?l?ments de mod?lisation
%--------------------------------------------------------------------------

% Param?tres g?om?triques du v?hicule
H = paramENG.H;      % Demi-longueur de l'essieu arri?re
L = paramENG.L;      % Distance inter-essieux

% Param?tres du contr?le du v?hicule
tauVIT = paramENG.tauVIT;   % Constante de temps sur la vitesse
tauPHI = paramENG.tauPHI;	% Constante de temps sur la braquage

% Param?tres des odom?tres
dt_ODO      = paramODO.dt;          % Pas d'?chantillonnage [s]
sig_vitODO	= paramODO.sig_vit;     % Ecart-type du bruit [m/s]

% Param?tres du gyrom?tre
dt_IMU      = paramIMU.dt;      % Pas d'?chantillonnage [s]
sig_omeIMU  = paramIMU.sig_ome;	% Ecart-type du bruit [?/s]

% Param?tres du GPS
dt_GPS      = paramGPS.dt;      % Pas d'?chantillonnage [s]
sig_posGPS  = paramGPS.sig_pos;	% Ecart-type du bruit [m]

% Enregistrement
paramEST.dt_ODO     = dt_ODO;
paramEST.dt_IMU     = dt_IMU;
paramEST.dt_GPS     = dt_GPS;


switch nom
    
    case 'model_EST_1'
        %------------------------------------------------------------------
        % X = [pos;att]
        % U = [V^*,omeGPS]
        % Z = [posGPS]
        %------------------------------------------------------------------
        %warning(['Le mod?le ',nom,' doit ?tre adapt? aux donn?es!!!']);
                
        % Param?tres de l'ESTIMATEUR
        paramEST.dt	= 0.01;     % Pas d'?chantillonnage [s]
        paramEST.Q = eye(3,3);
        paramEST.U = diag([1,paramIMU.sig_ome]);
        paramEST.R = eye(2,2);       
        
        % Etat initial
        initEST.POS	= [0 ; 0];	% Position du v?hicule
        initEST.ATT	= pi;        % Direction du v?hicule [?]

        % Enregistrement
        initEST.x	= [initEST.POS;initEST.ATT*pi/180];
        initEST.nx	= length(initEST.x);
                
        % Covariance initiale
        P = 100*eye(initEST.nx,initEST.nx);
        
        % Enregistrement
        initEST.P = P;
        
    case 'model_EST_1.2'
        %------------------------------------------------------------------
        % X = [pos;att;biais gyro]
        % U = [V^*,omeGPS]
        % Z = [posGPS]
        %------------------------------------------------------------------
        %warning(['Le mod?le ',nom,' doit ?tre adapt? aux donn?es!!!']);
                
        % Param?tres de l'ESTIMATEUR
        paramEST.dt	= 0.01;     % Pas d'?chantillonnage [s]
        paramEST.U = diag([1,paramIMU.sig_ome^2]);
        paramEST.R = diag([paramGPS.sig_pos paramGPS.sig_pos].^2);
        paramEST.Q = diag([0.1 0.1 0.1 0]);
        
        % Etat initial
        initEST.POS	= [0 ; 0];	% Position du v?hicule
        initEST.ATT	= 0;        % Direction du v?hicule [?]
        initEST.BIAIS = 0;      % Biais du gyrom?tre
        
        % Enregistrement
        initEST.x	= [initEST.POS; initEST.ATT*pi/180; initEST.BIAIS];
        initEST.nx	= length(initEST.x);
                
        % Covariance initiale
        P = diag([10/3, 10/3, pi/3, 5*pi/180].^2);
        
        % Enregistrement
        initEST.P = P;
            
    case 'model_EST_2'
        %------------------------------------------------------------------
        % X = [pos;att;VIT;PHI]
        % U = [V^*,PHI^*,omeGPS]
        % Z = [posGPS]
        %------------------------------------------------------------------
        warning(['Le mod?le ',nom,' doit ?tre adapt? aux donn?es!!!']);
                
        % Param?tres de l'ESTIMATEUR
        paramEST.dt	= 0.01;     % Pas d'?chantillonnage [s]
        paramEST.R = diag([paramGPS.sig_pos paramGPS.sig_pos].^2);
        
        
        % Etat initial
        initEST.POS	= [0 ; 0];	% Position du v?hicule
        initEST.ATT	= 0;        % Direction du v?hicule [?]
        initEST.VIT	= 0;        % Vitesse du v?hicule
        initEST.PHI	= 0;        % Angle de braquage des roues [?]

        % Enregistrement
        initEST.x	= [initEST.POS;initEST.ATT*pi/180;initEST.VIT;initEST.PHI];
        initEST.nx	= length(initEST.x);
                
        % Covariance initiale
        P = 100*eye(initEST.nx,initEST.nx);
        
        % Enregistrement
        initEST.P = P;
    
    otherwise
        
        error('Modele ESTIMATEUR inconnu');
        
end;

