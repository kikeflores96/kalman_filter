function [paramENG,initENG] = modele_VEHICULE(nom)

% [paramENG,initENG] = modele_VEHICULE(nom);
%--------------------------------------------------------------------------
% Modèle de SIMULATION du VEHICULE
%--------------------------------------------------------------------------
%   nom :   Nom du modèle à utiliser
%
%	paramENG :	structure contenant les paramètres
%	initENG :   structure contenant les initialisations
%--------------------------------------------------------------------------

if nargin==0
    nom = 'model_ENG_1';
end;

switch nom
    
    case 'model_ENG_1'
        %------------------------------------------------------------------
        % Modèle d'un véhicule lent
        %------------------------------------------------------------------
        
        % Paramètres géométriques du véhicule
        paramENG.H	= 0.8;      % Demi-longueur de l'essieu arrière
        paramENG.L	= 2.0;      % Distance inter-essieux
        
        % Paramètres du contrôle de vitesse
        paramENG.tauVIT     =  3;   % Constante de temps sur la vitesse
        paramENG.maxVIT     = 10;	% Vitesse maximale [m/s]
        paramENG.minVIT     =  0;	% Vitesse minimale [m/s]
        paramENG.maxVITP	=  1;	% Accélération maximale [m/s^2]
        
        % Paramètres du contrôle de braquage des roues
        paramENG.tauPHI     =  1;	% Constante de temps sur la braquage
        paramENG.maxPHI     =  5;	% Braquage maximal [°]
        paramENG.maxPHIP	= 10;	% Vitesse de braquage maximale [°/s]
        
        % Etat initial
        initENG.POS	= [0 ; 0];	% Position du véhicule
        initENG.ATT	= 0;        % Direction du véhicule [°]
        initENG.VIT	= 0;        % Vitesse du véhicule
        initENG.PHI	= 0;        % Angle de braquage des roues [°]
        
        % Enregistrement
        initENG.x	= [initENG.POS;initENG.ATT*pi/180];
        initENG.nx	= length(initENG.x);
        
    otherwise
        error('Modele VEHICULE inconnu');
        
end;

