function [paramENG,initENG] = modele_VEHICULE(nom)

% [paramENG,initENG] = modele_VEHICULE(nom);
%--------------------------------------------------------------------------
% Mod�le de SIMULATION du VEHICULE
%--------------------------------------------------------------------------
%   nom :   Nom du mod�le � utiliser
%
%	paramENG :	structure contenant les param�tres
%	initENG :   structure contenant les initialisations
%--------------------------------------------------------------------------

if nargin==0
    nom = 'model_ENG_1';
end;

switch nom
    
    case 'model_ENG_1'
        %------------------------------------------------------------------
        % Mod�le d'un v�hicule lent
        %------------------------------------------------------------------
        
        % Param�tres g�om�triques du v�hicule
        paramENG.H	= 0.8;      % Demi-longueur de l'essieu arri�re
        paramENG.L	= 2.0;      % Distance inter-essieux
        
        % Param�tres du contr�le de vitesse
        paramENG.tauVIT     =  3;   % Constante de temps sur la vitesse
        paramENG.maxVIT     = 10;	% Vitesse maximale [m/s]
        paramENG.minVIT     =  0;	% Vitesse minimale [m/s]
        paramENG.maxVITP	=  1;	% Acc�l�ration maximale [m/s^2]
        
        % Param�tres du contr�le de braquage des roues
        paramENG.tauPHI     =  1;	% Constante de temps sur la braquage
        paramENG.maxPHI     =  5;	% Braquage maximal [�]
        paramENG.maxPHIP	= 10;	% Vitesse de braquage maximale [�/s]
        
        % Etat initial
        initENG.POS	= [0 ; 0];	% Position du v�hicule
        initENG.ATT	= 0;        % Direction du v�hicule [�]
        initENG.VIT	= 0;        % Vitesse du v�hicule
        initENG.PHI	= 0;        % Angle de braquage des roues [�]
        
        % Enregistrement
        initENG.x	= [initENG.POS;initENG.ATT*pi/180];
        initENG.nx	= length(initENG.x);
        
    otherwise
        error('Modele VEHICULE inconnu');
        
end;

