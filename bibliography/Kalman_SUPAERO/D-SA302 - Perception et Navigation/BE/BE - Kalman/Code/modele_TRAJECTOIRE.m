function [IN_consVIT,IN_consPHI] = modele_TRAJECTOIRE(nom)

% [IN_consVIT,IN_consPHI] = modele_TRAJECTOIRE(nom);
%--------------------------------------------------------------------------
% Trajectoire de l'ENGIN
%--------------------------------------------------------------------------
%   nom :   Nom du modèle à utiliser
%
%	IN_consVIT :	structure contenant le signal de consigne en vitesse
%	IN_consPHI :	structure contenant le signal de consigne en braquage
%--------------------------------------------------------------------------


if nargin==0
    nom = 'model_TRA_1';
end;

switch nom
    
    case 'model_TRA_1'
        %------------------------------------------------------------------
        % Trajectoire pour véhicule lent
        %------------------------------------------------------------------

        % Séquence des consignes de vitesse: date[s], vitesse[m/s]
        consVIT = [];
        consVIT = [consVIT;[   0   0  ]];
        consVIT = [consVIT;[   5   5  ]];
        consVIT = [consVIT;[  50   3  ]];
        consVIT = [consVIT;[  80  10  ]];
        consVIT = [consVIT;[ 120   8  ]];
        consVIT = [consVIT;[ 160   0  ]];
        
        % Séquence des consignes de braquage: date[s], braquage[°]
        consPHI = [];
        consPHI = [consPHI;[   0   0  ]];
        consPHI = [consPHI;[  10   2  ]];
        consPHI = [consPHI;[  20   0  ]];
        consPHI = [consPHI;[  25  -1  ]];
        consPHI = [consPHI;[  30   0  ]];
        consPHI = [consPHI;[  40   5  ]];
        consPHI = [consPHI;[  50  -0.5  ]];
        consPHI = [consPHI;[  60   2  ]];
        consPHI = [consPHI;[  80  -5  ]];
        consPHI = [consPHI;[ 120   2  ]];
        consPHI = [consPHI;[ 130   0  ]];
        consPHI = [consPHI;[ 140  -1  ]];
        consPHI = [consPHI;[ 155   0  ]];
        
        % Enregistrement
        IN_consVIT.time                 = consVIT(:,1);
        IN_consVIT.signals.values       = consVIT(:,2);
        IN_consVIT.signals.dimensions	= 1;
        
        % Enregistrement
        IN_consPHI.time                 = consPHI(:,1);
        IN_consPHI.signals.values       = consPHI(:,2);
        IN_consPHI.signals.dimensions	= 1;
        
    otherwise
        error('Modele TRAJECTOIRE inconnu');
        
end;
