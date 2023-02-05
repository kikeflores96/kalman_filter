% Données bathymétriques provenant de
% http://www.gebco.net/data_and_products/gridded_bathymetry_data/
% taille de la grille : 30s d'arc, soit un demi-mille marin (926 m) en
% longitude

%% Section 1
% Chargement et affichage des données altimétriques
close all;
clear all;
Z=load('map.asc');
[I J]=size(Z);

%% section 2
% Initialisation des paramètres
N = 50;                % Number of time steps.
t = 1:1:N;              % Time.
v0=1;                   % speed along x1
% x = zeros(N,2);         % Hidden states.
% y = zeros(N,1);         % Observations.
x(1,1) = 30;            % Initial state.
x(1,2) = 170;           % Initial state.
Rreal = 10^2;           % Measurement noise real variance.
R=Rreal;             % Measurement noise used for estimation
Qreal = [0.1 0;0 10];   % Process noise real variance.
Q = Qreal;                 % Process noise variance used for estimation
initVar = [0 0;0 0]; % Initial variance of the states.
numSamples = 100 ;
numSim = 50; % Number of simulation

listfactor = [0.9 1 1.2 1.5];
listEQM = zeros(length(listfactor),N);
for tsim=1:numSim
    
%% Section 3
% Génération de la trajectoire et des mesures
for t=2:N
    x(t,:)=x(t-1,:)+[v0 0]+randn(1,2)*sqrt(Qreal); % trajectory (process)
end;
%filtrage de la trajectoire, pour "adoucissement"
alpha=0.01;
b=1-alpha;
a=[1 -alpha];
x=filter(b,a,x);
%mesures
v = sqrt(Rreal)*randn(N,1); % measurement noise
for t=1:N,
    y(t,1) = interp2(Z,x(t,1),x(t,2)) + v(t,1); % measurement
end;

for factor =1:length(listfactor)
    
    R = listfactor(factor)*Rreal;
%% section 4
% Particules initiales (prior)
xxu=zeros(N,2,numSamples);
xu=sqrt(initVar)*randn(2,numSamples);
q=ones(1,numSamples);
xu(1,:)=xu(1,:)+x(1,1);
xu(2,:)=xu(2,:)+x(1,2);


%% Section 5
% Update et prédiction
for t=1:N-1
    %Predict
    %from the set of particles xu generate a new set xu
    
    for k=1:numSamples
        xu(:, k)=xu(:, k)+[v0 0]'+(randn(1,2)*sqrt(Q))';
    end;
    
    
    %Importance wheights
    m=[];
    for k=1:numSamples
        m(k)=interp2(Z,xu(1,k),xu(2,k));%mesures prédites pour chaque particules
    end;
    %from the set of weights q compute the new set of weights q
    q=q.*exp(-1/(2*R)*(y(t)*ones(1,numSamples)-m).^2); %REMPLACER !
    [ii jj]=find(xu(1,:)>J | xu(1,:)<1 | xu(2,:)>I | xu(2,:)<1 );
    q(jj)=0; %Elimine les éventuelles particules "hors du cadre"
    q=q./sum(q);
    
    %Resampling
    Neff=1/sum(q.^2);
    if Neff<0.75*numSamples
        %Resamplpling
        method='multinomial';
        switch method
            case 'none'
                xur=xu;
                q=q;
            case 'multinomial'              
                xur=xu(:,resample(q));%REMPLACER !!
                q=ones(1,numSamples)/numSamples;
        xu=xur;
    end
    end
    %Stockage dans xxu
    xxu(t,:,:)=xu;
    %Ellipsoid
    X0=mean(xu,2);
    EQM = (X0-x(t,:)')'*(X0-x(t,:)');
    listEQM(factor,t)= listEQM(factor,t) + sqrt(EQM);
end
end
end

listEQM = listEQM/numSim;
hold on
set(gca,'fontsize',20)
for factor =1:length(listfactor)
    plot(1:N-1,listEQM(factor,1:N-1),'LineWidth',6)
end
xlabel('Temps de simulation')
ylabel('Erreur moyenne de position des particules')
legend('0.9','1','1.2','1.5')
