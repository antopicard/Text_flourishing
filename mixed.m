clear;
clc;
close all;

%% Paramètres du système
dt = 0.1;                     % Pas de temps
T = 150;                      % Nombre d'étapes de temps
nbVarX = 2;                   % Nombre de variables d'état (x, y)
nbVarU = 2;                   % Nombre de variables de contrôle (action)
Q = eye(nbVarX);              % Matrice de coût pour les états
R = 0.01 * eye(nbVarU);       % Matrice de coût pour les actions (pénalisation des grandes actions)
G = 6.67430e-11;              % Constante gravitationnelle

% Initialisation des demi-axes pour les ellipses
a1 = 5;   % Demi-grand axe de la première ellipse
b1 = 3;   % Demi-petit axe de la première ellipse
a2 = 4;   % Demi-grand axe de la seconde ellipse
b2 = 2;   % Demi-petit axe de la seconde ellipse

% Seuil de distance pour déclencher l'iLQR
threshold = 1; % Distance seuil entre la planète et l'ellipse

% Masse des foyers et de la planète
m1 = 5e10;   % Masse du premier foyer
m2 = 5e10;   % Masse du second foyer
mass_planet = 1000; % Masse de la planète

% Point de départ sur la première ellipse
theta_start = pi / 2; % Angle de départ
x0 = [a1 * cos(theta_start); b1 * sin(theta_start)]; % Position initiale

xGoal = [0; 0];               % Objectif à atteindre (centre de la seconde ellipse)
u = zeros(nbVarU, T-1);       % Initialisation des actions

%% Modèle dynamique
A = eye(nbVarX);              % Matrice d'état (simple déplacement)
B = dt * eye(nbVarU);         % Matrice de contrôle (effet des actions sur les états)

%% Initialisation des variables
x = zeros(nbVarX, T);         % Trajectoire d'état
x(:,1) = x0;                  % État initial
x_traj = zeros(nbVarX, T);    % Trajectoire désirée

%% Générer deux trajectoires elliptiques
% Première ellipse
for t = 1:T/2
    theta = 2 * pi * (t / (T/2));
    x_traj(1,t) = a1 * cos(theta);  % Position X
    x_traj(2,t) = b1 * sin(theta);  % Position Y
end

% Seconde ellipse
for t = (T/2)+1:T
    theta = 2 * pi * ((t - (T/2)) / (T/2));
    x_traj(1,t) = a2 * cos(theta) + (a1 + a2 + 1); % Décalage pour séparer les ellipses
    x_traj(2,t) = b2 * sin(theta);
end

%% Simulation avant l'utilisation d'iLQR
for t = 2:T
    % Simuler sous l'influence de la gravité avant d'atteindre le seuil
    if norm(x(:,t-1) - x_traj(:,t-1)) > threshold
        % Calcul de la force gravitationnelle
        r1 = norm(x(:,t-1) - [0; 0]);   % Distance au premier foyer
        r2 = norm(x(:,t-1) - [a1 + a2 + 1; 0]); % Distance au second foyer

        F1 = -G * m1 * mass_planet / r1^2 * (x(:,t-1) / r1); % Force du premier foyer
        F2 = -G * m2 * mass_planet / r2^2 * ((x(:,t-1) - [a1 + a2 + 1; 0]) / r2); % Force du second foyer

        % Mise à jour de l'état selon la gravité
        x(:,t) = x(:,t-1) + dt * (F1 + F2) / mass_planet;

    else
        % Déclenchement de l'iLQR si la planète atteint le seuil
        
        % Algorithme iLQR commence ici
        % Simulation avec iLQR
        for iter = 1:100  % Limite à 100 itérations
            for t_inner = 2:T
                x(:,t_inner) = A*x(:,t_inner-1) + B*u(:,t_inner-1); % Simulation du système
            end
            
            % Calcul du coût initial
            cost = 0;
            for t_inner = 1:T-1
                cost = cost + (x(:,t_inner) - x_traj(:,t_inner))' * Q * (x(:,t_inner) - x_traj(:,t_inner)) + u(:,t_inner)' * R * u(:,t_inner);
            end
            cost = cost + (x(:,T) - x_traj(:,T))' * Q * (x(:,T) - x_traj(:,T)); % Coût final
            
            % Affichage du coût
            disp(['Itération ' num2str(iter) ', coût = ' num2str(cost)]);
            
            % Rétropropagation (Backward Pass)
            Vx = Q * (x(:,T) - x_traj(:,T));    % Gradient du coût terminal
            Vxx = Q;                            % Hessienne du coût terminal
            
            K = zeros(nbVarU, nbVarX, T-1);     % Gains de feedback
            d = zeros(nbVarU, T-1);             % Termes feedforward
            
            for t_inner = T-1:-1:1
                Qx = Q * (x(:,t_inner) - x_traj(:,t_inner)) + A' * Vx;
                Qu = R * u(:,t_inner) + B' * Vx;
                Qxx = Q + A' * Vxx * A;
                Qux = B' * Vxx * A;
                Quu = R + B' * Vxx * B;
                
                invQuu = inv(Quu);
                K(:,:,t_inner) = -invQuu * Qux;
                d(:,t_inner) = -invQuu * Qu;
                
                Vx = Qx + K(:,:,t_inner)' * Quu * d(:,t_inner) + K(:,:,t_inner)' * Qu + Qux' * d(:,t_inner);
                Vxx = Qxx + K(:,:,t_inner)' * Quu * K(:,:,t_inner) + K(:,:,t_inner)' * Qux + Qux' * K(:,:,t_inner);
            end
            
            % Mise à jour des actions
            alpha = 1.0;
            for t_inner = 1:T-1
                u(:,t_inner) = u(:,t_inner) + alpha * d(:,t_inner) + K(:,:,t_inner) * (x(:,t_inner) - x_traj(:,t_inner));
                x(:,t_inner+1) = A * x(:,t_inner) + B * u(:,t_inner); % Mise à jour de l'état
            end
            
            % Vérification de la convergence
            if cost < 1e-6
                break;
            end
        end
        break;
    end
end

%% Affichage des résultats
figure;
plot(x(1,:), x(2,:), 'b-', 'LineWidth', 2); hold on;
plot(x_traj(1,:), x_traj(2,:), 'r--', 'LineWidth', 1.5);
plot(xGoal(1), xGoal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('Position X');
ylabel('Position Y');
legend('Trajectoire optimisée', 'Trajectoire désirée (Ellipse à Ellipse)', 'Objectif');
grid on;
axis equal;
