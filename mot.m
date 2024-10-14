clear;
clc;
close all;

%% Paramètres du système
dt = 0.1;                    % Pas de temps
T = 150;                     % Nombre d'étapes de temps
nbVarX = 2;                  % Nombre de variables d'état (x, y)
nbVarU = 2;                  % Nombre de variables de contrôle (action)
Q = eye(nbVarX);             % Matrice de coût pour les états
R = 0.01 * eye(nbVarU);      % Matrice de coût pour les actions (pénalisation des grandes actions)

% Initialisation des demi-axes pour les ellipses
a1 = 5;  % Demi-grand axe de la première ellipse
b1 = 3;  % Demi-petit axe de la première ellipse
a2 = 4;  % Demi-grand axe de la seconde ellipse
b2 = 2;  % Demi-petit axe de la seconde ellipse

% Point de départ sur la première ellipse
theta_start = pi / 2; % Angle de départ pour se positionner sur l'ellipse
x0 = [a1 * cos(theta_start); b1 * sin(theta_start)]; % Position initiale sur l'ellipse

xGoal = [0; 0];              % Objectif à atteindre (au centre de la seconde ellipse)
u = zeros(nbVarU, T-1);      % Initialisation des actions

%% Modèle dynamique simple (modèle linéaire)
A = eye(nbVarX);             % Matrice d'état (simple déplacement)
B = dt * eye(nbVarU);        % Matrice de contrôle (effet des actions sur les états)

%% Initialisation des variables
x = zeros(nbVarX, T);        % Trajectoire d'état
x(:,1) = x0;                 % État initial
x_traj = zeros(nbVarX, T);   % Trajectoire désirée

%% Générer deux trajectoires elliptiques
% Générer la première ellipse
for t = 1:T/2
    theta = 2 * pi * (t / (T/2)); % angle pour la première ellipse
    x_traj(1,t) = a1 * cos(theta); % Position X
    x_traj(2,t) = b1 * sin(theta); % Position Y
end

% Générer la seconde ellipse
for t = (T/2)+1:T
    theta = 2 * pi * ((t - (T/2)) / (T/2)); % angle pour la seconde ellipse
    x_traj(1,t) = a2 * cos(theta) + (a1 + a2 + 1); % Décalage à droite pour séparer les ellipses
    x_traj(2,t) = b2 * sin(theta); % Position Y
end

%% Algorithme iLQR
for iter = 1:100  % Limiter à 100 itérations
    % Simulation avant la rétropropagation
    for t = 2:T
        x(:,t) = A*x(:,t-1) + B*u(:,t-1); % Simulation du système
    end

    % Calcul du coût initial
    cost = 0;
    for t = 1:T-1
        cost = cost + (x(:,t) - x_traj(:,t))' * Q * (x(:,t) - x_traj(:,t)) + u(:,t)' * R * u(:,t);
    end
    cost = cost + (x(:,T) - x_traj(:,T))' * Q * (x(:,T) - x_traj(:,T)); % Coût final

    % Affichage du coût pour chaque itération
    disp(['Itération ' num2str(iter) ', coût = ' num2str(cost)]);

    % Rétropropagation (Backward Pass)
    Vx = Q * (x(:,T) - x_traj(:,T));    % Gradient du coût terminal
    Vxx = Q;                            % Hessienne du coût terminal

    K = zeros(nbVarU, nbVarX, T-1);     % Gains de feedback
    d = zeros(nbVarU, T-1);             % Termes feedforward

    for t = T-1:-1:1
        % Calcul des matrices Qxx, Qux, Quu
        Qx = Q * (x(:,t) - x_traj(:,t)) + A' * Vx;
        Qu = R * u(:,t) + B' * Vx;
        Qxx = Q + A' * Vxx * A;
        Qux = B' * Vxx * A;
        Quu = R + B' * Vxx * B;

        % Calcul des gains de feedback et feedforward
        invQuu = inv(Quu);
        K(:,:,t) = -invQuu * Qux;
        d(:,t) = -invQuu * Qu;

        % Mise à jour des valeurs pour l'étape suivante
        Vx = Qx + K(:,:,t)' * Quu * d(:,t) + K(:,:,t)' * Qu + Qux' * d(:,t);
        Vxx = Qxx + K(:,:,t)' * Quu * K(:,:,t) + K(:,:,t)' * Qux + Qux' * K(:,:,t);
    end

    % Simulation avec la mise à jour des actions (Forward Pass)
    alpha = 1.0; % Paramètre de ligne (line search)
    for t = 1:T-1
        u(:,t) = u(:,t) + alpha * d(:,t) + K(:,:,t) * (x(:,t) - x_traj(:,t));
        x(:,t+1) = A * x(:,t) + B * u(:,t);  % Mise à jour de l'état
    end

    % Vérification de la convergence (petit changement du coût)
    if cost < 1e-6
        break;
    end
end

%% Affichage des résultats
figure;
plot(x(1,:), x(2,:), 'b-', 'LineWidth', 2); hold on;
plot(x_traj(1,:), x_traj(2,:), 'r--', 'LineWidth', 1.5);
plot(xGoal(1), xGoal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
title('Trajectoire optimisée avec iLQR (Ellipse à Ellipse)');
xlabel('Position X');
ylabel('Position Y');
legend('Trajectoire optimisée', 'Trajectoire désirée (Ellipse à Ellipse)', 'Objectif');
grid on;
axis equal;
