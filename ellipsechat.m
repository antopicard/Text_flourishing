% Constantes physiques
G = 6.67430e-11; % Constante gravitationnelle (m^3 kg^-1 s^-2)
m_foyer = 1e24; % Masse d'un foyer ajustée (kg) pour un effet plus elliptique
m_planet = 1000; % Masse de la planète (kg)

% Paramètres de l'orbite initiale
a1 = 1e7; % Demi-grand axe initial (m)
e1 = 0.7; % Excentricité augmentée pour rendre l'ellipse plus ovale
f1 = [0, 0]; % Position du premier foyer (m)

% Conditions initiales de la planète
theta0 = 0; % Angle initial (radians)
r0 = a1 * (1 - e1^2) / (1 + e1 * cos(theta0)); % Distance initiale (m)
v0 = sqrt(G * m_foyer * (2/r0 - 1/a1)); % Vitesse ajustée pour une orbite elliptique (m/s)
pos_planet = [r0, 0]; % Position initiale (x, y)
vel_planet = [0, v0]; % Vitesse initiale (vx, vy)

% Simulation du mouvement de la planète
dt = 1; % Pas de temps (s)
T_total = 100000; % Durée totale de la simulation (s)
n_foyers = 3; % Nombre de foyers

% Initialisation des positions pour la visualisation
positions = [];
positions = [positions; pos_planet];

% Liste des foyers
foyers = zeros(n_foyers, 2); % Stockage des positions des foyers
foyers(1, :) = f1; % Le premier foyer est initialisé

% Génération des autres foyers sans contrainte sur la distance
for i = 2:n_foyers
    % Positions aléatoires sans contrainte sur la distance entre foyers
    foyers(i, :) = foyers(i-1,:)+[2e7, -1e7];
    
end
foyers(3,:)= foyers(2,:) -[4e7,0];

% Index du foyer actuel
index_foyer = 1;

for t = 1:T_total
    % Position actuelle du foyer
    current_foyer = foyers(index_foyer, :);
    
    % Calcul de la force gravitationnelle vers le foyer actuel
    r_vec = pos_planet - current_foyer; % Vecteur de position par rapport au foyer
    r = norm(r_vec); % Distanc$e entre la planète et le foyer
    
    % Force gravitationnelle
    F_grav = -G * m_foyer * m_planet / r^2 * (r_vec / r);
    
    % Mise à jour de la vitesse et de la position de la planète
    accel_planet = F_grav / m_planet;
    vel_planet = vel_planet + accel_planet * dt;
    pos_planet = pos_planet + vel_planet * dt;

    % Mise à jour de la direction de la planète
    direction = vel_planet / norm(vel_planet);
    
    % Calcul du vecteur entre la planète et le prochain foyer
    if index_foyer < n_foyers
        next_foyer = foyers(index_foyer + 1, :);
        r_vec_next = next_foyer - pos_planet;

        % Calcul de la projection de r_vec_next sur la direction de la planète
        proj_r_next_on_dir = dot(r_vec_next, direction); % Projection scalaire

        % Calcul de la distance perpendiculaire
        if proj_r_next_on_dir>0
            perp_distance = sqrt(norm(r_vec_next)^2 - proj_r_next_on_dir^2);
        
            % Condition pour changer de foyer
            if perp_distance <= 5*r0
                index_foyer = index_foyer + 1;
            
                m_foyer = 1.5*m_foyer;
        
                if index_foyer > n_foyers
                 disp('Tous les foyers ont été visités');
                  break;
                end
            end
        end
       end
    
    
    % Sauvegarder la position pour la visualisation
    positions = [positions; pos_planet];
end

% Visualisation de la trajectoire
figure;
plot(positions(:, 1), positions(:, 2), 'DisplayName', 'Trajectoire de la planète','Color', 'r','LineWidth',3);
hold on;

% Tracer les foyers
for i = 1:n_foyers
    plot(foyers(i, 1), foyers(i, 2), 'o', 'MarkerSize', 10, 'DisplayName', ['Foyer ' num2str(i)]);
end

xlabel('Position X (m)');
ylabel('Position Y (m)');
title('Trajectoire de la planète autour de foyers avec orbites elliptiques');
legend('show');
axis equal;
grid on;
