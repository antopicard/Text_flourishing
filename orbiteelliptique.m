% Paramètres de la première ellipse
a1 = 1.5e11;       % Demi-grand axe (en mètres)
e1 = 0.6;          % Excentricité
b1 = a1 * sqrt(1 - e1^2);  % Demi-petit axe
c1 = a1 * e1;      % Distance focale (foyer partagé)

% Paramètres de la deuxième ellipse (en miroir)
a2 = 2e11;         % Demi-grand axe (en mètres, ajusté pour croiser)
e2 = 0.5;          % Excentricité
b2 = a2 * sqrt(1 - e2^2);  % Demi-petit axe
c2 = a2 * e2;      % Distance focale

% Nombre de points pour tracer les ellipses
nPoints = 1000;
theta = linspace(0, 2*pi, nPoints);
current_ellipse = 1;
% Calcul des positions pour la première ellipse
x1 = a1 * cos(theta) - c1; % Décalage de c1 pour centrer sur le foyer
y1 = b1 * sin(theta);

% Calcul des positions pour la deuxième ellipse (en miroir)
x2 = a2 * cos(theta) + c2; % Décalage de c2 de l'autre côté
y2 = b2 * sin(theta);

% Préparation du graphique
figure;
hold on;
axis equal;
xlabel('x (m)');
ylabel('y (m)');

% Tracer la première ellipse
plot(x1, y1, 'b--', 'LineWidth', 1.5); % Première ellipse en bleu

% Tracer la deuxième ellipse
plot(x2, y2, 'r--', 'LineWidth', 1.5); % Deuxième ellipse en rouge

% Soleil (centre du système)
plot(0, 0, 'yo', 'MarkerSize', 12, 'MarkerFaceColor', 'yellow'); % Soleil au centre

% Tracer le foyer partagé
plot(-c1, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'black'); % Foyer partagé

% Simulation du point en mouvement sur les ellipses
theta_move = 0;   % Angle initial
planet = plot(x1(1), y1(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'green'); % Planète en mouvement

% Seuil pour la détection d'entrée
threshold = 9e9; % Distance seuil pour détecter l'entrée

% Animation continue de la planète
while true
    % Mise à jour de l'angle
    theta_move = mod(theta_move + 0.01,2*pi); % Incrémenter l'angle

    % Calcul de la position de la planète sur la première ellipse
    if current_ellipse == 1
        x_planet = a1 * cos(theta_move) - c1; 
        y_planet = b1 * sin(theta_move);
    end
    if current_ellipse == 2
        x_planet = a2 * cos(theta_move) + c2; 
        y_planet = b2 * sin(theta_move);
    end
    % Vérification si le point est à l'intérieur du seuil de la deuxième ellipse
    switch_to_second_ellipse = false; % Indicateur de changement d'ellipse
    if current_ellipse == 1
      for i = 1:nPoints
        % Calcul de la distance à la deuxième ellipse
        distance = sqrt((x_planet - x2(i))^2 + (y_planet - y2(i))^2);
      
        % Si la distance est inférieure au seuil, on fait le switch
        if distance < threshold
            
            switch_to_second_ellipse = true; % Changement d'ellipse demandé
            
            % Réinitialiser les données pour la deuxième ellipse
            theta_move = theta(i); % Garder l'angle courant
            % Calculer la nouvelle position sur la deuxième ellipse
            
            break; % Sortir de la boucle une fois le switch effectué
        end
      end
    end

if current_ellipse == 2
      for i = 1:nPoints
        % Calcul de la distance à la deuxième ellipse
        distance = sqrt((x_planet - x1(i))^2 + (y_planet - y1(i))^2);
      
        % Si la distance est inférieure au seuil, on fait le switch
        if distance < threshold
            
            switch_to_second_ellipse = true; % Changement d'ellipse demandé
            
            % Réinitialiser les données pour la deuxième ellipse
            theta_move = theta(i); % Garder l'angle courant
            % Calculer la nouvelle position sur la deuxième ellipse
            
            break; % Sortir de la boucle une fois le switch effectué
        end
      end
 end



    % Si on est sur la deuxième ellipse, mettre à jour l'angle
    if switch_to_second_ellipse
        % Positionner le point légèrement après le seuil pour débuter le mouvement
        % Cela permet de passer à l'ellipse en douceur
        if current_ellipse == 1
          theta_move = mod(theta_move + 0.1, 2*pi); % Ajustement léger de l'angle
        % Calculer la nouvelle position sur la deuxième ellipse
          x_planet = a2 * cos(theta_move) + c2; 
          y_planet = b2 * sin(theta_move);
          current_ellipse = 2;
        end
        if current_ellipse ==2 
                 theta_move = mod(theta_move + 0.1, 2*pi); % Ajustement léger de l'angle
        % Calculer la nouvelle position sur la deuxième ellipse
                 x_planet = a1 * cos(theta_move)- c1; 
                 y_planet = b1 * sin(theta_move);
                 current_ellipse = 1;
        end
    end

    % Mise à jour de la position du corps sur le graphique
    set(planet, 'XData', x_planet, 'YData', y_planet);
    
    % Pause pour ralentir l'animation
    pause(0.01);
    disp(current_ellipse);
end

