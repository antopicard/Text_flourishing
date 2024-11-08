% Add the necessary MATLAB commands to run the code
clc;
clear;
close all;
%% Traitement de l'image

img = imread('C:/Users/anton/Downloads/sun1.png');  % Charger l'image
img = flipud(img);  % Inverser l'image verticalement

img_gray = rgb2gray(img);



% Seuil pour détecter les pixels noirs (texte)
seuil = 200;  
masque_pixels_noirs = img_gray < seuil;

% Squelettisation
squelette = bwmorph(masque_pixels_noirs, 'skel', inf);  % Squelettisation de l'image binaire

% Nettoyage du squelette pour enlever les petites queues
squelette_propre = bwmorph(squelette, 'clean');  % Supprime les petites branches

% Éliminer les petits brins (queues)
squelette_propre = bwmorph(squelette_propre, 'spur', 8);  % Enlève les petites "queues

% Trouver les coordonnées des pixels du squelette propre
[y_squelette, x_squelette] = find(squelette_propre);  % Obtenir les coordonnées des points squelettiques

% Créer une nouvelle figure avec un fond blanc
h = figure;
hold on;

% Définir les limites de l'axe pour correspondre à l'image
axis([0 size(img, 2) 0 size(img, 1)]);
set(gca, 'color', 'w');

% Tracer les points du squelette propre (en noir sur un fond blanc)
scatter(x_squelette, y_squelette, 5, 'black', 'filled');  % Dessiner les points du squelette

% Identifier les points extrêmes
extremites = [];  % Initialiser une liste pour stocker les points extrêmes

% Définir la tolérance pour identifier les extrémités
tolerance = 1;  % Ajustez cette valeur selon vos besoins


%% Identifier les extrémités

for i = 1:length(y_squelette)
    % Vérifier si le point (x_squelette(i), y_squelette(i)) est un extrême
    voisinage = find((abs(x_squelette - x_squelette(i)) <= tolerance) & ...
                     (abs(y_squelette - y_squelette(i)) <= tolerance));
    if length(voisinage) > 2  % Vérifier si le voisinage a plus de 2 points
        continue;  % Si plusieurs points sont dans le voisinage, ne pas ajouter à extremites
    end
    extremites = [extremites; x_squelette(i), y_squelette(i)];  % Ajouter le point à la liste des extrémités
end

%on prend un maximum de trois points si possible au hasard
n = 2;

% Indices de lignes aléatoires sans remise
indices = randperm(size(extremites, 1), n);

% Sélectionner les lignes correspondantes
pt_final = extremites(indices, :);

%{
%Tracer les points extrêmes (en rouge)
if ~isempty(pt_final)
    
    scatter(pt_final(:, 1), pt_final(:, 2), 50, 'red', 'filled', 'MarkerEdgeColor', 'black');  % Dessiner les extrémités en rouge
else
    disp('Aucun point extrême trouvé.');
end
%}

%% Segmentation du texte

n = length(x_squelette);  % Number of points
ordered_points = zeros(n, 2);  % Initialize ordered points array
visited = false(n, 1);  % Keep track of visited points

% Start from the first point
current_index = 1;
ordered_points(1, :) = [x_squelette(current_index), y_squelette(current_index)];
visited(current_index) = true;

for i = 2:n
    distances = sqrt((x_squelette - ordered_points(i-1, 1)).^2 + ...
                     (y_squelette - ordered_points(i-1, 2)).^2);
    distances(visited) = Inf;
    [~, min_index] = min(distances);
    % Update ordered points
    ordered_points(i, :) = [x_squelette(min_index), y_squelette(min_index)];
    visited(min_index) = true;  % Mark this point as visited
end

% Assuming ordered_points contains the ordered skeleton points
threshold_distance = 4;  % Define the maximum distance to keep the segments
filtered_points = ordered_points(1, :);  % Start with the first point


for i = 2:size(ordered_points, 1)
    % Calculate the distance between the last point in filtered_points and the current point
    distance = sqrt((ordered_points(i, 1) - ordered_points(i-1, 1))^2 + ...
                    (ordered_points(i, 2) - ordered_points(i-1, 2))^2);
    
    % Check if the distance is less than or equal to the threshold
    if distance > threshold_distance
      %  plot(filtered_points(:, 1), filtered_points(:, 2), '-', 'MarkerSize', 1, ...
    % 'Color', 'k', 'MarkerFaceColor', 'k', LineWidth=2);

        %on vérifie si on ne peut pas joindre le premier point du segment
         distances = sqrt((ordered_points(:,1) - filtered_points(1, 1)).^2 + ...
                     (ordered_points(:,2) - filtered_points(1, 2)).^2);
         start_index = max(1, i - size(filtered_points, 1));  % Ensure start index is within bounds
         end_index = i - 1;  % End index
         distances(start_index:end_index) = Inf;
         % Find the index of the nearest unvisited point
         [~, min_index] = min(distances);
         if distances(min_index) < threshold_distance
           % plot([ordered_points(min_index, 1), filtered_points(1, 1)], [ordered_points(min_index, 2),filtered_points(1, 2) ], 'k-', 'LineWidth', 5);
         end
        
         filtered_points = ordered_points(i,:);
    else
    filtered_points = [filtered_points;ordered_points(i,1),ordered_points(i,2)];
    end
end


%% Recherche de la tangente en chaque points 


tangentes_external = cell(size(pt_final, 1), 1);  % Initialize cell array for tangents
num_neighbors = 10;

for i = 1:size(pt_final, 1)
    curr_point = pt_final(i, :);
    tangentes_external{i} = curr_point;  % Start with the initial point in cell i
    num_neighbors = 10;
    % Loop to find neighboring points
    while num_neighbors>0
        % Calculate distances to all skeleton points
        distances = sqrt((x_squelette - curr_point(1)).^2 + ...
                         (y_squelette - curr_point(2)).^2);
                     
       for k = 1:size(tangentes_external{i}, 1)
            % Find the indices of points already in tangentes_external{i}
            index = find((x_squelette == tangentes_external{i}(k, 1)) & ...
                         (y_squelette == tangentes_external{i}(k, 2)));
            if ~isempty(index)
                distances(index) = inf;  % Set distance to inf for each point in tangentes_external{i}
            end
        end
        
        % Find the closest neighbor
        [min_distance, min_index] = min(distances);
      
        % Update tangent points in the cell array
        pt = [x_squelette(min_index), y_squelette(min_index)];
        tangentes_external{i} = [tangentes_external{i}; pt];
        
        % Update curr_point for the next iteration
        curr_point = pt;
        num_neighbors = num_neighbors - 1;

    end
end

%% Tracage d'Ellipses aux points extérieurs

degree = 1;  % Degree for linear curve fitting
tangent_slopes = zeros(size(pt_final, 1), 1);  % Initialize the tangent slopes
Ellipse = cell(size(pt_final, 1), 1);  % Initialize cell array to store ellipses

for i = 1:size(pt_final, 1)
    % Extract x and y coordinates of neighboring points
    neighbors = tangentes_external{i};
    x_neighbors = neighbors(:, 1);
    y_neighbors = neighbors(:, 2);
    
    % Polynomial fitting and tangent calculation
    p = polyfit(x_neighbors, y_neighbors, degree);
    dp = polyder(p);
    tangent_slopes(i) = atan(polyval(dp, pt_final(i, 1)));
    if y_neighbors(2)  > y_neighbors(1)
        tangent_slopes(i) = tangent_slopes(i) + pi;
    end
    
    % Calculate the ellipse orientation angle (theta) in radians
    x0 = pt_final(i, 1);
    y0 = pt_final(i, 2);
  
    % Store ellipse parameters in the cell array
    rand_theta = 0;
    nb_ellipse = 2;
    for j = 1: nb_ellipse
      
       rand_x0= 100 *rand + x0;
       rand_y0 = 50 *rand+y0;

       rand_a = 40;
       rand_b = rand_a /2;

       Ellipse{i} = [Ellipse{i}; struct('x0', rand_x0, 'y0', rand_y0, 'a', rand_a, 'b', rand_b, 'theta', rand_theta)];

       t = linspace(0, 2*pi, 100);
       x_ellipse = rand_x0 + rand_a * cos(t) * cos(rand_theta) - rand_b * (sin(t)) * sin(rand_theta);
       y_ellipse = rand_y0 + rand_a * cos(t) * sin(rand_theta) + rand_b * (sin(t)) * cos(rand_theta);
       plot(x_ellipse, y_ellipse, '--', 'LineWidth', 1.5, 'DisplayName', sprintf('Ellipse for pt %d', i));

    end
    
 %{

    % Plot the polynomial curve fit
    x_range = linspace(min(x_neighbors), max(x_neighbors), 100);
    y_fit = polyval(p, x_range);
    plot(x_range, y_fit, '-', 'LineWidth', 2, 'DisplayName', sprintf('Poly Fit for pt %d', i));

    tangent_y = polyval(p, pt_final(i, 1)) + theta * (x_range - pt_final(i, 1));
    plot(x_range, tangent_y, '--', 'LineWidth', 1.5, 'DisplayName', sprintf('Tangent at pt %d', i));
 %}
end

%% Tracé final


nbAgents = size(Ellipse,1); % Nombre d'agents
nbData = 5000; % Nombre de points dans la trajectoire
dt = 0.001; % Pas de temps
p = 0.1; % Paramètre du cycle limite
transitionDuration = 500; % Durée de la transition entre les ellipses


acceleration = 0; % Variable d'accélération
accelerationRate = 0.01; % Taux d'accélération
v_magnitude = 1;

function [last, v_final] = compute_ellipse(ellipse, tangent, pt,v_ini, t_total)

    x = zeros(2, t_total); % Initialize x to be a 2D array with two rows and nbData columns
    x(:, 1) = pt; % Assign the starting point based on pt_final

    if v_ini == 0
       v_magnitude = 1;
       vx0 = v_magnitude * cos(tangent);
       vy0 = v_magnitude * sin(tangent);
       v = [vx0; vy0];
    else
       v = v_ini;
    end
    disp(v);
    dt = 0.001; % Pas de temps
    p = 0.1; % Paramètre du cycle limite
    a = ellipse.a;
    b = ellipse.b;
    x0 = ellipse.x0;
    y0 = ellipse.y0;
    alpha = ellipse.theta;
    orientation = 0;

    ptx0 = [x0; y0] - pt;

    % Calcul du produit vectoriel en 2D (déterminant)
    cross_product = v(1) * ptx0(2) - v(2) * ptx0(1);

    % Déterminer si le point est à gauche ou à droite du vecteur
    if cross_product > 0
        orientation = 1;  % Point à gauche du vecteur
    else 
        orientation = -1; % Point à droite du vecteur
    end


    for t = 1:t_total-1
      x_shift1 = x(:, t) - [x0; y0]; % Translation vers le centre de l'ellipse
      x_rot1 = [cos(-alpha), -sin(-alpha); sin(-alpha), cos(-alpha)] * x_shift1; % Rotation inverse

          % Calcul du champ vectoriel sur l'ellipse
      dx =orientation*[-a / b * x_rot1(2); b / a * x_rot1(1)] + x_rot1 * (1 - (x_rot1(1)^2) / a^2 - (x_rot1(2)^2) / b^2) * p;

      % Rotation inverse et translation inverse du champ
      dx_rot = [cos(alpha), -sin(alpha); sin(alpha), cos(alpha)] * dx;

      % Mise à jour de la vitesse
      v =  dx_rot * dt+ max(0,1- 0.001*t)*v;

      % Mise à jour de la position
      x(:, t + 1) = x(:, t) + v ;
      if t <200
          plot(x(1,t), x(2,t), 'b.-', LineWidth=2);
      end
    end
   
    plot(x(1,:), x(2,:), 'k.', LineWidth=2);

    last = x(:,t_total);
    v_final = v;

end


for n = 1:nbAgents
    v = 0;
    [last_pos, v] = compute_ellipse(Ellipse{n}(1), tangent_slopes(n), pt_final(n,:),v, 5000);

    for i = 2:size(Ellipse{n},1)
        if norm(v) ~= 0
            v = v / norm(v);
        end
        [last_pos, v] = compute_ellipse(Ellipse{n}(i),0,last_pos,v,5000);
        
    end  
end









xlabel('X');
ylabel('Y');
axis on; 

hold off;
waitfor(h);
close all;