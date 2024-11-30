
function Main()
    h = figure;
    hold on;

    [x_skeleton, y_skeleton] = ImageTreatment("flourishing_");
    pt_final = compute_extremites(x_skeleton, y_skeleton);
    neighbours = compute_neighbours(x_skeleton, y_skeleton, pt_final);
    [Ellipse, tangent_slopes] = ellipse_setup(pt_final,neighbours, x_skeleton, y_skeleton);
    points = trace(Ellipse,tangent_slopes,pt_final);
    


    DisplayImageWithAddedPoints("flourishing_",points); 
    
    waitfor(h);
    close all;

end

Main();

%% Traitement de l'image

function [x_skeleton, y_skeleton] = ImageTreatment(image)
    imgPath = strcat('C:/Users/anton/Downloads/', image, '.png');
    img = imread(imgPath);                                                 
    img = flipud(img);                                                      % Invert image vertically 
    img_gray = rgb2gray(img);                                               % convert image in grayscale 

    threshold = 200;                                                        % threshold to detect black pixels
    mask = img_gray < threshold;
    skeleton = bwmorph(mask, 'skel', inf);                                  % Skeletisation of the image
    clean_skeleton = bwmorph(skeleton, 'clean');                            % remove branch on the skeleton
    clean_skeleton = bwmorph(clean_skeleton, 'spur', 8);                    % clean the skeleton

    [y_skeleton, x_skeleton] = find(clean_skeleton);                           

    axis([0 size(img, 2) 0 size(img, 1)]);                                  
    set(gca, 'color', 'w');
    scatter(x_skeleton, y_skeleton, 5, 'black', 'filled');                  
end

%% Identify extremites

function pt_final = compute_extremites(x_skeleton, y_skeleton)
    extremites = [];  
    tolerance = 1; 
    n = 2;                                                                  % number of extreme point we want to take
    pt_final = [];

    for i = 1:length(y_skeleton)                                                            
       neighbors = find((abs(x_skeleton - x_skeleton(i)) <= tolerance) & ...% verify if it is an extreme point
                     (abs(y_skeleton - y_skeleton(i)) <= tolerance));
       if length(neighbors) > 2                                             % check wether there are more than one neighbors
           continue;                                                        
       end
       extremites = [extremites; x_skeleton(i), y_skeleton(i)];            
    end

    % A changer absolument plus tard
    indices = randperm(size(extremites, 1), n);                             % Indices taken as the start of our flourishing
    pt_final = extremites(indices, :); 
    pt_final = sortrows(pt_final, 1);

    %Tracer les points extrêmes (en rouge)
    if ~isempty(pt_final)
        scatter(pt_final(:, 1), pt_final(:, 2), 50, 'red', 'filled', ...
        'MarkerEdgeColor', 'black'); 
    else
        disp('Aucun point extrême trouvé.');
    end
    
end
%% Segmentation du texte

function segmentation(x_skeleton, y_skeleton)
    n = length(x_skeleton);  % Number of points
    ordered_points = zeros(n, 2);  % Initialize ordered points array
    visited = false(n, 1);  % Keep track of visited points

    % Start from the first point
    current_index = 1;
    ordered_points(1, :) = [x_skeleton(current_index), y_skeleton(current_index)];
    visited(current_index) = true;

    for i = 2:n
        distances = sqrt((x_skeleton - ordered_points(i-1, 1)).^2 + ...
                     (y_skeleton - ordered_points(i-1, 2)).^2);
        distances(visited) = Inf;
        [~, min_index] = min(distances);
        % Update ordered points
        ordered_points(i, :) = [x_skeleton(min_index), y_skeleton(min_index)];
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
end

%% Recherche de la tangente en chaque points 

function neighbours = compute_neighbours(x_skeleton, y_skeleton, pt_final)
    neighbours = cell(size(pt_final, 1), 1);  

    for i = 1:size(pt_final, 1)
        curr_point = pt_final(i, :);
        neighbours{i} = curr_point;  
        num_neighbours = 10;
        % Loop to find neighboring points
        while num_neighbours>0
            % Calculate distances to all skeleton points
            distances = sqrt((x_skeleton - curr_point(1)).^2 + ...
                         (y_skeleton - curr_point(2)).^2);
                     
            for k = 1:size(neighbours{i}, 1)
                % Find the indices of points already in neighbours{i}
                index = find((x_skeleton == neighbours{i}(k, 1)) & ...
                         (y_skeleton == neighbours{i}(k, 2)));
                if ~isempty(index)
                    distances(index) = inf;  % Set distance to inf for each point in neighbours{i}
                end
            end
        
            % Find the closest neighbor
            [min_distance, min_index] = min(distances);
      
            % Update tangent points in the cell array
            pt = [x_skeleton(min_index), y_skeleton(min_index)];
            neighbours{i} = [neighbours{i}; pt];
        
            % Update curr_point for the next iteration
            curr_point = pt;
            num_neighbours = num_neighbours - 1;

       end
    end
end

%% Tracage d'Ellipses aux points extérieurs

function [Ellipse, tangent_slopes] = ellipse_setup(pt_final, neighbours, x_skeleton, y_skeleton)
    degree = 1;                                                             % Degree for linear curve fitting
    tangent_slopes = zeros(size(pt_final, 1), 1);                           % Initialize the tangent slopes

    for i = 1:size(pt_final, 1)
        % Extract x and y coordinates of neighboring points
        x_neighbours = neighbours{i}(:, 1);
        y_neighbours = neighbours{i}(:, 2);
    
        % Polynomial fitting and tangent calculation
        p = polyfit(x_neighbours, y_neighbours, degree);
        dp = polyder(p);
        tangent_slopes(i) = atan(polyval(dp, pt_final(i, 1)));
        if y_neighbours(2)  > y_neighbours(1)
            tangent_slopes(i) = tangent_slopes(i) + pi;
        end
    
     %{ 
        % Calculate the ellipse orientation angle (theta) in radians
        x0 = pt_final(i, 1);
        y0 = pt_final(i, 2);
  
        % Store ellipse parameters in the cell array
        rand_theta = 0;
        nb_ellipse = 2;
        for j = 1: nb_ellipse
            rand_x0 = 100 *rand + x0;
            rand_y0 = 50 *rand+y0;
            rand_a = 40;
            rand_b = rand_a /2;

            Ellipse{i} = [Ellipse{i}; struct('x0', rand_x0, 'y0', rand_y0, 'a', rand_a, 'b', rand_b, 'theta', rand_theta)];

            t = linspace(0, 2*pi, 100);
            x_ellipse = rand_x0 + rand_a * cos(t) * cos(rand_theta) - rand_b * (sin(t)) * sin(rand_theta);
            y_ellipse = rand_y0 + rand_a * cos(t) * sin(rand_theta) + rand_b * (sin(t)) * cos(rand_theta);
            plot(x_ellipse, y_ellipse, '--', 'LineWidth', 1.5, 'DisplayName', sprintf('Ellipse for pt %d', i));

        end
    
   

        % Plot the polynomial curve fit
        x_range = linspace(min(x_neighbours), max(x_neighbours), 100);
        y_fit = polyval(p, x_range);
        plot(x_range, y_fit, '-', 'LineWidth', 2, 'DisplayName', sprintf('Poly Fit for pt %d', i));

        tangent_y = polyval(p, pt_final(i, 1)) + theta * (x_range - pt_final(i, 1));
        plot(x_range, tangent_y, '--', 'LineWidth', 1.5, 'DisplayName', sprintf('Tangent at pt %d', i));
    %}
    end
    Ellipse = place_ellipses2(x_skeleton, y_skeleton,pt_final, tangent_slopes);
    %Ellipse = place_ellipses(x_skeleton, y_skeleton,pt_final);

end

%% Tracé final
function y = sigmoid(x)
    y = 1 / (1 + exp(-x));
end



function [last, v_final, points] = compute_ellipse(ellipse, tangent, pt,v_ini, t_total, points)

    x = zeros(2, t_total); % Initialize x to be a 2D array with two rows and nbData columns
    x(:, 1) = pt; % Assign the starting point based on pt_final
        
     v_magnitude = 0.06;
     if v_ini == 0
       
       vx0 = v_magnitude * cos(tangent);
       vy0 = v_magnitude * sin(tangent);
       vi = [vx0; vy0];
     else
       vi = v_ini/norm(v_ini)*v_magnitude;
     end
    dt = 0.001; % Pas de temps
    p = 0.5; % Paramètre du cycle limite
    a = ellipse.a;
    b = ellipse.b;
    x0 = ellipse.x0;
    y0 = ellipse.y0;
    alpha = ellipse.theta;
    orientation = ellipse.orientation;

    ptx0 = zeros(2,1);
    ptx0(1,1)= x0;
    ptx0(2,1)= y0;
    ptx0(:,1) = ptx0(:,1)-x(:,1);

    % Calcul du produit vectoriel en 2D (déterminant)
    %cross_product = vi(1) * ptx0(2) - vi(2) * ptx0(1);

    % Déterminer si le point est à gauche ou à droite du vecteur
   % orientation = 1 * (cross_product > 0) - 1 * (cross_product <= 0);
    
    for t = 1:t_total-1
      x_shift1 = x(:, t) - [x0; y0];                                        
      x_rot1 = [cos(-alpha), -sin(-alpha); sin(-alpha), cos(-alpha)] * x_shift1; % Rotation inverse

      % Calcul du champ vectoriel sur l'ellipse
      dx = orientation*[-a / b * x_rot1(2); b / a * x_rot1(1)] + x_rot1 * (1 - (x_rot1(1)^2) / a^2 - (x_rot1(2)^2) / b^2) * p;

      % Rotation inverse et translation inverse du champ
      dx_rot = [cos(alpha), -sin(alpha); sin(alpha), cos(alpha)] * dx;

      % Mise à jour de la vitesse
      % Calcule un facteur de transition qui évolue linéairement de 0 à 1 au cours du temps
       transition_factor = min(1, (t*0.02/ 5000));
      % transition_factor = min(1,sigmoid(t/2000-4));

      % Combinaison de la vitesse initiale et du champ vectoriel sur l'ellipse
      v = transition_factor * dx_rot * dt + (1 - transition_factor) * vi;

      % Mise à jour de la position
      x(:, t + 1) = x(:, t) + v;
      vi = v;
      points = [points; x(1,t+1), x(2,t+1)];
      v_final = v;
    end
   
    plot(x(1,:), x(2,:), 'k.', LineWidth=1);
    last = x(:,t_total);
    

end

function [last, v_final, points] = mixed_ellipses(Ellipse1, Ellipse2, last_pos, v, nbData, points)

    x = zeros(2, nbData); % Initialize x to be a 2D array with two rows and nbData columns
    x(:, 1) = last_pos; % Assign the starting point based on pt_final
    dt = 0.001; % Pas de temps
    p = 0.1; % Paramètre du cycle limite

    a1 = Ellipse1.a;
    b1 = Ellipse1.b;
    x01 = Ellipse1.x0;
    y01 = Ellipse1.y0;
    alpha1 = Ellipse1.theta;
    orientation1 = 0;

    a2 = Ellipse2.a;
    b2 = Ellipse2.b;
    x02 = Ellipse2.x0;
    y02 = Ellipse2.y0;
    alpha2 = Ellipse2.theta;
    orientation2 = 0;

    ptx01 = zeros(2,1);
    ptx01(1,1)= x01;
    ptx01(2,1)= y01;
    ptx01(:,1) = ptx01(:,1)-x(:,1);
    
    ptx02 = zeros(2,1);
    ptx02(1,1)= x02;
    ptx02(2,1)= y02;
    ptx02(:,1) = ptx02(:,1)-x(:,1);

    % Calcul du produit vectoriel en 2D (déterminant)
    cross_product1 = v(1) * ptx01(2) - v(2) * ptx01(1);
    cross_product2 = v(1) * ptx02(2) - v(2) * ptx02(1);

    % Déterminer si le point est à gauche ou à droite du vecteur
    orientation1 = 1 * (cross_product1 > 0) - 1 * (cross_product1 <= 0);
    orientation2 = 1 * (cross_product2 > 0) - 1 * (cross_product2 <= 0);
    for t = 1:nbData-1
      x_shift1 = x(:, t) - [x01; y01]; 
      x_shift2 = x(:, t) - [x02; y02];
      x_rot1 = [cos(-alpha1), -sin(-alpha1); sin(-alpha1), cos(-alpha1)] * x_shift1; % Rotation inverse
      x_rot2 = [cos(-alpha2), -sin(-alpha2); sin(-alpha2), cos(-alpha2)] * x_shift2; % Rotation inverse


      % Calcul du champ vectoriel sur l'ellipse
      dx1 = orientation1*[-a1 / b1 * x_rot1(2); b1 / a1 * x_rot1(1)] + x_rot1 * (1 - (x_rot1(1)^2) / a1^2 - (x_rot1(2)^2) / b1^2) * p;
      dx2 = orientation2*[-a2 / b2 * x_rot2(2); b2 / a2 * x_rot2(1)] + x_rot2 * (1 - (x_rot2(1)^2) / a2^2 - (x_rot2(2)^2) / b2^2) * p;

      % Rotation inverse et translation inverse du champ
      dx_rot1 = [cos(alpha1), -sin(alpha1); sin(alpha1), cos(alpha1)] * dx1;
      dx_rot2 = [cos(alpha2), -sin(alpha2); sin(alpha2), cos(alpha2)] * dx2;

      % Mise à jour de la vitesse
      transition_factor = min(1, (t/ 100));

      % Combinaison de la vitesse initiale et du champ vectoriel sur l'ellipse
      v = (1-transition_factor) * dx_rot1 *8*dt + (transition_factor) * dx_rot2 * dt;

      %v = max(0, 1 - 0.0005*t) * dx_rot1 * dt + min(1, 0.0005*t) * dx_rot2 * dt;

      % Mise à jour de la position
      x(:, t + 1) = x(:, t) + v;
      points = [points; x(1,t+1), x(2,t+1)];
      v_final = v;
    end
   
    plot(x(1,:), x(2,:), 'k.', LineWidth=1);
   
    last = x(:,nbData);
    

end

function points = trace(Ellipse, tangent_slopes, pt_final)
    points = [];
    nbAgents = size(Ellipse,1); % Nombre d'agents
    
    for n = 1:nbAgents
        v = 0;
        [last_pos, v, points] = compute_ellipse(Ellipse{n}(1), tangent_slopes(n), pt_final(n,:),v, 4500,points);
        for i = 2:size(Ellipse{n},2)
            
            if norm(v) ~= 0
                v = v/ norm(v);
            end
              [last_pos, v, points] = compute_ellipse(Ellipse{n}(i), 0, last_pos, v, 5000, points);
              %[last_pos, v, points] = mixed_ellipses(Ellipse{n}(i-1), Ellipse{n}(i), last_pos, v, 5000, points);
        end  
    end
end


function find_borders(pt_final, x_skeleton, y_skeleton)
    Borders = [];
    top = 400;
    bottom = 0;
    right = 1100;
    left = 0;

    left_border = left;
    mean_y = mean(y_skeleton);
    pt_up = pt_final(pt_final(:,2) >= mean_y);
    pt_down = pt_final(pt_final(:,2) < mean_y);
    
 
end

function Ellipses = place_ellipses(x_skeleton, y_skeleton, pt_final)
    top = max(y_skeleton);
    bottom = min(y_skeleton)+30;
    right = max(x_skeleton)+100;
    left = 0;
    Ellipses = cell(size(pt_final, 1), 1);
    for i = 1:size(pt_final,1)
            theta = 0:99;
            theta = theta(randperm(length(theta)))*(2 * pi / 100);
            pt = pt_final(i,:);
            xy_0 = [];

            for j = 1:99
                t = theta(j);
                R = [cos(t), -sin(t); sin(t), cos(t)];
                if isempty(xy_0)
                    x0_y0 = [70,0]*R + pt;
                else 
                    x0_y0 = [60,0]*R + pt;
                end
                
                distances1 = [x_skeleton, y_skeleton]-x0_y0;
                distances1 = vecnorm(distances1, 2, 2);
                [min_distance, min_index]= min(distances1);
                
                
                check = false;
                if isempty(xy_0)
                    check = true;
                else 
                    distances2 = xy_0 - x0_y0;
                    distances2 = vecnorm(distances2,2,2);
                    [d,min_idx]= min(distances2);
                    check = distances2(min_idx)>=60;
                end

                if distances1(min_index) > 40 && check && x0_y0(2)<top && x0_y0(2)>bottom && x0_y0(1)>left && x0_y0(1)<right && size(xy_0,1) <2
                    rand_theta = rand*2*pi;
                    Ellipses{i}= [Ellipses{i};struct('x0', x0_y0(1), 'y0', x0_y0(2), 'a', 30, 'b', 20, 'theta', rand_theta)];
                    pt = x0_y0;
                    xy_0 = [xy_0; x0_y0];

                    v = linspace(0, 2*pi, 100);
                    x_ellipse = x0_y0(1) + 30 * cos(v) * cos(rand_theta) - 20 * (sin(v)) * sin(rand_theta);
                    y_ellipse = x0_y0(2) + 30 * cos(v) * sin(rand_theta) + 20 * (sin(v)) * cos(rand_theta);
                    plot(x_ellipse, y_ellipse, '--', 'LineWidth', 1.5, 'DisplayName', sprintf('Ellipse for pt %d', i));
                end
            end 
    end
end
function Ellipse = place_ellipses_first(x_skeleton, y_skeleton, pt_final,tangent)
    top = max(y_skeleton);
    bottom = min(y_skeleton)+30;
    right = max(x_skeleton)+100;
    left = 0;
    theta = 0:99;
    theta = theta(randperm(length(theta)))*(2 * pi / 100);
    pt = pt_final;
    x0_y0 = [0,0];

        found = false;
        vector = [70,0];
        limit = 100;
        j = 1;
        while ~found && vector(1)<limit
            while ~found && j <100
                t = theta(j);
                R = [cos(t), -sin(t); sin(t), cos(t)];
                x0_y0 = vector *R + pt;
                
                distances1 = [x_skeleton, y_skeleton]-x0_y0;
                distances1 = vecnorm(distances1, 2, 2);
                [min_distance, min_index]= min(distances1);
                
            

                if min_distance >= 40 && x0_y0(2)<top && x0_y0(2)>bottom && x0_y0(1)>left && x0_y0(1)<right
                    rand_theta = 0;
                     ptx0 = zeros(2,1);
                     ptx0(1,1)= x0_y0(1);
                     ptx0(2,1)= x0_y0(2);
                     ptx0(:,1) = ptx0(:,1) - pt';
                     vi = [cos(tangent);sin(tangent)];

                     cross_product = vi(1) * ptx0(2) - vi(2) * ptx0(1);
                     orientation = 1 * (cross_product > 0) - 1 * (cross_product <= 0);
                     if x0_y0(1)> pt(1)
                         direction = "right";
                     else
                         direction = "left";
                     end
                    Ellipse = struct('x0', x0_y0(1), 'y0', x0_y0(2), 'a', 30, 'b', 20, 'theta', rand_theta, 'orientation', orientation, 'direction', direction);
                  
                    v = linspace(0, 2*pi, 100);
                    x_ellipse = x0_y0(1) + 30 * cos(v) * cos(rand_theta) - 20 * (sin(v)) * sin(rand_theta);
                    y_ellipse = x0_y0(2) + 30 * cos(v) * sin(rand_theta) + 20 * (sin(v)) * cos(rand_theta);
                    plot(x_ellipse, y_ellipse, '--', 'LineWidth', 1.5, 'DisplayName', sprintf('Ellipse for pt %d', 1));
                    found = true;
                end
                j = j+1;
            end 
            vector = vector +[1,0];
        end
    
end
function Ellipses = place_ellipses2(x_skeleton, y_skeleton, pt_final, tangent_slopes)
    Ellipses = cell(size(pt_final, 1), 1); 
    list_ellipses = [];

    for i= 1:size(pt_final,1)
        mode = "s";
        pt = pt_final(i,:);
        change = false;
        
        Ellipse = place_ellipses_first(x_skeleton, y_skeleton, pt,tangent_slopes(i));
        Ellipses{i}= [Ellipses{i}, Ellipse];
        direction = Ellipse.direction;
        orientation = Ellipse.orientation;

        while ~change
            if mode == "s" 
                if strcmp(direction, 'right')
                      direction = 'left';
                   else
                      direction = 'right';
                end

               orientation = -orientation;
               [new_ellipse, list_ellipses] = build_S(pt, x_skeleton,y_skeleton, orientation,direction, list_ellipses);
               if new_ellipse.x0 ~= 0
                   Ellipses{i}= [Ellipses{i}, new_ellipse];
                   pt = [new_ellipse.x0, new_ellipse.y0];
                   
               else
                   change = true;
               end
               disp(size(list_ellipses,2));
               %change = true;
               mode = "c"; 
            else 
                disp("c");
               [new_ellipse, list_ellipses] = build_C(pt, x_skeleton,y_skeleton, orientation, direction, list_ellipses);
               if new_ellipse.x0 ~= 0
                   Ellipses{i}= [Ellipses{i}, new_ellipse];
                   pt = [new_ellipse.x0, new_ellipse.y0];

                   
               else
                   change = true ;
               end
               disp(size(list_ellipses,2));

               change = true;
                mode = "s";
            end
        end
    end
    
end

function [ellipse, list_ellipses] = build_S(pt, x_skeleton, y_skeleton, orientation, direction, list_ellipses)
     x0_y0 = check_spaceS(pt, x_skeleton,y_skeleton,direction, list_ellipses);
     if x0_y0(1)~=0
        ellipse = struct('x0', x0_y0(1), 'y0', x0_y0(2), 'a', 30, 'b', 20, 'theta', 0,'orientation', orientation, 'direction' , direction);
        list_ellipses = [list_ellipses, ellipse];
         v = linspace(0, 2*pi, 100);
                    x_ellipse = x0_y0(1) + 30 * cos(v) * cos(0) - 20 * (sin(v)) * sin(0);
                    y_ellipse = x0_y0(2) + 30 * cos(v) * sin(0) + 20 * (sin(v)) * cos(0);
                    plot(x_ellipse, y_ellipse, '--', 'LineWidth', 1.5, 'DisplayName', sprintf('Ellipse for pt %d', 1));
     else 
         ellipse= struct('x0', 0);
     end
end

function [ellipse, list_ellipses] = build_C(pt, x_skeleton, y_skeleton, orientation, direction, list_ellipses)
     x0_y0 = check_spaceC(pt, x_skeleton,y_skeleton,orientation, direction, list_ellipses);
     if x0_y0(1) ~= 0
        ellipse = struct('x0', x0_y0(1), 'y0', x0_y0(2), 'a', 30, 'b', 20, 'theta', 0,'orientation', orientation, 'direction' , direction);
        list_ellipses = [list_ellipses, ellipse];
        v = linspace(0, 2*pi, 100);
        x_ellipse = x0_y0(1) + 30 * cos(v) * cos(0) - 20 * (sin(v)) * sin(0);
        y_ellipse = x0_y0(2) + 30 * cos(v) * sin(0) + 20 * (sin(v)) * cos(0);
        plot(x_ellipse, y_ellipse, '--', 'LineWidth', 1.5, 'DisplayName', sprintf('Ellipse for pt %d', 1));
     else
         ellipse = struct('x0',0);
     end
end

function x0_y0 = check_spaceS(pt, x_skeleton, y_skeleton,direction, list_ellipses)
   
    top = 350;
    bottom = 50;
    right = max(x_skeleton)+100;
    left = min(x_skeleton)-70;

    vector = [110,0];
    if strcmp(direction, 'right')
        indices = -12:12;
        theta = indices * (2 * pi / 100);
        theta = theta(randperm(length(theta)));
    else 
        indices = -12:12;
        indices = indices +50;
        theta = indices * (2 * pi / 100);
        theta = theta(randperm(length(theta)));
    end
    min_dist = 40;
    
    x0_y0=[0,0];
    dist = false;
    while ~dist && vector(1) >= min_dist
        t = 1;
        while ~dist && t<25
            curr_theta = theta(t);
            R = [cos(curr_theta), -sin(curr_theta); sin(curr_theta), cos(curr_theta)];
            x0_y0 = vector * R + pt;
            distances = [x_skeleton, y_skeleton]-x0_y0;
            distances = vecnorm(distances, 2, 2);
            [min_distance, min_index]= min(distances);

            if min_distance > min_dist && x0_y0(2)<top && x0_y0(2)>bottom && x0_y0(1)>left && x0_y0(1)<right
                dist = true;
            end
            
            for j = 1:size(list_ellipses,2)
                a = list_ellipses(j).a;
                center = [list_ellipses(j).x0, list_ellipses(j).y0];
                if vecnorm(x0_y0-center) < max(a,30)
                    dist = false;
                end
            end

            t = t+1;
            
        end
        vector = vector -[1,0];
    end
    if ~dist
        x0_y0=[0,0];
    end

end

function x0_y0 = check_spaceC(pt, x_skeleton, y_skeleton, orientation, direction, list_ellipses)
   
    top = 350;
    bottom = 50;
    right = max(x_skeleton)+100;
    left = min(x_skeleton)-70;
    disp(orientation);
    vector = [90,0];
    if (strcmp(direction, 'right') && orientation ==1)||(strcmp(direction, 'left')&& orientation == -1)
        indices = 20:30;
        theta = indices * (2 * pi / 100);
        theta = theta(randperm(length(theta)));
    else 
        indices = 70:80;
        theta = indices * (2 * pi / 100);
        theta = theta(randperm(length(theta)));
    end
    min_dist = 60;
    
    x0_y0=[0,0];
    dist = false;
    while ~dist && vector(1) >= min_dist
        t = 1;
        while ~dist && t<12
            curr_theta = theta(t);
            R = [cos(curr_theta), -sin(curr_theta); sin(curr_theta), cos(curr_theta)];
            x0_y0 = vector * R + pt;
            distances = [x_skeleton, y_skeleton]-x0_y0;
            distances = vecnorm(distances, 2, 2);
            [min_distance, min_index]= min(distances);

            if min_distance > min_dist && x0_y0(2)<top && x0_y0(2)>bottom && x0_y0(1)>left && x0_y0(1)<right
                dist = true;
            end
            
            for j = 1:size(list_ellipses,2)
                a = list_ellipses(j).a;
                center = [list_ellipses(j).x0, list_ellipses(j).y0];
                if vecnorm(x0_y0-center) < max(a,30)
                    dist = false;
                end
            end

            t = t+1;
            
        end
        vector = vector -[1,0];
    end
    if ~dist
        x0_y0=[0,0];
    end

end

function DisplayImageWithAddedPoints(originalImageName, x_y_points)
    % Load the original image
    h = figure;
    hold("on");
    imgPath = strcat('C:/Users/anton/Downloads/', originalImageName, '.png');
    img = imread(imgPath);  
    % Ensure the image is grayscale or RGB
    if size(img, 3) == 3
        img_gray = rgb2gray(img);
    else
        img_gray = img;
    end
    
    % Convert the grayscale image to RGB to add colored points
    img_rgb = cat(3, img_gray, img_gray, img_gray);
    [a,b]= max(x_y_points(:,1));
    
    % Étendre la largeur de l'image en ajoutant des colonnes blanches (255)
    % additionalWidth détermine combien de colonnes ajouter de chaque côté
    
    additional = max(round(x_y_points(b,1)))-size(img_rgb,2)+50;
    if additional>0
        rightPadding = uint8(255 * ones(size(img_rgb, 1), additional, 3));
        img_rgb = [img_rgb, rightPadding];
    end

    % Get the image height
    imgHeight = size(img_rgb, 1);
    
    % Invert y-coordinates to match the image coordinate system
    x_y_points(:, 2) = imgHeight - x_y_points(:, 2);

    pointColor = [1, 0, 0];

    % Define point size (radius of the circle)
    pointRadius = 2; % Increase this value for larger points
    % Add each point in x_y_points to the image as a filled circle
    numPoints = size(x_y_points, 1);
    for i = 1:numPoints
        x = round(x_y_points(i, 1));
        y = round(x_y_points(i, 2));
        
        % Ensure the coordinates are within bounds
        if y > pointRadius && y <= size(img_rgb, 1) - pointRadius && ...
           x > pointRadius && x <= size(img_rgb, 2) - pointRadius
            % Draw a filled circle (disk) at the specified point
            for dx = -pointRadius:pointRadius
                for dy = -pointRadius:pointRadius
                    if dx^2 + dy^2 <= pointRadius^2  % Check within circle
                        img_rgb(y + dy, x + dx, :) = pointColor;
                    end
                end
            end
        end
    end

    % Display the modified image
    imshow(img_rgb);
    title('Image with Added Points');
end






