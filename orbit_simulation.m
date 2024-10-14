
function [point_x, point_y] = simulate_orbit(mass, semi_major_axis, dt, current_ellipse, r0,v0, best_i,best_i2)
    G = 6.67430e-11;
    mu = G * mass;
   
    % Période orbitale et temps de simulation
    T = 2 * pi * sqrt(semi_major_axis^3 / mu);
      % Augmenter le pas de temps pour accélérer le mouvement
    t = 0:dt:T;  % Couvrir une période entière

    traj = plot(r0(1), r0(2), 'b');  % Initialiser la trajectoire vide
    point = plot(r0(1), r0(2), 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'b');  % Point satellite

        % Variables pour accumuler la trajectoire
    full_traj_x = [];  % Accumuler les coordonnées x de la trajectoire
    full_traj_y = [];  % Accumuler les coordonnées y de la trajectoire

    % Boucle infinie pour orbite continue
    
      % Initialisation des matrices pour une orbite
         r = zeros(2, length(t));
         v = zeros(2, length(t));
         r(:,best_i) = r0;
         v(:,best_i) = v0;
   
        % Simuler une orbite complète
       
         i = best_i+1;
        % Calculer les coefficients de Runge-Kutta 
        while true
         
         if mod(i,length(t)) ~= best_i2+1
         if current_ellipse == 1
         k1_r = v(:,i-1);
         k1_v = -mu / norm(r(:,i-1))^3 * r(:,i-1);
        
         k2_r = v(:,i-1) + 0.5 * dt * k1_v;
         k2_v = -mu / norm(r(:,i-1) + 0.5 * dt * k1_r)^3 * (r(:,i-1) + 0.5 * dt * k1_r);
        
         k3_r = v(:,i-1) + 0.5 * dt * k2_v;
         k3_v = -mu / norm(r(:,i-1) + 0.5 * dt * k2_r)^3 * (r(:,i-1) + 0.5 * dt * k2_r);
        
         k4_r = v(:,i-1) + dt * k3_v;
         k4_v = -mu / norm(r(:,i-1) + dt * k3_r)^3 * (r(:,i-1) + dt * k3_r);
        
          r(:,i) = r(:,i-1) + (dt / 6) * (k1_r + 2*k2_r + 2*k3_r + k4_r);
          v(:,i) = v(:,i-1) + (dt / 6) * (k1_v + 2*k2_v + 2*k3_v + k4_v);
          else
           k1_r2 = v(:,i-1);
             k1_v2 = -mu / norm(r(:,i-1) - [5000e3; 0])^3 * (r(:,i-1) - [5000e3; 0]);  % Accélération

             k2_r2 = v(:,i-1) + 0.5 * dt * k1_v2;
             k2_v2 = -mu / norm(r(:,i-1) + 0.5 * dt * k1_r2 - [5000e3; 0])^3 * (r(:,i-1) + 0.5 * dt * k1_r2 - [5000e3; 0]);

             k3_r2 = v(:,i-1) + 0.5 * dt * k2_v2;
             k3_v2 = -mu / norm(r(:,i-1) + 0.5 * dt * k2_r2 - [5000e3; 0])^3 * (r(:,i-1) + 0.5 * dt * k2_r2 - [5000e3; 0]);

             k4_r2 = v(:,i-1) + dt * k3_v2;
             k4_v2 = -mu / norm(r(:,i-1) + dt * k3_r2 - [5000e3; 0])^3 * (r(:,i-1) + dt * k3_r2 - [5000e3; 0]);

             r(:,i) = r(:,i-1) + (dt / 6) * (k1_r2 + 2*k2_r2 + 2*k3_r2 + k4_r2);
             v(:,i) = v(:,i-1) + (dt / 6) * (k1_v2 + 2*k2_v2 + 2*k3_v2 + k4_v2);
          end
         % Mise à jour de la trajectoire accumulée
         full_traj_x = [full_traj_x, r(1,i)];  % Ajoute la coordonnée x
         full_traj_y = [full_traj_y, r(2,i)];  % Ajoute la coordonnée y
        
          % Mise à jour du graphique
         set(traj, 'XData', full_traj_x, 'YData', full_traj_y);  % Affiche toute la trajectoire
         set(point, 'XData', r(1,i), 'YData', r(2,i));  % Affiche la position actuelle du satellite
         drawnow;
        
          % Réduire la pause pour augmenter la vitesse
         pause(0.001);  % Ajuster la pause selon la vitesse souhaitée
            i = i +1;
         else 
            break;
         end
        end
      

          
        
    end
  
      
    
      % Réinitialiser les positions pour recommencer une nouvelle orbite, mais
     % la trajectoire accumulée reste tracée
  



G = 6.67430e-11;
M = 5.972e24;
a = 7000e3;
e = 0.5;
decalage= 5000e3; 
time = 20;
mu = G * M; 

[x_intersection, y_intersection]= compute_ellipse_intersections(a, e, decalage);
ri1 = [x_intersection; y_intersection(2)]; %point d intersection nord 
ri2 = [x_intersection; y_intersection(1)]; % point d intersection sud

v0 = [0; sqrt(mu * (1 + e) / (a * (1 - e)))];  % Vitesse initiale

[r0_ellipse1, v0_ellipse1, i_11] = best(ri1,[a * (1 -e); 0],v0, time,mu,a,1); % premier point d intersection
[r0_ellipse12, v0_ellipse12, i_12] = best(ri2,[a * (1 -e); 0],v0, time,mu,a,1); % deuxieme point d intersection

r0_2 = [a * (1 - e) + 5000e3; 0];  % Position initiale de la deuxième ellipse
[r0_ellipse2, v0_ellipse2, i_21] = best(ri1,r0_2,v0, time,mu,a,2); % premier point d intersection
[r0_ellipse22, v0_ellipse22, i_22] = best(ri2,r0_2,v0, time,mu,a,2);

 
    figure;
    hold on;
    
    plot(0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Terre
    plot(5000e3, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');  % Centre ellipse 2
    
    axis equal;
    xlim([-1.5*a, 2*a + 5000e3]);  % Ajuster les axes pour voir les deux ellipses
    ylim([-1.5*a, 1.5*a]);
    xlabel('Position en x (m)');
    ylabel('Position en y (m)');
    title('Simulation de deux orbites elliptiques identiques se croisant');
while true
   current_ellipse = 1;
   simulate_orbit(M,a,time, current_ellipse,r0_ellipse1, v0_ellipse1, i_11, i_12);
   current_ellipse = 2;
   simulate_orbit(M,a,time, current_ellipse,r0_ellipse22, v0_ellipse22, i_22, i_21);


end



function [best_r,best_v, best_i] = best(ri,r0,v0, dt,mu,a, curr)
         
    T = 2 * pi * sqrt(a^3 / mu);
      % Augmenter le pas de temps pour accélérer le mouvement
    t = 0:dt:T;
         r = zeros(2, length(t));
         v = zeros(2, length(t));
         r(:,1) = r0;
         v(:,1) = v0;
         best_r = r(:,1);
         best_v = v(:,1);
         best_i = 1;

    for i = 2:length(t)
        if curr== 1
        % Calculer les coefficients de Runge-Kutta
         k1_r = v(:,i-1);
         k1_v = -mu / norm(r(:,i-1))^3 * r(:,i-1);
         
         k2_r = v(:,i-1) + 0.5 * dt * k1_v;
         k2_v = -mu / norm(r(:,i-1) + 0.5 * dt * k1_r)^3 * (r(:,i-1) + 0.5 * dt * k1_r);
        
         k3_r = v(:,i-1) + 0.5 * dt * k2_v;
         k3_v = -mu / norm(r(:,i-1) + 0.5 * dt * k2_r)^3 * (r(:,i-1) + 0.5 * dt * k2_r);
        
         k4_r = v(:,i-1) + dt * k3_v;
         k4_v = -mu / norm(r(:,i-1) + dt * k3_r)^3 * (r(:,i-1) + dt * k3_r);
        
         r(:,i) = r(:,i-1) + (dt / 6) * (k1_r + 2*k2_r + 2*k3_r + k4_r);
         v(:,i) = v(:,i-1) + (dt / 6) * (k1_v + 2*k2_v + 2*k3_v + k4_v);
        else
             k1_r2 = v(:,i-1);
             k1_v2 = -mu / norm(r(:,i-1) - [5000e3; 0])^3 * (r(:,i-1) - [5000e3; 0]);  % Accélération

             k2_r2 = v(:,i-1) + 0.5 * dt * k1_v2;
             k2_v2 = -mu / norm(r(:,i-1) + 0.5 * dt * k1_r2 - [5000e3; 0])^3 * (r(:,i-1) + 0.5 * dt * k1_r2 - [5000e3; 0]);

             k3_r2 = v(:,i-1) + 0.5 * dt * k2_v2;
             k3_v2 = -mu / norm(r(:,i-1) + 0.5 * dt * k2_r2 - [5000e3; 0])^3 * (r(:,i-1) + 0.5 * dt * k2_r2 - [5000e3; 0]);

             k4_r2 = v(:,i-1) + dt * k3_v2;
             k4_v2 = -mu / norm(r(:,i-1) + dt * k3_r2 - [5000e3; 0])^3 * (r(:,i-1) + dt * k3_r2 - [5000e3; 0]);

             r(:,i) = r(:,i-1) + (dt / 6) * (k1_r2 + 2*k2_r2 + 2*k3_r2 + k4_r2);
             v(:,i) = v(:,i-1) + (dt / 6) * (k1_v2 + 2*k2_v2 + 2*k3_v2 + k4_v2);
             

        end
        
         
         if (r(1,i)-ri(1))^2 + (r(2,i)-ri(2))^2 <((best_r(1)-ri(1))^2 + (best_r(2)-ri(2))^2) 
             best_r = r(:,i);
             best_v = v(:,i);
             best_i = i;
         end
    end

end

function [x_intersection, y_intersection] = compute_ellipse_intersections(a, e, d)
    % a: Demi-grand axe des ellipses
    % e: Excentricité des ellipses
    % d: Décalage horizontal entre les deux ellipses
    
    % Calcul du demi-petit axe b
    b = a * sqrt(1 - e^2);
    
    % Calcul de la coordonnée x des points d'intersection
    x_intersection = -a*e +d / 2;
    
    % Calcul de la coordonnée y des points d'intersection
    y_intersection = b * sqrt(1 - (d^2 / (4 * a^2)));
    
    % Les points d'intersection sont symétriques, donc on a deux valeurs pour y
    y_intersection = [-y_intersection, y_intersection];
end