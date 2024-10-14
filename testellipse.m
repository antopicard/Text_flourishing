function test_ellipse()

    % Paramètres de la planète
    r0 = [0, 0];         % Position initiale
    v0 = [3, 2];         % Vitesse initiale
    r_final = [100, 200]; % Position cible
    v_final_dir = [2, 1]; % Direction de la vitesse cible (normalisée)
    v_final_dir = v_final_dir / norm(v_final_dir); % Normalisation de la direction

    m_planet = 1;        % Masse de la planète

    % Initialisation du foyer (hypothèse)
    r_foyer_guess = [50, 50];
    M_foyer_guess = 500;

    % Options d'optimisation
    options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');

    % Fonction objectif
    objective = @(params) trajectory_error(params, r0, v0, r_final, v_final_dir, m_planet);

    % Recherche du foyer et de sa masse
    params_opt = fmincon(objective, [r_foyer_guess, M_foyer_guess], [], [], [], [], [], [], [], options);

    % Paramètres optimaux du foyer
    r_foyer_opt = params_opt(1:2);
    M_foyer_opt = params_opt(3);
    fprintf('Foyer optimal: [%.2f, %.2f], Masse: %.2f\n', r_foyer_opt(1), r_foyer_opt(2), M_foyer_opt);

    % Simulation de la trajectoire optimale
    [r_traj, ~] = simulate_trajectory(r0, v0, r_foyer_opt, M_foyer_opt, m_planet);

    % Plot de la trajectoire finale
    figure;
    plot(r_traj(:, 1), r_traj(:, 2), 'b-', 'LineWidth', 2); hold on; % Trajectoire
    plot(r0(1), r0(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Point initial
    plot(r_final(1), r_final(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Point final
    plot(r_foyer_opt(1), r_foyer_opt(2), 'kx', 'MarkerSize', 12, 'LineWidth', 2); % Foyer optimal
    xlabel('X Position');
    ylabel('Y Position');
    title('Trajectoire de la planète');
    legend('Trajectoire', 'Point initial', 'Point final', 'Foyer optimal');
    grid on;
    axis equal;
end

% Fonction pour simuler la trajectoire
function [r_traj, v_traj] = simulate_trajectory(r0, v0, r_foyer, M_foyer, m_planet)

    G = 6.67430e-11; % Constante gravitationnelle
    dt = 0.01;       % Intervalle de temps
    r = r0;
    v = v0;
    r_traj = r;
    v_traj = v;

    for t = 0:dt:1000  % Boucle temporelle
        r_to_foyer = r - r_foyer;
        dist_to_foyer = norm(r_to_foyer);

        if dist_to_foyer < 1e-3
            break;
        end

        % Force gravitationnelle
        F_gravity = -G * M_foyer * m_planet * r_to_foyer / dist_to_foyer^3;

        % Accélération
        a = F_gravity / m_planet;

        % Mise à jour de la vitesse et de la position
        v = v + a * dt;
        r = r + v * dt;

        % Stockage des trajectoires
        r_traj = [r_traj; r];
        v_traj = [v_traj; v];
    end
end

% Fonction pour calculer l'erreur de la trajectoire
function err = trajectory_error(params, r0, v0, r_final, v_final_dir, m_planet)

    r_foyer = params(1:2); % Position du foyer
    M_foyer = params(3);   % Masse du foyer

    % Simulation de la trajectoire
    [r_traj, v_traj] = simulate_trajectory(r0, v0, r_foyer, M_foyer, m_planet);

    % Position finale simulée
    r_final_sim = r_traj(end, :);
    v_final_sim = v_traj(end, :);

    % Calcul de l'erreur de position et de direction
    position_error = norm(r_final_sim - r_final);
    direction_error = norm(v_final_sim / norm(v_final_sim) - v_final_dir);

    % Erreur totale
    err = position_error + direction_error;
end
