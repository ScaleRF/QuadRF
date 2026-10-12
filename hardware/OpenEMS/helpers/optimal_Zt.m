function [Zt_opt, Gamma_t_opt] = optimal_Zt(S, Z0)
% Finds the optimal single impedance Zt for source and loads
% to minimize power lost (reflected + dissipated) at a 3-port

    if nargin < 2
        Z0 = 50;
    end
    
    % Constrain |Gamma| < 1 (passive loads only)
    gamma_try = linspace(-0.99, 0.99,10000);
    best_idx = 0;
    best_loss = 10000;
    for k = 1:length(gamma_try)

        loss = loss_fraction(S, gamma_try(k));
        if (loss < best_loss)
            best_loss = loss;
            best_idx = k;
        end
    end

    % Convert Gamma to Zt
    Gamma_t_opt = gamma_try(best_idx);
    Zt_opt = Z0 * (1 + Gamma_t_opt) / (1 - Gamma_t_opt);
end

function loss = loss_fraction(S, Gamma_t)
    b1 = S * [1;0;0]; % incident
    a = diag([Gamma_t; Gamma_t; Gamma_t]) * b1;
    b2 = S * a;
    loss = norm(b2)^2;
end