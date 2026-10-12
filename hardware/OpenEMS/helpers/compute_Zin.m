function Zin = compute_Zin(S, Z0, Zt)
% Computes input impedance Zin at port 1 of a 3-port network
% given terminations Zt on ports 2 and 3.

    if nargin < 3
        Zt = Z0;  % matched termination if not specified
    end

    % Compute reflection coefficient for terminated ports
    Gamma_t = (Zt - Z0) / (Zt + Z0);
    Gamma_L = Gamma_t * eye(2);

    % Partition S-matrix
    S11 = S(1,1);
    S12 = S(1,2:3);   % 1x2
    S21 = S(2:3,1);   % 2x1
    S22 = S(2:3,2:3); % 2x2

    % Effective reflection at port 1
    Gamma_in = S11 + S12 * ((eye(2) - S22 * Gamma_L) \ (S21 * Gamma_t));

    % Input impedance
    Zin = Z0 * (1 + Gamma_in) / (1 - Gamma_in);
end