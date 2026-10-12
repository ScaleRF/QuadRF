function S11_eff = effective_S11(S, Zt, Z0)
% effective_S11 computes the effective S11 when ports 2 and 3 are terminated with Zt
%
% Inputs:
%   S   - 3x3 S-parameter matrix
%   Zt  - termination impedance at ports 2 and 3 (scalar)
%   Z0  - system reference impedance (usually 50 ohms)
%
% Output:
%   S11_eff - effective reflection coefficient at port 1

    if nargin < 3
        Z0 = 50; % default system impedance
    end

    % Compute normalized reflection coefficient at ports 2 and 3
    Gamma_t = (Zt - Z0) / (Zt + Z0);
    Gamma_L = Gamma_t * eye(2); % 2x2 diagonal reflection matrix

    % Partition the S-matrix
    S11 = S(1,1);
    S12 = S(1,2:3);  % 1x2
    S21 = S(2:3,1);  % 2x1
    S22 = S(2:3,2:3); % 2x2

    % Matrix expression for effective S11
    S11_eff = S11 + S12 * ((eye(2) - S22 * Gamma_L) \ (S21 * Gamma_t));
end
