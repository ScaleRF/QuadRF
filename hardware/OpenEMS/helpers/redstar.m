function [S_combined, port_order] = redstar(Sa, Sb, A_idx, B_idx)
% REDHEFFER_STAR Connects one port from Sa to one port from Sb using star product
%
%   [S_combined, port_order] = redheffer_star(Sa, Sb, A_idx, B_idx)
%
%   Sa, Sb: complex square S-matrices
%   A_idx: index (1-based) of port in Sa to connect
%   B_idx: index (1-based) of port in Sb to connect
%
%   Returns:
%     S_combined: combined S-matrix with connected ports removed
%     port_order: indices of remaining ports [Sa_ext, Sb_ext] (1-based global index)

    nA = size(Sa, 1);
    nB = size(Sb, 1);

    assert(size(Sa,2)==nA && size(Sb,2)==nB, 'Inputs must be square');
    assert(isscalar(A_idx) && A_idx >= 1 && A_idx <= nA);
    assert(isscalar(B_idx) && B_idx >= 1 && B_idx <= nB);

    % Build block-diagonal S-matrix
    S_full = blkdiag(Sa, Sb);

    % Define total ports
    N = nA + nB;

    % Define ports to eliminate
    int_ports = [A_idx, nA + B_idx];
    keep_ports = setdiff(1:N, int_ports);

    % Partition S_full
    S_kk = S_full(keep_ports, keep_ports);      % external -> external
    S_ki = S_full(keep_ports, int_ports);       % external -> internal
    S_ik = S_full(int_ports, keep_ports);       % internal -> external
    S_ii = S_full(int_ports, int_ports);        % internal -> internal

    % Check invertibility
    I = eye(size(S_ii));
    D = I - S_ii;
    if rcond(D) < 1e-12
        warning('Near singular matrix in star product denominator');
    end

    % Apply network reduction (Redheffer star product)
    S_combined = S_kk + S_ki * (D \ S_ik);

    % Track port ordering: extA followed by extB
    extA = setdiff(1:nA, A_idx);
    extB = setdiff(1:nB, B_idx);
    port_order = [extA, nA + extB];  % global indices
end