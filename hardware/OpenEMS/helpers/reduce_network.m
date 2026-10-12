function [S_total, ext_ports] = reduce_network(blocks, connections)
%REDUCE_NETWORK Multi-port Redheffer star reduction with exact pairwise equivalence.
%
%   [S_total, ext_ports] = reduce_network(blocks, connections)
%
% Inputs:
%   blocks      - array of structs with:
%                   .S     (ni x ni): local S-matrix
%                   .ports (1 x ni): global port labels
%   connections - k x 2 array: each row [pA, pB] connects global ports
%
% Outputs:
%   S_total     - reduced S-matrix with all internal ports eliminated
%   ext_ports   - remaining global port indices

% Collect all ports and matrices
all_ports = [];
for i = 1:length(blocks)
    all_ports = [all_ports, blocks(i).ports];
end
n_ports = length(all_ports);
unique_ports = unique(all_ports);
if length(unique_ports) ~= n_ports
    error('Duplicate port labels detected.');
end

% Initialize global S-matrix
S_global = zeros(n_ports);
port_map = containers.Map(all_ports, 1:n_ports);

% Fill global S-matrix
for i = 1:length(blocks)
    ports = blocks(i).ports;
    S = blocks(i).S;
    idx = cell2mat(values(port_map, num2cell(ports)))';
    S_global(idx, idx) = S;
end

% Identify internal and external ports
conn_ports = unique(connections(:));
ext_ports = setdiff(all_ports, conn_ports);

% Get indices for external and internal ports, ensuring column vectors
ext_idx = cell2mat(values(port_map, num2cell(ext_ports)))';
int_idx = cell2mat(values(port_map, num2cell(conn_ports)))';
int_idx = int_idx(:);
% Handle empty port cases
if isempty(ext_ports)
    ext_idx = [];
end
if isempty(conn_ports)
    int_idx = [];
end

% Check if both ext_idx and int_idx are empty
if isempty(ext_idx) && isempty(int_idx)
    error('No ports available after processing connections.');
end

% Reorder S-matrix: external ports first, then internal
perm_idx = [ext_idx; int_idx];
S_global = S_global(perm_idx, perm_idx);

% Partition S-matrix
n_ext = length(ext_ports);
n_int = length(conn_ports);
S_ee = S_global(1:n_ext, 1:n_ext);
S_ei = S_global(1:n_ext, n_ext+1:end);
S_ie = S_global(n_ext+1:end, 1:n_ext);
S_ii = S_global(n_ext+1:end, n_ext+1:end);

% For a single connection, align with redstar
if size(connections, 1) == 1
    % Map connections to internal port indices
    pA = connections(1, 1);
    pB = connections(1, 2);
    idx_A = find(conn_ports == pA);
    idx_B = find(conn_ports == pB);

    % Partition for redstar compatibility
    SAA = S_ii(idx_A, idx_A);
    SBB = S_ii(idx_B, idx_B);
    SAx = S_ii(idx_A, idx_B);
    SxA = S_ii(idx_B, idx_A);
    Sxx = S_ee;
    SBx = S_ei(:, idx_B);
    SxB = S_ie(idx_B, :);
    Syy = S_ee; % Since external ports include both blocks' external ports

    % Compute Redheffer star product
    I = eye(1);
    D = I - SAA * SBB;
    if abs(det(D)) < 1e-12
        warning('Near singular matrix in Redheffer product, result may be inaccurate.');
    end

    S11 = Sxx + SxA * inv(D) * SAA * SAx;
    S12 = SxA * inv(D) * SBx;
    S21 = SxB * inv(D) * SAx;
    S22 = Syy + SxB * inv(D) * SBB * SBx;

    S_total = [S11, S12; S21, S22];
else
    % General case for multiple connections
    % Create connection matrix C for internal ports
    C = zeros(n_int);
    for i = 1:size(connections, 1)
        pA = connections(i, 1);
        pB = connections(i, 2);
        idx_A = find(conn_ports == pA);
        idx_B = find(conn_ports == pB);
        C(idx_A, idx_B) = 1;
        C(idx_B, idx_A) = 1;
    end
    I = eye(n_int);
    D = I - S_ii * C;
    if abs(det(D)) < 1e-12
        warning('Near singular matrix in Redheffer product, result may be inaccurate.');
    end
    S_total = S_ee + S_ei * inv(D) * S_ie;
end

% Reorder S_total to match ext_ports order
[~, idx] = sort(ext_ports);
S_total = S_total(idx, idx);

end