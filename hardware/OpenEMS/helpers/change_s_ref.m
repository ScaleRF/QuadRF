% Change the reference impedance of S-param matrix
function S_new = change_s_ref(S_old, Z_old, Z_new)
    I = eye(size(S_old));
    % Convert to Z-parameters
    Z = Z_old * (I + S_old) / (I - S_old);
    % Convert to new S-parameters
    S_new = (Z - Z_new * I) / (Z + Z_new * I);
end