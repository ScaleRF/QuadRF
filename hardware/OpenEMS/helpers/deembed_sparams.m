function S_dut = deembed_with_Zc(S_meas, freq, Z0, Zc, l, er_eff)
    % Change S from Z0 to Zc
    S_zc = change_s_ref(S_meas, Z0, Zc);

    % Compute phase shift for line de-embedding
    c = 299792458;
    vp = c / sqrt(er_eff);
    beta = 2 * pi * freq / vp;
    phi = exp(-1i * beta * l);
    Phi = diag([phi, phi, phi]);

    % De-embed the lines (backward and forward propagation)
    S_dut_zc = Phi \ S_zc / Phi;

    % Change S from Zc back to Z0
    S_dut = change_s_ref(S_dut_zc, Zc, Z0);
end