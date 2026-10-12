%% === Analysis and Far-Field Postprocessing ===
freq = linspace(max([1e9, f0 - fc]), f0 + fc, 501);

% Calculate all port parameters (this already processes all 21 ports)
for n = 1:total_ports
    ports{n} = calcPort(ports{n}, Sim_Path, freq, 'RefImpedance', 50);
end

% Index of the excited port in the global 'ports' list (Center Antenna)
excited_port_idx = (active_antenna-1)*3 + active_port_on_antenna;

% --- NEW: Define the neighboring port to measure coupling against ---
neighbor_antenna = 5; % First outer hexagonal element
neighbor_port_idx = (neighbor_antenna-1)*3 + active_port_on_antenna;

% Get impedance of the excited port
Zin = ports{excited_port_idx}.uf.tot ./ ports{excited_port_idx}.if.tot;

% Plot impedance
%figure('Position', [100, 100, 1200, 800]);
subplot(2,2,1);
plot(freq/1e6, real(Zin), 'k-', 'LineWidth', 2); hold on;
plot(freq/1e6, imag(Zin), 'r--', 'LineWidth', 2);
grid on;
title(['Input Impedance (Antenna ', num2str(active_antenna), ', Port ', num2str(active_port_on_antenna), ')']);
xlabel('Frequency (MHz)'); ylabel('Z_{in} (\Omega)');
legend('Re', 'Im');

% --- NEW: Compute S-parameters for Return Loss and Mutual Coupling ---
S11 = ports{excited_port_idx}.uf.ref ./ ports{excited_port_idx}.uf.inc;
S_coupling = ports{neighbor_port_idx}.uf.ref ./ ports{excited_port_idx}.uf.inc;

subplot(2,2,2); hold on;
plot(freq/1e6, 20*log10(abs(S11)), 'k-', 'LineWidth', 2); hold on;
plot(freq/1e6, 20*log10(abs(S_coupling)), 'r--', 'LineWidth', 2);
ylim([-60, 0]); % Expanded the Y-axis slightly as coupling may be very low
grid on;
xlabel('Frequency (MHz)'); ylabel('Magnitude (dB)');
title('Return Loss & Mutual Coupling');
legend(['S_{11} (Ant ' num2str(active_antenna) ')'], ...
       ['S_{21} Coupling to Ant ' num2str(neighbor_antenna)]);

% Identify resonance frequency from the return loss of the excited port
[~, f_res_ind] = min(abs(S11));
f_res = freq(f_res_ind);
disp(['Resonant frequency: ', num2str(f_res/1e9), ' GHz']);

% --- Far-field analysis
disp('Calculating 3D far-field pattern...');
thetaRange = -180:2:180;
phiRange = -180:2:180;
nf2ff = CalcNF2FF(nf2ff, Sim_Path, f_res, deg2rad(thetaRange), deg2rad(phiRange));

disp( ['Single-feed radiated power: Prad = ' num2str(nf2ff.Prad) ' Watt']);
disp( ['Single-feed directivity: Dmax = ' num2str(nf2ff.Dmax) ' (' num2str(10*log10(nf2ff.Dmax)) ' dBi)'] );
disp( ['Single-feed efficiency: nu_rad = ' num2str(100*nf2ff.Prad./ports{excited_port_idx}.P_inc(f_res_ind)) ' %']);

% Combine feeds for RHCP
disp('Combining feeds by symmetry for RHCP...');
dphi_deg = rad2deg(nf2ff.phi(2) - nf2ff.phi(1));
phi_shift = round(120 / dphi_deg);
Et1 = nf2ff.E_theta{1}; Ep1 = nf2ff.E_phi{1};
Et2 = circshift(Et1, [0 -phi_shift]); Ep2 = circshift(Ep1, [0 -phi_shift]);
Et3 = circshift(Et1, [0 +phi_shift]); Ep3 = circshift(Ep1, [0 +phi_shift]);
w = [1, exp(1j*2*pi/3), exp(1j*4*pi/3)]; % RHCP phasing
Et_total = w(1)*Et1 + w(2)*Et2 + w(3)*Et3;
Ep_total = w(1)*Ep1 + w(2)*Ep2 + w(3)*Ep3;
nf2ff.E_theta{1} = Et_total; nf2ff.E_phi{1} = Ep_total;
E_total = sqrt(abs(Et_total).^2 + abs(Ep_total).^2);
nf2ff.E_norm = {E_total / max(E_total(:))};

% Plot combined 3D field
subplot(2,2,3);
plotFF3D(nf2ff, 'logscale', -20);
title('Combined RHCP Far-Field (Single Element in Array)');

% --- Co- and Cross-Polar Plots
E_co    = (Et_total + 1j * Ep_total) / sqrt(2);  % RHCP = co-pol
E_cross = (Et_total - 1j * Ep_total) / sqrt(2);  % LHCP = cross-pol
nf_co = nf2ff; nf_cross = nf2ff;
nf_co.E_theta{1} = E_co; nf_co.E_phi{1} = zeros(size(E_co));
nf_co.E_norm = {abs(E_co) / max(abs(E_co(:)))};
nf_cross.E_theta{1} = E_cross; nf_cross.E_phi{1} = zeros(size(E_cross));
nf_cross.E_norm = {abs(E_cross) / max(abs(E_co(:)))};
nf_cross.E_norm{1}(1,:) = 1; % Fix for polarFF normalization

subplot(2,2,4)
polarFF(nf_co, 'logscale', [-30 10], 'xtics', 5, 'xaxis', 'theta', 'phi', 0, 'param', 1, 'linestyle', 'k-',  'linewidth', 2, 'normalize', 0); hold on;
polarFF(nf_cross, 'logscale', [-30 10], 'xtics', 5, 'xaxis', 'theta', 'phi', 0, 'param', 1, 'linestyle', 'k--', 'linewidth', 1.5, 'normalize', 0);
legend('RHCP (co-pol)', 'LHCP (cross-pol)', 'Location', 'southwest');
title('Co- and Cross-Polar Pattern at \phi = 0^\circ');