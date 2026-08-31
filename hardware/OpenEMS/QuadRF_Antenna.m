% ===================================================================================
% 7-Element Hexagonal Array of QuadRF Antennas
%
% Description:
% This script simulates a central patch antenna surrounded by six identical antennas
% in a hexagonal arrangement. Only the central antenna is excited, while the
% surrounding elements are terminated with loads. This setup allows
% for the analysis of the central element's performance / coupling within an array environment.
%
% Copyright 2026 Scale RF Inc.
% ===================================================================================

%% --- Setup Environment
addpath('C:/openEMS/matlab'); % Ensure this path is correct for your system
clear all;
clc;

%% --- Tunable Parameters & Physical Constants
active_antenna = 1; % Index of the antenna to excite (1 = center)
active_port_on_antenna = 1;  % Port on the active antenna to excite (1, 2, or 3)
physical_constants;
unit = 1e-3; % All dimensions in mm

%% --- Antenna Array Configuration
antenna_spacing = 3e8/5.6e9*0.85 * 1e3; % Center-to-center distance between antennas
num_antennas = 7;
% Calculate offsets for the 7 antennas (1st is center, 2-7 are hexagonal)
antenna_offsets = zeros(num_antennas, 2);
hex_angles = (0:5) * pi/3 + pi/6; % 6 angles for 6 positions (0, 60, 120, ...)
for i = 2:num_antennas
    antenna_offsets(i, :) = [antenna_spacing * cos(hex_angles(i-1)), ...
                             antenna_spacing * sin(hex_angles(i-1))];
end

%% --- Single Antenna Geometric Definition
pcb_thickness = 1.0;
gnd_r = 28; % have them merge
patch_r = 11.8; % expanded for PET
patch_h = 4.0;
feed_r = 0.5;
copper_thickness = 0.035;
feed_z1 = patch_h + copper_thickness + 0.1;
% Feed pin positions relative to antenna center
feed_pos_relative = [ 0.000,  4.041;
                     -3.500, -2.021;
                      3.500, -2.021 ];
cutout_r = 1.25;
FR4_epsr = 4.4;
port_resistance = 100; % Ohm

% Calculate microstrip dimensions for the given port resistance
[microstrip.W, eeff] = microstrip_width(port_resistance, FR4_epsr, pcb_thickness);
microstrip.L = c0/5.8e9/sqrt(eeff) /4 * 1e3 ; % Quarter-wavelength at 5.8 GHz
microstrip.dir = [ 0, 1; -1, 0; 1, 0]; % Directions for the 3 microstrips

disp(['Using port resistance ', num2str(port_resistance), ' Ohm']);
disp(['Microstrip width = ', num2str(microstrip.W), ' mm']);
disp(['Microstrip length = ', num2str(microstrip.L), ' mm']);

% Port definition relative to microstrip end
port_w = microstrip.W * 2/3;
port_z_start = 0;
port_z_height = -pcb_thickness;

%% --- Simulation Setup
SimBox_radius = antenna_spacing + 70;
SimBox_height = 100;
SimBox = [SimBox_radius*2, SimBox_radius*2, SimBox_height];

f0 = 5.8e9; % Center frequency
fc = 1e9;   % Bandwidth is 2*fc

FDTD = InitFDTD('NrTS', 1e6, 'EndCriteria', 1e-5);
FDTD = SetGaussExcite(FDTD, f0, fc);
FDTD = SetBoundaryCond(FDTD, {'PML_8', 'PML_8', 'PML_8', 'PML_8', 'PML_8', 'PML_8'});

%% --- CSX Geometry Initialization
CSX = InitCSX();
% Define materials
CSX = AddMaterial(CSX, 'air');
CSX = AddMaterial(CSX, 'FR4');
CSX = SetMaterialProperty(CSX, 'FR4', 'Epsilon', FR4_epsr, 'TanD', 0.018);
% Define metal components
CSX = AddMetal(CSX, 'ground');
CSX = AddMetal(CSX, 'patch');

%CSX = AddMaterial(CSX, 'PET');
%CSX = SetMaterialProperty(CSX, 'PET', 'Epsilon', 3.3, 'TanD', 0.015);

%% --- Build Antenna Array using a Function
ports = {};
port_positions = [];
total_ports = num_antennas * 3;

for i_ant = 1:num_antennas
    offset = antenna_offsets(i_ant, :);
    is_active_antenna = (i_ant == active_antenna);
    
    % Call function to create one antenna at the specified offset
    [CSX, new_ports, new_port_pos] = CreateTripleFeedPatch(CSX, i_ant, offset, ...
        is_active_antenna, active_port_on_antenna, ...
        'feed_pos_relative', feed_pos_relative, 'gnd_r', gnd_r, 'pcb_thickness', pcb_thickness, ...
        'copper_thickness', copper_thickness, 'cutout_r', cutout_r, 'feed_r', feed_r, ...
        'feed_z1', feed_z1, 'microstrip', microstrip, 'port_w', port_w, ...
        'port_z_start', port_z_start, 'port_z_height', port_z_height, ...
        'port_resistance', port_resistance, 'patch_h', patch_h, 'patch_r', patch_r);
        
    ports = [ports, new_ports];
    port_positions = [port_positions; new_port_pos];
end

% Add some ground round nuts

halo_radius = 3e8/5.6e9*0.85 / 2 * 1e3  /  0.866; % match other instance!

angles = (0:6) * pi/3;
pos_nut = [halo_radius * cos(angles); halo_radius * sin(angles)].';

for k=1:6
 

CSX = AddCylinder(CSX, 'ground', 1000, [pos_nut(k,:), 0], [pos_nut(k,:), patch_h+copper_thickness], 5.5/2);

end

%% --- Setup Meshing
max_res = c0 / (f0 + fc) / unit / 20; % ~lambda/20 of highest freq
mesh.x = [];
mesh.y = [];

% Create mesh lines for each antenna in the array
for i_ant = 1:num_antennas
    offset = antenna_offsets(i_ant, :);
    
    % Add lines around feed pins
    feed_x_local = unique([feed_pos_relative(:,1)-feed_r*2/3, feed_pos_relative(:,1), feed_pos_relative(:,1)+feed_r*2/3]);
    feed_y_local = unique([feed_pos_relative(:,2)-feed_r*2/3, feed_pos_relative(:,2), feed_pos_relative(:,2)+feed_r*2/3]);
    
    mesh.x = [mesh.x, feed_x_local' + offset(1)];
    mesh.y = [mesh.y, feed_y_local' + offset(2)];
    
    % Add lines for patch radius
    mesh.x = [mesh.x, -patch_r+offset(1), patch_r+offset(1)];
    mesh.y = [mesh.y, -patch_r+offset(2), patch_r+offset(2)];
    
    % Add lines for ground plane radius
    %mesh.x = [mesh.x, -gnd_r+offset(1), gnd_r+offset(1)];
    %mesh.y = [mesh.y, -gnd_r+offset(2), gnd_r+offset(2)];
end

% Add port edges to the mesh (port edges must be in mesh!)
for i_port = 1:total_ports
    x = port_positions(i_port,1);
    y = port_positions(i_port,2);
    mesh.x = [mesh.x, x, x - port_w/2, x + port_w/2];
    mesh.y = [mesh.y, y, y - port_w/2, y + port_w/2];
end

%-pcb_thickness - max_res/16, -pcb_thickness, -pcb_thickness + max_res/16,   ... % substrate

% Z-mesh definition (only needs to be defined once)
mesh.z = [ -SimBox_height/2, ...
           linspace(0, pcb_thickness, 5),   ... % substrate strangely works better
           0, 0 + 0.2,   ... % substrate
           patch_h - max_res/16, patch_h, ... % patch_h + copper_thickness, ...
           patch_h + 0.8, ... %patch_h + copper_thickness, ...
           2.0, ... % spacer
           port_z_start, port_z_start + port_z_height, ...
           SimBox_height/2 ];

% =================================================================================
% BUG FIX: Round mesh coordinates to a high precision to merge very close points.
% This prevents floating-point inaccuracies from causing the mesh smoothing
% algorithm to request an impossibly large number of mesh lines.
% =================================================================================
precision = 1e-5; % Corresponds to 0.01 micron precision
mesh.x = round(mesh.x / precision) * precision;
mesh.y = round(mesh.y / precision) * precision;

% Finalize mesh
mesh.x = SmoothMeshLines(unique([-SimBox_radius, SimBox_radius, mesh.x]), max_res);
mesh.y = SmoothMeshLines(unique([-SimBox_radius, SimBox_radius, mesh.y]), max_res);
mesh.z = SmoothMeshLines(unique(mesh.z), max_res);
mesh = AddPML(mesh, [8 8 8 8 8 8]);

% Apply mesh to CSX
CSX = DefineRectGrid(CSX, unit, mesh);

%% --- Post-Processing Setup & Simulation Run
Sim_Path = 'triple_feed_patch_array_sim';
Sim_CSX = 'triple_feed_patch_array.xml';

% Near-to-far field box for measurement
nf2ff_box_size = SimBox - 2 * 10 * max_res; % Ensure box is away from PML
[CSX, nf2ff] = CreateNF2FFBox(CSX, 'nf2ff', -nf2ff_box_size/2, nf2ff_box_size/2);

% Write CSX file, create simulation folder, and run simulation
[~,~] = rmdir(Sim_Path, 's');
[~,~] = mkdir(Sim_Path);
WriteOpenEMS([Sim_Path '/' Sim_CSX], FDTD, CSX);
CSXGeomPlot([Sim_Path '/' Sim_CSX]);
RunOpenEMS(Sim_Path, Sim_CSX);

%% === Analysis and Far-Field Postprocessing ===
freq = linspace(max([1e9, f0 - fc]), f0 + fc, 501);

% Calculate all port parameters (only first 3 are from the active antenna)
for n = 1:total_ports
    ports{n} = calcPort(ports{n}, Sim_Path, freq, 'RefImpedance', 50);
end

% Index of the excited port in the global 'ports' list
excited_port_idx = (active_antenna-1)*3 + active_port_on_antenna;

% Get impedance of the excited port
Zin = ports{excited_port_idx}.uf.tot ./ ports{excited_port_idx}.if.tot;

% Plot impedance
figure('Position', [100, 100, 1200, 800]);
subplot(2,2,1);
plot(freq/1e6, real(Zin), 'k-', 'LineWidth', 2); hold on;
plot(freq/1e6, imag(Zin), 'r--', 'LineWidth', 2);
grid on;
title(['Input Impedance (Antenna ', num2str(active_antenna), ', Port ', num2str(active_port_on_antenna), ')']);
xlabel('Frequency (MHz)'); ylabel('Z_{in} (\Omega)');
legend('Re', 'Im');

% Compute and plot S-parameters for the active antenna
s1a = ports{(active_antenna-1)*3 + 1}.uf.ref ./ ports{excited_port_idx}.uf.inc;
s2a = ports{(active_antenna-1)*3 + 2}.uf.ref ./ ports{excited_port_idx}.uf.inc;
s3a = ports{(active_antenna-1)*3 + 3}.uf.ref ./ ports{excited_port_idx}.uf.inc;

subplot(2,2,2);
plot(freq/1e6, 20*log10(abs(s1a)), 'k-', 'LineWidth', 2); hold on;
plot(freq/1e6, 20*log10(abs(s2a)), 'b--', 'LineWidth', 2);
plot(freq/1e6, 20*log10(abs(s3a)), 'g--', 'LineWidth', 2);
ylim([-40, 0]);
grid on;
xlabel('Frequency (MHz)'); ylabel('|S_{ia}| (dB)');
title(sprintf('S-parameters for Active Antenna (a=%d)', active_port_on_antenna));
legend('|S_{1a}|','|S_{2a}|','|S_{3a}|');

% Identify resonance frequency from the return loss of the excited port
[~, f_res_ind] = min(abs(s1a));
f_res = freq(f_res_ind);
disp(['Resonant frequency: ', num2str(f_res/1e9), ' GHz']);

% --- Far-field analysis (unchanged, uses the single excited run)
disp('Calculating 3D far-field pattern...');
thetaRange = -180:2:180;
phiRange = -180:2:180;
nf2ff = CalcNF2FF(nf2ff, Sim_Path, f_res, deg2rad(thetaRange), deg2rad(phiRange));

% =================================================================================
% ADDED BACK: Display power, directivity, and efficiency
% =================================================================================
disp( ['Single-feed radiated power: Prad = ' num2str(nf2ff.Prad) ' Watt']);
disp( ['Single-feed directivity: Dmax = ' num2str(nf2ff.Dmax) ' (' num2str(10*log10(nf2ff.Dmax)) ' dBi)'] );
disp( ['Single-feed efficiency: nu_rad = ' num2str(100*nf2ff.Prad./ports{excited_port_idx}.P_inc(f_res_ind)) ' %']);


% Combine feeds for RHCP (logic remains the same)
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


%% --- Reusable Function to Create a Single Antenna ---
function [CSX, ports, port_pos] = CreateTripleFeedPatch(CSX, ant_idx, offset, ...
    is_active_antenna, active_port_on_antenna, varargin)
    
    % --- Parse Input Parameters
    p = inputParser;
    addParameter(p, 'feed_pos_relative', [0 0; -3 -2; 3 -2]);
    addParameter(p, 'gnd_r', 21.4);
    addParameter(p, 'pcb_thickness', 1.0);
    addParameter(p, 'copper_thickness', 0.035);
    addParameter(p, 'cutout_r', 1.3);
    addParameter(p, 'feed_r', 0.3);
    addParameter(p, 'feed_z1', 4.135);
    addParameter(p, 'microstrip', struct('W', 1, 'L', 10, 'dir', [0 1; -1 0; 1 0]));
    addParameter(p, 'port_w', 1);
    addParameter(p, 'port_z_start', 0);
    addParameter(p, 'port_z_height', -1);
    addParameter(p, 'port_resistance', 100);
    addParameter(p, 'patch_h', 4.0);
    addParameter(p, 'patch_r', 11.8);
    parse(p, varargin{:});
    
    % Extract parameters into local variables for clarity
    args = p.Results;
    
    % --- Initialize Outputs
    ports = cell(1, 3);
    port_pos = zeros(3, 2);

 
    % hex parameters
    patch_fr4_r = args.patch_r + 0.2;
    patch_fr4_h = 0.8; % back to FR4
    power_width = 5.5;% 6.81; % 4 gap
    halo_radius = 3e8/5.6e9*0.85 / 2 * 1e3  /  0.866; 
    inner_hex_radius = halo_radius - power_width/2 / 0.866;
    
    angles = (0:6) * pi/3;
    outer_hex_points = [halo_radius * cos(angles) + offset(1); halo_radius * sin(angles) + offset(2)];
    inner_hex_points = [inner_hex_radius * cos(angles) + offset(1); inner_hex_radius * sin(angles) + offset(2)];

    % --- Add Geometry for one antenna at the specified offset
    
    % 1. Substrate
    %CSX = AddCylinder(CSX, 'FR4', 100*ant_idx+0, [offset, 0], [offset, -args.pcb_thickness], args.gnd_r);
    
    % 2. Ground spacer
    %CSX = AddCylinder(CSX, 'ground', 100*ant_idx+1, [offset, 0], [offset, args.patch_h], args.gnd_r); % all around ground spacer now
    CSX = AddLinPoly(CSX, 'FR4',   100*ant_idx+1, 'z', -args.pcb_thickness, outer_hex_points, args.pcb_thickness + args.patch_h + patch_fr4_h);
    CSX = AddLinPoly(CSX, 'ground',   100*ant_idx+2, 'z', 0.0, outer_hex_points, 2.0+args.copper_thickness); % 12V
    CSX = AddLinPoly(CSX, 'ground',   100*ant_idx+3, 'z', args.patch_h, outer_hex_points, args.copper_thickness); % 12V on antenna bottom
    CSX = AddLinPoly(CSX, 'ground',   100*ant_idx+3, 'z', args.patch_h+ patch_fr4_h, outer_hex_points, args.copper_thickness); % 12V on antenna top

    % hexagonal FR4 top power that connects to patch
    CSX = AddLinPoly(CSX, 'air',   100*ant_idx+4, 'z', 0, inner_hex_points, args.patch_h+patch_fr4_h  + args.copper_thickness); % now cutout ground spacer and top fr4
    %CSX = AddLinPoly(CSX, 'air',   100*ant_idx+4, 'z', 2.0+args.copper_thickness, outer_hex_points, args.patch_h-2.0-args.copper_thickness); % now cutout ground spacer and top fr4

    % add back a basic ground plane
    %CSX = AddCylinder(CSX, 'ground', 100*ant_idx+4, [offset, 0], [offset, args.copper_thickness], args.gnd_r); % base ground plane
    CSX = AddLinPoly(CSX, 'ground',   100*ant_idx+5, 'z', 0, outer_hex_points, args.copper_thickness);

    % cutout vias in ground
    for n = 1:3
        fpos = args.feed_pos_relative(n,:) + offset;
        CSX = AddCylinder(CSX, 'air', 100*ant_idx+6, [fpos, 0], [fpos, args.copper_thickness], args.cutout_r);
    end
    
    % 3. Feed wires, microstrips, and ports
    for n = 1:3
        fpos = args.feed_pos_relative(n,:) + offset;
        mdir = args.microstrip.dir(n,:);
        
        % Vertical feed wire
        CSX = AddCylinder(CSX, 'patch', 100*ant_idx+7, [fpos, -args.pcb_thickness-0.1], [fpos, args.feed_z1], args.feed_r);
        CSX = AddCylinder(CSX, 'patch', 100*ant_idx+8, [fpos, -args.pcb_thickness-args.copper_thickness], [fpos, -args.pcb_thickness], 0.8); % via ring
        
        % Microstrip
        start = [fpos - mdir*args.feed_r + (~mdir * -args.microstrip.W/2), -args.pcb_thickness-args.copper_thickness];
        stop  = [fpos + mdir*args.microstrip.L + (~mdir * +args.microstrip.W/2), -args.pcb_thickness];
        CSX = AddBox(CSX, 'patch', 100*ant_idx+9, start, stop);
        
        % Port definition
        ppos = fpos + mdir*args.microstrip.L;
        start = [ppos - [args.port_w/2, args.port_w/2], args.port_z_start];
        stop  = [ppos + [args.port_w/2, args.port_w/2], args.port_z_start + args.port_z_height];
        port_pos(n,:) = ppos;
        
        % Add port (only excite if it's the active antenna and correct port)
        excite_flag = (is_active_antenna && (n == active_port_on_antenna));
        port_number = (ant_idx-1)*3 + n;
        [CSX, ports{n}] = AddLumpedPort(CSX, port_number, port_number, args.port_resistance, start, stop, [0 0 1], excite_flag);
    end
    
    % 4. Patch Disk and Halo
    
    
    %CSX = AddLinPoly(CSX, 'ground', 100*ant_idx+7, 'z', 0, outer_hex_points, args.patch_h); % all around ground spacer
    %CSX = AddLinPoly(CSX, 'ground', 100*ant_idx+8, 'z', args.patch_h + patch_fr4_h, outer_hex_points, args.copper_thickness);

    % Add actual patch disk
    CSX = AddCylinder(CSX, 'FR4',   100*ant_idx+10, [offset, args.patch_h], [offset, args.patch_h + patch_fr4_h], patch_fr4_r);
    % need to remove FR4 if doing flex
    CSX = AddCylinder(CSX, 'patch', 100*ant_idx+11, [offset, args.patch_h], [offset, args.patch_h + args.copper_thickness], args.patch_r);
    


    % =================================================================================
    % 5. Add connector FR4 straight out
    % =================================================================================

    for n = 1:6
        connector_w = 2.0;
        
        theta = 2*pi/6 * n + pi/6;
        % Define the start and end points for the radial connector
        start_pos_rel = args.feed_pos_relative(1,:) * [cos(theta), sin(theta); -sin(theta), cos(theta)]*2.9;
        
        % Calculate the radial direction vector from the center to the feed
        if norm(start_pos_rel) < 1e-9 % Handle the central feed point case
            radial_dir = [0, 1]; % Define it as pointing 'up' for the top feed
        else
            radial_dir = start_pos_rel / norm(start_pos_rel);
        end
        
        % Calculate the tangential vector (perpendicular to radial)
        tangent_dir = [-radial_dir(2), radial_dir(1)];
        
        % Define the outer edge of the connector
        end_pos_rel = radial_dir * (args.gnd_r - 4.04);
        
        % Calculate the 4 vertices of the polygon
        v1 = start_pos_rel - tangent_dir * connector_w / 2;
        v2 = end_pos_rel   - tangent_dir * connector_w / 2;
        v3 = end_pos_rel   + tangent_dir * connector_w / 2;
        v4 = start_pos_rel + tangent_dir * connector_w / 2;
        
        % Assemble the points into a matrix and apply the antenna's offset
        poly_points = [v1' + offset', v2' + offset', v3' + offset', v4' + offset'];
        
        % Define the z-start and height for the polygon
        z_start = args.patch_h + args.copper_thickness;
        height = patch_fr4_h - args.copper_thickness;
        
        % Add the polygon to the geometry
        CSX = AddLinPoly(CSX, 'FR4', 100*ant_idx+12+n, 'z', z_start, poly_points, height);
    end

end
