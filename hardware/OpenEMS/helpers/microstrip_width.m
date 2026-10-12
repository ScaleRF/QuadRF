function [w, eeff] = microstrip_width(Z0, er, h)
%MICROSTRIP_WIDTH_CLOSEDFORM Calculates microstrip width for given impedance
% and returns effective dielectric constant.
%
%   [w, eeff] = microstrip_width_closedform(Z0, er, h)
%
%   Inputs:
%       Z0  - desired characteristic impedance (ohms)
%       er  - relative permittivity of substrate
%       h   - substrate height (mm)
%
%   Outputs:
%       w    - required microstrip trace width (mm)
%       eeff - effective dielectric constant
%
%   Assumes copper thickness t = 0.035 mm (1 oz copper)
%   Valid for Z0 approximately in [30, 130] ohms

t = 0.035;  % trace thickness in mm

% Sanity check
if Z0 < 30 || Z0 > 130
    warning('Z0 = %.1f ohms is outside the valid range [30, 130] for this formula. Results may be inaccurate.', Z0);
end

% Intermediate expressions from Hammerstad-Jensen model
A = Z0 / 60 * sqrt((er + 1)/2) + (er - 1)/(er + 1)*(0.23 + 0.11/er);
B = (377 * pi) / (2 * Z0 * sqrt(er));

% Initial width/height ratio estimate
if Z0 <= 120 * pi / sqrt(er)
    w_h = (8 * exp(A)) / (exp(2*A) - 2);
else
    w_h = (2 / pi) * (B - 1 - log(2*B - 1) + ...
        ((er - 1)/(2*er)) * (log(B - 1) + 0.39 - 0.61/er));
end

% Thickness correction (delta_w)
delta_w = (t/pi) * log(1 + (4 * exp(1) * h) / (t * (1/cosh(sqrt(6.517 * w_h)))));
w = h * w_h - delta_w;

% Sanity check
if w <= 0
    error('Computed width is invalid. Try using a numeric solver instead.');
end

% Effective dielectric constant
w_eff = w + delta_w;
eeff = (er + 1)/2 + (er - 1)/2 * (1 / sqrt(1 + 12 * h / w_eff));

end