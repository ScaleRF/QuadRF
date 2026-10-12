% Microstrip calculations

function [Z0, eeff] = microstrip_impedance(er, h, w)
%MICROSTRIP_IMPEDANCE Calculates the impedance of a microstrip line.
%
%   Z0 = microstrip_impedance(er, h, w)
%
%   Inputs:
%       er - relative dielectric constant of substrate (unitless)
%       h  - height above ground plane in mm
%       w  - width of the trace in mm
%
%   Output:
%       Z0 - characteristic impedance in ohms (Ω)
%
%   Assumes copper thickness t = 0.035 mm (1 oz copper)
%

t = 0.035; % trace thickness in mm

% Effective width including thickness correction (Hammerstad and Jensen)
if w/h <= 1
    F = (t/pi) * log(1 + (4*exp(1)*h)/(t * (1/cosh(sqrt(6.517*w/h)))));
else
    F = (t/pi) * log(1 + (2*h)/(t));
end

weff = w + F;

% Effective dielectric constant
eeff = (er + 1)/2 + (er - 1)/2 * (1 / sqrt(1 + 12*h/weff));

% Characteristic impedance
Z0 = (60 / sqrt(eeff)) * log(8*h/weff + 0.25*weff/h);

end
