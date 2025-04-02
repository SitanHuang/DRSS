
% Author: Sitan Huang
% Wind turbulence added 4/1/2025

function w = wind_calc(sys, wr, zr, z, xi, t, low_wind, high_wind, turbulence, freq)
    % Set optional arguments if not provided:
    if nargin < 5, t = 0; end
    if nargin < 6, low_wind = wr; end
    if nargin < 7, high_wind = 1.2 * wr; end
    if nargin < 8, turbulence = 0; end
    if nargin < 9, freq = 2; end

    % For negative heights, simply return the reference wind speed.
    if z < 0
        w = wr;
        return;
    end

    % Base wind speed following boundary layer profile.
    w_base = wr * ((z + zr) / zr)^(1 / xi);

    % If any turbulence-related parameter is zero or NaN, just use the base value.
    if isnan(low_wind) || isnan(high_wind) || isnan(turbulence) || isnan(freq) || ...
       freq == 0 || turbulence == 0
        w = w_base;
        return;
    end

    % --- Turbulence generation using filtered, asymmetrical white noise ---
    if isempty(sys.windModelFunc)
      sys.windModelFunc = DRSS.util.genAsymmetricalTurbulence(w_base, low_wind, high_wind, freq, max(sys.timeSpan), 1 / freq / 10);
    end

    w = sys.windModelFunc(t);
end
