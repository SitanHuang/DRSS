
% Author: Sitan Huang
% Wind turbulence added 4/1/2025
% Aerological model added 4/9/2025

function w = wind_calc(sys, wr, zr, z, xi, t, low_wind, high_wind, turbulence, freq)
  % Set optional arguments if not provided:
  if nargin < 5, t = 0; end
  if nargin < 6, low_wind = wr; end
  if nargin < 7, high_wind = 1.2 * wr; end
  if nargin < 8, turbulence = 0; end
  if nargin < 9, freq = 2; end
  
  % --- Base Wind Speed Calculation ---
  % If wr is a scalar, use the power law boundary layer model.
  % Otherwise, assume wr is an [N x 2] matrix with [altitude, windSpeed] pairs,
  % and use linear interpolation to determine the base wind speed at altitude z.
  if isscalar(wr)
    if z < 0
      w = wr;
      return;
    end
  
    w_base = wr * ((z + zr) / zr)^(1 / xi);
  else
    % Perform linear interpolation based on altitude.
    % 'extrap' ensures values outside the provided range are extrapolated.
    w_base = interp1(wr(:,1), wr(:,2), z, 'linear', 'extrap');
  end
  
  % If any turbulence-related parameter is zero or NaN, just use the base value.
  if isnan(low_wind) || isnan(high_wind) || isnan(turbulence) || isnan(freq) || ...
      freq == 0 || turbulence == 0
    w = w_base;
    return;
  end
  
  % --- Turbulence generation using filtered, asymmetrical white noise ---
  if isempty(sys.windModelFunc)
    turbulentModel = DRSS.util.genAsymmetricalTurbulence(w_base, low_wind, high_wind, freq, max(sys.timeSpan), turbulence, 1 / freq / 10);

    sys.windModelFunc = @(t, w_base_t) windModelFunc(turbulentModel, t, w_base, w_base_t);
  end
  
  w = sys.windModelFunc(t, w_base);
end

function w = windModelFunc(turbulentModel, t, w_base0, w_base_t)
  w = turbulentModel(t) * w_base_t / w_base0;
end