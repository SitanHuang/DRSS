function [windFunction, t, windSpeed] = genAsymmetricalTurbulence( ...
    baseWind, lowWind, highWind, approxFreq, totalTime, dt)
% genAsymmetricalTurbulence
%
% Generates a gusting wind signal that:
%   (a) Has a specified baseline (baseWind).
%   (b) Goes down to lowWind and up to highWind.
%   (c) Distributes wind speed asymmetrically around the baseline if
%       (baseWind - lowWind) != (highWind - baseWind).
%   (d) Smooths out random variations via low-pass filtering.
%
% INPUTS:
%   baseWind  - "Nominal" or "baseline" wind speed (e.g. 14).
%   lowWind   - Minimum wind speed (e.g. 8).
%   highWind  - Maximum wind speed (e.g. 34).
%   seed      - RNG seed for reproducibility (integer).
%   approxFreq- Approximate cutoff frequency in Hz for gust variations.
%   totalTime - Total simulation duration in seconds.
%   dt        - Time step in seconds.
%
% OUTPUTS:
%   windFunction - A function handle windFunction(tQuery) that returns
%                  wind speed at the queried time (via interpolation).
%   t            - The time vector (0 : dt : totalTime).
%   windSpeed    - The generated wind-speed array corresponding to t.
%
% Example:
%   [fWind, t, w] = generateAsymmetricGustingWind(14, 8, 34, 123, 0.05, 600, 0.5);
%   figure; plot(t, w); xlabel('Time (s)'); ylabel('Wind Speed (m/s)');
%   title('Asymmetric Gusting Wind');
%

    % 1) Set up time vector
    t = 0:dt:(2*totalTime); 
    N = length(t);
    Fs = 1/dt; % sampling frequency
    % 2) Seed the RNG & generate raw uniform noise
    rng('shuffle');

    rawNoise = 2 * rand(N, 1) - 1;  % [-1, 1]

    % 3) Low-pass filter design
    %    We use a 2nd-order Butterworth for "smooth" gusting.
    cutoff = approxFreq / (Fs/2); 
    % clamp cutoff to valid range (0 < cutoff < 1)
    cutoff = max(min(cutoff, 0.99), 0.001);
    [b, a] = butter(4, cutoff, 'low');
    
    % 4) Filter the noise forward-backward for zero phase shift
    filteredX = filtfilt(b, a, rawNoise);

    % 5) Re-clamp
    filteredX = max(min(filteredX, 3), -3);

    % 6) Map X in [-1,1] to wind speed in [lowWind, highWind], with
    %    baseWind corresponding to X=0.  We do a *piecewise* linear map:
    %
    %    X = -1  ->  wind = lowWind
    %    X = 0   ->  wind = baseWind
    %    X = +1  ->  wind = highWind
    %
    %    For X in [-1, 0], we interpolate linearly between lowWind and baseWind.
    %    For X in [0,  1], we interpolate linearly between baseWind and highWind.
    
    windSpeed = zeros(size(filteredX));

    % Indices where X <= 0  => map to [lowWind, baseWind]
    idxLow = (filteredX <= 0);
    Xneg = filteredX(idxLow);
    % fraction from -1 to 0
    % -1 -> lowWind, 0 -> baseWind
    windSpeed(idxLow) = lowWind + (baseWind - lowWind) * ((Xneg - (-1)) / (0 - (-1)));
    % This is lowWind + (baseWind - lowWind)*((Xneg + 1)/1)

    % Indices where X > 0 => map to [baseWind, highWind]
    idxHigh = (filteredX > 0);
    Xpos = filteredX(idxHigh);
    % fraction from 0 to +1
    % 0 -> baseWind, +1 -> highWind
    windSpeed(idxHigh) = baseWind + (highWind - baseWind) * (Xpos / 1);

    % 7) Build an interpolation function handle
    tOffset = rand() * totalTime;
    windFunction = @(tQuery) interp1(t, windSpeed, tQuery + tOffset, 'pchip', 'extrap');

    % figure;
    % plot(t, windSpeed);
    % xlabel('Time (s)');
    % ylabel('Wind Speed (m/s)');
    % title('Asymmetric Gusting Wind Simulation');
    % grid on;

end
