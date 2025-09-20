function P = params_default()
% PARAMS_DEFAULT  Default parameters for the simple combined-slip tire.
% Units consistent with SAE J670 (N, m, rad).
P.Fz0            = 4000;     % N
P.mu_x0          = 1.00;
P.mu_y0          = 1.00;
P.px             = 0.02;     % mild load sensitivity exponents
P.py             = 0.02;

P.Kx0            = 8.0e4;    % N (per unit slip)
P.Ky0            = 1.2e5;    % N/rad
P.sx             = 1.00;
P.sy             = 1.00;

P.camber_gain    = 0.10;     % [1/rad]
P.trail0         = 0.08;     % m
P.trail_decay    = 6.0;      % 1/rad
P.trail_load_exp = 0.30;

P.v_eps          = 0.2;      % m/s (low-speed taper)
P.fz_eps         = 50.0;     % N   (low-load taper)

P.ellipse_x      = 1.0;
P.ellipse_y      = 1.0;
end
