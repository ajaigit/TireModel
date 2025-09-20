function [Fx, Fy, Mz] = tire_model(kappa, alpha, gamma, Fz, Vx, mu_scale, K_scale, P)
%#codegen
% Combined-slip tire model with camber & load sensitivity (SAE J670).
% Fx (+X), Fy (+Y left), Mz right-hand rule. alpha>0 => Fy>0; kappa>0 tractive; gamma>0 pos camber.

% ---------- Guard & sanitize ----------
if ~isfinite(kappa); kappa = 0; end
if ~isfinite(alpha); alpha = 0; end
if ~isfinite(gamma); gamma = 0; end
if ~isfinite(Fz)   ; Fz    = 0; end
if ~isfinite(Vx)   ; Vx    = 0; end
if ~isfinite(mu_scale) || mu_scale < 0; mu_scale = 0; end
if ~isfinite(K_scale)  || K_scale  < 0; K_scale  = 0; end
Fz = max(Fz,0.0);

% ---------- Params with codegen-safe defaults ----------
if ~isstruct(P); P = struct; end

% Reference load & friction
Fz0    = 4000.0;  if isfield(P,'Fz0'),     Fz0    = P.Fz0;     end
mu_x0  = 1.0;     if isfield(P,'mu_x0'),   mu_x0  = P.mu_x0;   end
mu_y0  = 1.0;     if isfield(P,'mu_y0'),   mu_y0  = P.mu_y0;   end
px     = 0.0;     if isfield(P,'px'),      px     = P.px;      end
py     = 0.0;     if isfield(P,'py'),      py     = P.py;      end

% Cornering/long stiffness (per unit slip / per rad)
Kx0    = 8e4;     if isfield(P,'Kx0'),     Kx0    = P.Kx0;     end
Ky0    = 1.2e5;   if isfield(P,'Ky0'),     Ky0    = P.Ky0;     end
sx     = 1.0;     if isfield(P,'sx'),      sx     = P.sx;      end
sy     = 1.0;     if isfield(P,'sy'),      sy     = P.sy;      end

% Camber & trail
cg     = 0.1;     if isfield(P,'camber_gain'),     cg  = P.camber_gain;     end
t0     = 0.08;    if isfield(P,'trail0'),          t0  = P.trail0;          end
ct     = 6.0;     if isfield(P,'trail_decay'),     ct  = P.trail_decay;     end
st     = 0.3;     if isfield(P,'trail_load_exp'),  st  = P.trail_load_exp;  end

% Hygiene / blending
v_eps  = 0.2;     if isfield(P,'v_eps'),   v_eps   = P.v_eps;   end
fz_eps = 50.0;    if isfield(P,'fz_eps'),  fz_eps  = P.fz_eps;  end

% Friction ellipse weighting
ell_x  = 1.0;     if isfield(P,'ellipse_x'), ell_x = P.ellipse_x; end
ell_y  = 1.0;     if isfield(P,'ellipse_y'), ell_y = P.ellipse_y; end

% ---------- Load sensitivity ----------
fz_ratio = 0.0;
if Fz0 > 0, fz_ratio = Fz / Fz0; end
mu_x = mu_x0 * (fz_ratio^px) * mu_scale;
mu_y = mu_y0 * (fz_ratio^py) * mu_scale;
Kx   = Kx0   * (fz_ratio^sx) * K_scale;
Ky   = Ky0   * (fz_ratio^sy) * K_scale;

% ---------- Base (uncoupled) forces (tanh saturation) ----------
Fx0 = mu_x * Fz * tanh( safe_div(Kx * kappa, max(mu_x * Fz, 1.0)) );

alpha_eff = alpha + cg * gamma; % camber as equivalent slip-angle shift
Fy0 = mu_y * Fz * tanh( safe_div(Ky * alpha_eff, max(mu_y * Fz, 1.0)) );

% ---------- Combined-slip via friction ellipse scaling ----------
Fx_lim = mu_x * Fz * ell_x;
Fy_lim = mu_y * Fz * ell_y;

nx = safe_div(Fx0, max(Fx_lim, 1.0));
ny = safe_div(Fy0, max(Fy_lim, 1.0));
den = hypot(nx, ny);

scale = 1.0;
if den > 1.0
    scale = 1.0 / den; % vector scale to ellipse boundary
end
Fx = Fx0 * scale;
Fy = Fy0 * scale;

% ---------- Low-speed & low-load taper ----------
sv = smoothstep( abs(Vx) / v_eps );  % 0 at Vx=0; ->1 by ~v_eps
sf = smoothstep( Fz / fz_eps );      % 0 at Fz=0; ->1 by ~fz_eps
Fx = Fx * sv * sf;
Fy = Fy * sv * sf;

% ---------- Aligning moment ----------
trail = t0 * exp(-ct * abs(alpha_eff)) * (fz_ratio^st);
Mz = -trail * Fy;

% ---------- Cleanup ----------
if abs(Fx) < 1e-12, Fx = 0.0; end
if abs(Fy) < 1e-12, Fy = 0.0; end
if abs(Mz) < 1e-12, Mz = 0.0; end
end

% ===== helpers (coder-safe) =====
function y = smoothstep(x)
% 0..1 C2 blend; saturates outside [0,1]
if ~isfinite(x); x = 0; end
if x <= 0
    y = 0.0;
elseif x >= 1
    y = 1.0;
else
    y = 3*x*x - 2*x*x*x;
end
end

function y = safe_div(a,b)
% Sign-preserving division with nonzero floor on |b|
epsb = 1e-9;
sgn  = double(b>=0) - double(b<0);        % +1 for b>=0, -1 for b<0
den  = sgn .* max(abs(b), epsb);          % never 0, preserves sign
y    = a ./ den;
end
