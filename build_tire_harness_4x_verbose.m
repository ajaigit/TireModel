function build_tire_harness_4x_verbose()
% BUILD_TIRE_HARNESS_4X_VERBOSE
% - Uses your mdl name ('tire_harness'), wrapper pattern, and solver settings
% - Creates a TireCorner subsystem, instantiates FL/FR/RL/RR
% - Adds Steering (alpha sine) & Braking (kappa step) scenarios with selector
% - Prints after every add_block and add_line for interactive progress

t0 = tic;
mdl = 'tire_harness';
box = @(x,y,w,h) [x y x+w y+h];

% -------------------------------------------------------------------------
% New model
% -------------------------------------------------------------------------
if bdIsLoaded(mdl)
    close_system(mdl,0);
    fprintf('[%6.2fs] Closed existing model: %s\n', toc(t0), mdl);
end
new_system(mdl);
open_system(mdl);
fprintf('[%6.2fs] Created and opened model: %s\n', toc(t0), mdl);

% -------------------------------------------------------------------------
% TireCorner subsystem
% -------------------------------------------------------------------------
parentSub = [mdl '/TireCorner'];
add_block('simulink/Ports & Subsystems/Subsystem', parentSub, ...
    'Position', box(360, 80, 580, 220));
fprintf('[%6.2fs] Added subsystem: %s\n', toc(t0), parentSub);
open_system(parentSub);

% Inports (order must match tire_model wrapper):
inNames = {'kappa','alpha','gamma','Fz','Vx','mu_scale','K_scale'};
for i = 1:numel(inNames)
    blk = [parentSub '/' inNames{i}];
    add_block('simulink/Ports & Subsystems/In1', blk, ...
        'Position', [30 30+40*(i-1) 60 50+40*(i-1)]);
    fprintf('[%6.2fs]  + Inport: %s\n', toc(t0), blk);
end

% MATLAB Function (wrapper) inside TireCorner
mf = [parentSub '/TireFcn'];
add_block('simulink/User-Defined Functions/MATLAB Function', mf, ...
    'Position', [140 60 290 160]);
fprintf('[%6.2fs]  + MATLAB Function: %s\n', toc(t0), mf);

% Inject wrapper code (your exact pattern)
code = [ ...
'function [Fx, Fy, Mz] = TireFcn(kappa, alpha, gamma, Fz, Vx, mu_scale, K_scale)', newline, ...
'%#codegen', newline, ...
'persistent Pconst', newline, ...
'if isempty(Pconst)', newline, ...
'    Pconst = params_default();', newline, ...
'end', newline, ...
'[Fx, Fy, Mz] = tire_model(kappa, alpha, gamma, Fz, Vx, mu_scale, K_scale, Pconst);', newline, ...
'end' ];
rt = sfroot;
m  = rt.find('-isa','Stateflow.Machine','Name',mdl);
ch = m.find('-isa','Stateflow.EMChart','Path',mf);
ch.Script = code;
fprintf('[%6.2fs]  + Injected wrapper code into: %s\n', toc(t0), mf);

% Wire TireCorner Inports -> MATLAB Function
for i = 1:numel(inNames)
    add_line(parentSub, [inNames{i} '/1'], ['TireFcn/' num2str(i)], 'autorouting','on');
    fprintf('[%6.2fs]    wired: %s -> TireFcn/%d\n', toc(t0), inNames{i}, i);
end

% Outports: Fx, Fy, Mz
outNames = {'Fx','Fy','Mz'};
for i = 1:3
    blk = [parentSub '/' outNames{i}];
    add_block('simulink/Ports & Subsystems/Out1', blk, ...
        'Position', [340 60+40*(i-1) 370 80+40*(i-1)]);
    fprintf('[%6.2fs]  + Outport: %s\n', toc(t0), blk);
    add_line(parentSub, ['TireFcn/' num2str(i)], [outNames{i} '/1'], 'autorouting','on');
    fprintf('[%6.2fs]    wired: TireFcn/%d -> %s\n', toc(t0), i, outNames{i});
end

set_param(parentSub,'Open','off');
fprintf('[%6.2fs] Closed internals of: %s\n', toc(t0), parentSub);

% -------------------------------------------------------------------------
% Shared sources (your settings)
% -------------------------------------------------------------------------
add_block('simulink/Sources/Constant', [mdl '/Vx'], 'Value','20', ...
    'Position', box(80,120,110,140));
fprintf('[%6.2fs] Added Constant: Vx = 20\n', toc(t0));
add_block('simulink/Sources/Constant', [mdl '/mu_scale'], 'Value','1', ...
    'Position', box(80,160,110,180));
fprintf('[%6.2fs] Added Constant: mu_scale = 1\n', toc(t0));
add_block('simulink/Sources/Constant', [mdl '/K_scale'], 'Value','1', ...
    'Position', box(80,200,110,220));
fprintf('[%6.2fs] Added Constant: K_scale = 1\n', toc(t0));

% Scenario selector: 1=Steering, 0=Braking
add_block('simulink/Sources/Constant', [mdl '/ScenarioSel'], 'Value','1', ...
    'Position', box(80,260,110,280));
fprintf('[%6.2fs] Added Constant: ScenarioSel = 1 (Steering)\n', toc(t0));

% -------------------------------------------------------------------------
% Scenario signals
% -------------------------------------------------------------------------
% Braking: kappa step, alpha=0
add_block('simulink/Sources/Step', [mdl '/kappa_brk'], ...
    'Time','1','Before','0','After','-0.1', ...
    'Position', box(160,340,190,360));
fprintf('[%6.2fs] Added Step: kappa_brk (0 -> -0.1 at t=1s)\n', toc(t0));

add_block('simulink/Sources/Constant', [mdl '/alpha_brk'], 'Value','0', ...
    'Position', box(160,380,190,400));
fprintf('[%6.2fs] Added Constant: alpha_brk = 0\n', toc(t0));

% Steering: alpha sine (±5 deg), kappa ~ 0
add_block('simulink/Sources/Sine Wave', [mdl '/alpha_str'], ...
    'Amplitude','deg2rad(5)', 'Frequency','2*pi*0.5', 'Bias','0', 'Phase','0', ...
    'Position', box(160,420,220,440));
fprintf('[%6.2fs] Added Sine: alpha_str (±5 deg @ 0.5 Hz)\n', toc(t0));

add_block('simulink/Sources/Constant', [mdl '/kappa_str'], 'Value','0', ...
    'Position', box(160,460,190,480));
fprintf('[%6.2fs] Added Constant: kappa_str = 0\n', toc(t0));

% Switches
add_block('simulink/Signal Routing/Switch', [mdl '/SW_kappa'], ...
    'Criteria','u2 ~= 0','Position', box(250,350,290,390));
fprintf('[%6.2fs] Added Switch: SW_kappa (ScenarioSel)\n', toc(t0));

add_block('simulink/Signal Routing/Switch', [mdl '/SW_alpha'], ...
    'Criteria','u2 ~= 0','Position', box(250,430,290,470));
fprintf('[%6.2fs] Added Switch: SW_alpha (ScenarioSel)\n', toc(t0));

% Wire scenario selection
add_line(mdl,'kappa_str/1','SW_kappa/1','autorouting','on');
fprintf('[%6.2fs] wired: kappa_str -> SW_kappa/1\n', toc(t0));
add_line(mdl,'kappa_brk/1','SW_kappa/3','autorouting','on');
fprintf('[%6.2fs] wired: kappa_brk -> SW_kappa/3\n', toc(t0));
add_line(mdl,'ScenarioSel/1','SW_kappa/2','autorouting','on');
fprintf('[%6.2fs] wired: ScenarioSel -> SW_kappa/2\n', toc(t0));

add_line(mdl,'alpha_str/1','SW_alpha/1','autorouting','on');
fprintf('[%6.2fs] wired: alpha_str -> SW_alpha/1\n', toc(t0));
add_line(mdl,'alpha_brk/1','SW_alpha/3','autorouting','on');
fprintf('[%6.2fs] wired: alpha_brk -> SW_alpha/3\n', toc(t0));
add_line(mdl,'ScenarioSel/1','SW_alpha/2','autorouting','on');
fprintf('[%6.2fs] wired: ScenarioSel -> SW_alpha/2\n', toc(t0));

% -------------------------------------------------------------------------
% Per-corner constants
% -------------------------------------------------------------------------
add_block('simulink/Sources/Constant', [mdl '/Fz_FL'], 'Value','3500', ...
    'Position', box(80,540,110,560)); fprintf('[%6.2fs] Added Fz_FL = 3500\n', toc(t0));
add_block('simulink/Sources/Constant', [mdl '/Fz_FR'], 'Value','3500', ...
    'Position', box(80,580,110,600)); fprintf('[%6.2fs] Added Fz_FR = 3500\n', toc(t0));
add_block('simulink/Sources/Constant', [mdl '/Fz_RL'], 'Value','3000', ...
    'Position', box(80,620,110,640)); fprintf('[%6.2fs] Added Fz_RL = 3000\n', toc(t0));
add_block('simulink/Sources/Constant', [mdl '/Fz_RR'], 'Value','3000', ...
    'Position', box(80,660,110,680)); fprintf('[%6.2fs] Added Fz_RR = 3000\n', toc(t0));

add_block('simulink/Sources/Constant', [mdl '/gamma_FL'], 'Value','0', ...
    'Position', box(80,720,110,740)); fprintf('[%6.2fs] Added gamma_FL = 0\n', toc(t0));
add_block('simulink/Sources/Constant', [mdl '/gamma_FR'], 'Value','0', ...
    'Position', box(80,760,110,780)); fprintf('[%6.2fs] Added gamma_FR = 0\n', toc(t0));
add_block('simulink/Sources/Constant', [mdl '/gamma_RL'], 'Value','0', ...
    'Position', box(80,800,110,820)); fprintf('[%6.2fs] Added gamma_RL = 0\n', toc(t0));
add_block('simulink/Sources/Constant', [mdl '/gamma_RR'], 'Value','0', ...
    'Position', box(80,840,110,860)); fprintf('[%6.2fs] Added gamma_RR = 0\n', toc(t0));

% -------------------------------------------------------------------------
% Instantiate four TireCorner subsystems
% -------------------------------------------------------------------------
corners = {'FL','FR','RL','RR'};
y = [120, 260, 400, 540];
for i = 1:4
    name = ['Tire_' corners{i}];
    add_block(parentSub, [mdl '/' name], 'Position', box(480, y(i), 240, 120));
    fprintf('[%6.2fs] Instantiated: %s\n', toc(t0), [mdl '/' name]);
end

% Shared inputs
for i = 1:4
    tg = ['Tire_' corners{i}];
    add_line(mdl,'SW_kappa/1', [tg '/1'],'autorouting','on');
    fprintf('[%6.2fs] wired: SW_kappa -> %s/1 (kappa)\n', toc(t0), tg);
    add_line(mdl,'SW_alpha/1', [tg '/2'],'autorouting','on');
    fprintf('[%6.2fs] wired: SW_alpha -> %s/2 (alpha)\n', toc(t0), tg);
    add_line(mdl,'Vx/1',       [tg '/5'],'autorouting','on');
    fprintf('[%6.2fs] wired: Vx -> %s/5\n', toc(t0), tg);
    add_line(mdl,'mu_scale/1', [tg '/6'],'autorouting','on');
    fprintf('[%6.2fs] wired: mu_scale -> %s/6\n', toc(t0), tg);
    add_line(mdl,'K_scale/1',  [tg '/7'],'autorouting','on');
    fprintf('[%6.2fs] wired: K_scale -> %s/7\n', toc(t0), tg);
end

% Per-corner gamma & Fz
add_line(mdl,'gamma_FL/1','Tire_FL/3','autorouting','on'); fprintf('[%6.2fs] wired: gamma_FL -> Tire_FL/3\n', toc(t0));
add_line(mdl,'gamma_FR/1','Tire_FR/3','autorouting','on'); fprintf('[%6.2fs] wired: gamma_FR -> Tire_FR/3\n', toc(t0));
add_line(mdl,'gamma_RL/1','Tire_RL/3','autorouting','on'); fprintf('[%6.2fs] wired: gamma_RL -> Tire_RL/3\n', toc(t0));
add_line(mdl,'gamma_RR/1','Tire_RR/3','autorouting','on'); fprintf('[%6.2fs] wired: gamma_RR -> Tire_RR/3\n', toc(t0));

add_line(mdl,'Fz_FL/1','Tire_FL/4','autorouting','on');     fprintf('[%6.2fs] wired: Fz_FL -> Tire_FL/4\n', toc(t0));
add_line(mdl,'Fz_FR/1','Tire_FR/4','autorouting','on');     fprintf('[%6.2fs] wired: Fz_FR -> Tire_FR/4\n', toc(t0));
add_line(mdl,'Fz_RL/1','Tire_RL/4','autorouting','on');     fprintf('[%6.2fs] wired: Fz_RL -> Tire_RL/4\n', toc(t0));
add_line(mdl,'Fz_RR/1','Tire_RR/4','autorouting','on');     fprintf('[%6.2fs] wired: Fz_RR -> Tire_RR/4\n', toc(t0));

% -------------------------------------------------------------------------
% Scopes per corner + totals
% -------------------------------------------------------------------------
for i = 1:4
    sc = [mdl '/Scope_' corners{i}];
    add_block('simulink/Sinks/Scope', sc, 'NumInputPorts','3', ...
        'Position', box(900, y(i), 150, 100));
    fprintf('[%6.2fs] Added Scope: %s\n', toc(t0), sc);
    add_line(mdl, ['Tire_' corners{i} '/1'], [ 'Scope_' corners{i} '/1' ], 'autorouting','on');
    fprintf('[%6.2fs] wired: %s/1 -> Scope_%s/1 (Fx)\n', toc(t0), ['Tire_' corners{i}], corners{i});
    add_line(mdl, ['Tire_' corners{i} '/2'], [ 'Scope_' corners{i} '/2' ], 'autorouting','on');
    fprintf('[%6.2fs] wired: %s/2 -> Scope_%s/2 (Fy)\n', toc(t0), ['Tire_' corners{i}], corners{i});
    add_line(mdl, ['Tire_' corners{i} '/3'], [ 'Scope_' corners{i} '/3' ], 'autorouting','on');
    fprintf('[%6.2fs] wired: %s/3 -> Scope_%s/3 (Mz)\n', toc(t0), ['Tire_' corners{i}], corners{i});
end

add_block('simulink/Math Operations/Sum', [mdl '/Sum_Fx'], 'Inputs','++++', ...
    'Position', box(900, 660, 40, 40)); fprintf('[%6.2fs] Added Sum_Fx\n', toc(t0));
add_block('simulink/Math Operations/Sum', [mdl '/Sum_Fy'], 'Inputs','++++', ...
    'Position', box(900, 710, 40, 40)); fprintf('[%6.2fs] Added Sum_Fy\n', toc(t0));
add_block('simulink/Math Operations/Sum', [mdl '/Sum_Mz'], 'Inputs','++++', ...
    'Position', box(900, 760, 40, 40)); fprintf('[%6.2fs] Added Sum_Mz\n', toc(t0));
add_block('simulink/Sinks/Scope', [mdl '/Scope_Total'], 'NumInputPorts','3', ...
    'Position', box(1000, 680, 150, 100)); fprintf('[%6.2fs] Added Scope_Total\n', toc(t0));

add_line(mdl,'Tire_FL/1','Sum_Fx/1','autorouting','on'); fprintf('[%6.2fs] wired: Tire_FL/1 -> Sum_Fx/1\n', toc(t0));
add_line(mdl,'Tire_FR/1','Sum_Fx/2','autorouting','on'); fprintf('[%6.2fs] wired: Tire_FR/1 -> Sum_Fx/2\n', toc(t0));
add_line(mdl,'Tire_RL/1','Sum_Fx/3','autorouting','on'); fprintf('[%6.2fs] wired: Tire_RL/1 -> Sum_Fx/3\n', toc(t0));
add_line(mdl,'Tire_RR/1','Sum_Fx/4','autorouting','on'); fprintf('[%6.2fs] wired: Tire_RR/1 -> Sum_Fx/4\n', toc(t0));

add_line(mdl,'Tire_FL/2','Sum_Fy/1','autorouting','on'); fprintf('[%6.2fs] wired: Tire_FL/2 -> Sum_Fy/1\n', toc(t0));
add_line(mdl,'Tire_FR/2','Sum_Fy/2','autorouting','on'); fprintf('[%6.2fs] wired: Tire_FR/2 -> Sum_Fy/2\n', toc(t0));
add_line(mdl,'Tire_RL/2','Sum_Fy/3','autorouting','on'); fprintf('[%6.2fs] wired: Tire_RL/2 -> Sum_Fy/3\n', toc(t0));
add_line(mdl,'Tire_RR/2','Sum_Fy/4','autorouting','on'); fprintf('[%6.2fs] wired: Tire_RR/2 -> Sum_Fy/4\n', toc(t0));

add_line(mdl,'Tire_FL/3','Sum_Mz/1','autorouting','on'); fprintf('[%6.2fs] wired: Tire_FL/3 -> Sum_Mz/1\n', toc(t0));
add_line(mdl,'Tire_FR/3','Sum_Mz/2','autorouting','on'); fprintf('[%6.2fs] wired: Tire_FR/3 -> Sum_Mz/2\n', toc(t0));
add_line(mdl,'Tire_RL/3','Sum_Mz/3','autorouting','on'); fprintf('[%6.2fs] wired: Tire_RL/3 -> Sum_Mz/3\n', toc(t0));
add_line(mdl,'Tire_RR/3','Sum_Mz/4','autorouting','on'); fprintf('[%6.2fs] wired: Tire_RR/3 -> Sum_Mz/4\n', toc(t0));

add_line(mdl,'Sum_Fx/1','Scope_Total/1','autorouting','on'); fprintf('[%6.2fs] wired: Sum_Fx -> Scope_Total/1\n', toc(t0));
add_line(mdl,'Sum_Fy/1','Scope_Total/2','autorouting','on'); fprintf('[%6.2fs] wired: Sum_Fy -> Scope_Total/2\n', toc(t0));
add_line(mdl,'Sum_Mz/1','Scope_Total/3','autorouting','on'); fprintf('[%6.2fs] wired: Sum_Mz -> Scope_Total/3\n', toc(t0));

% -------------------------------------------------------------------------
% Solver & diagram layout
% -------------------------------------------------------------------------
set_param(mdl, 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.001', 'StopTime', '5');
fprintf('[%6.2fs] Solver set: FixedStepDiscrete, Ts=0.001, StopTime=5\n', toc(t0));

try
    Simulink.BlockDiagram.arrangeSystem(mdl, 'Direction','right', 'AutoRoute','on');
    fprintf('[%6.2fs] Diagram auto-arranged (no overlaps)\n', toc(t0));
catch
    fprintf('[%6.2fs] arrangeSystem not available; diagram left as-placed\n', toc(t0));
end

% Remove the template TireCorner so only the four instances remain
tmpl = [mdl '/TireCorner'];
if ~isempty(find_system(mdl,'SearchDepth',1,'FindAll','on','Type','Block','Name','TireCorner'))
    delete_block(tmpl);
    fprintf('Deleted template block: %s\n', tmpl);
end



% -------------------------------------------------------------------------
% Save
% -------------------------------------------------------------------------
here = fileparts(mfilename('fullpath'));
outdir = fullfile(here, 'harness');
if ~exist(outdir,'dir'); mkdir(outdir); fprintf('[%6.2fs] Created folder: %s\n', toc(t0), outdir); end
temp = save_system(mdl, fullfile(outdir, 'tire_harness.slx'));
fprintf('[%6.2fs] Saved harness to: %s\n', toc(t0), temp);
disp('Open and run. Set ScenarioSel = 1 (Steering) or 0 (Braking).');
end
