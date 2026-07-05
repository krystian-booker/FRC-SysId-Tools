function mechid_analyze(matPath, outJsonPath)
%MECHID_ANALYZE Identify FRC mechanism constants from a MechID session.
%   Called headless by the MechID Python CLI:
%     matlab -batch "addpath(...); mechid_analyze('session.mat', 'results.json')"
%
%   Model: kA*dv/dt = V - kS*sign(v) - kG*g(pos) - kV*v   (mechanism units)
%   Feedforward from quasistatic least squares, kA from tfest (System
%   Identification Toolbox) on the dynamic steps, PID via LQR (Control System
%   Toolbox), plus a stator-current-limit sweep simulation.
%
%   Emits results.json in the same schema as the Python engine.

S = load(matPath);
A = jsondecode(char(S.analysis_json));
warns = {};
if isfield(A, 'prewarnings') && ~isempty(A.prewarnings)
    pre = A.prewarnings;
    if ~iscell(pre), pre = cellstr(pre); end
    warns = [warns, reshape(pre, 1, [])];
end

nMotors = numel(S.motors);
motorsOut = {};
maxVelSeen = 0;
for i = 1:nMotors
    M = getMotor(S.motors, i);
    % Two-pass fit: kA from the first pass compensates the quasistatic ramp-lag
    % bias (kS inflated by rampRate*kA/kV) on the second pass.
    [ff, ~] = fitFF(M, A, 0);
    [dyn, ~] = fitDyn(M, ff, A);
    [ff, w1] = fitFF(M, A, dyn.kA);
    [dyn, w2] = fitDyn(M, ff, A);
    warns = [warns, w1, w2]; %#ok<AGROW>
    maxVelSeen = max(maxVelSeen, ff.max_velocity_seen);

    mo = struct();
    mo.name = char(M.name);
    mo.kS = ff.kS; mo.kV = ff.kV; mo.kA = dyn.kA; mo.kG = ff.kG;
    mo.kV_dynamic = dyn.kV_dynamic;
    mo.r2_quasistatic = ff.r2;
    mo.r2_dynamic = dyn.r2;
    mo.tfestFitPercent = dyn.tfestFitPercent;
    mo.n_quasistatic = ff.n_samples;
    mo.n_dynamic = dyn.n_samples;
    mo.max_velocity_seen = ff.max_velocity_seen;
    mo.frictionSplit = frictionSplit(M, ff, dyn.kA);
    mo.scatter = ff.scatter;
    mo.validation = validateSegs(M, ff, dyn.kA, A);
    motorsOut{end+1} = mo; %#ok<AGROW>
end

% --- Combine across motors (mean; per-module spread for swerve) ---
combined = struct();
for f = {'kS', 'kV', 'kA', 'kG'}
    vals = cellfun(@(m) m.(f{1}), motorsOut);
    combined.(f{1}) = mean(vals);
end
if nMotors > 1
    spread = 0;
    for f = {'kV', 'kA'}
        vals = cellfun(@(m) m.(f{1}), motorsOut);
        if abs(combined.(f{1})) > 1e-9
            spread = max(spread, 100 * (max(vals) - min(vals)) / abs(combined.(f{1})));
        end
    end
    combined.moduleSpreadPercent = spread;
    if spread > 15
        warns{end+1} = sprintf(['Per-module constants spread by %.0f%% - check for a ' ...
            'dragging or mechanically unhealthy module before trusting shared gains.'], spread);
    end
else
    combined.moduleSpreadPercent = NaN;
end

% --- Gearing (estimate from kV if the device-reported ratio is untrusted) ---
gearing = struct();
ratio = A.gearing.ratio;
if A.gearing.trusted && ~isempty(ratio) && isfinite(ratio) && ratio > 0
    gearing.ratio = ratio; gearing.trusted = true; gearing.estimated = false;
else
    gearing.ratio = combined.kV / A.phys.ke_v_per_rotor_rps;
    gearing.trusted = false; gearing.estimated = true;
    warns{end+1} = sprintf(['Gear ratio not trusted from device config (RemoteCANcoder ' ...
        'feedback?). Estimated rotor-to-mechanism ratio %.1f from identified kV - set ' ...
        '.withRotorToMechanismRatio() in the robot config for accurate current modeling.'], ...
        gearing.ratio);
end

% --- LQR gains ---
lqrP = struct();
lqrP.max_position_error = A.lqr.max_position_error;
if isempty(A.lqr.max_velocity_error) || ~isfinite(nanOr(A.lqr.max_velocity_error))
    lqrP.max_velocity_error = max(0.15 * maxVelSeen, 1e-6);
else
    lqrP.max_velocity_error = A.lqr.max_velocity_error;
end
lqrP.max_effort = A.lqr.max_effort;
combined.pid = lqrGains(char(A.pidDomain), combined.kV, combined.kA, lqrP);
combined.motionMagic = motionMagic(combined, char(A.gravityModel), A.vbat);
combined.lqrParams = lqrP;

% --- Current-limit sweep ---
motion = A.motionTemplate;
if strcmp(char(motion.kind), 'spinup') && isfield(motion, 'target_vel_factor')
    motion.target_vel = motion.target_vel_factor * maxVelSeen;
end
sweep = currentSweep(combined, A, gearing.ratio, motion);

% --- Assemble & write ---
results = struct();
results.engine = 'matlab';
results.sessionName = char(A.sessionName);
results.mechanismType = char(A.mechanismType);
results.pidDomain = char(A.pidDomain);
results.gravityModel = char(A.gravityModel);
results.positionUnits = char(A.positionUnits);
results.vbat = A.vbat;
results.nMotors = A.nMotors;
results.motors = motorsOut;
results.combined = combined;
results.gearing = gearing;
results.phys = A.phys;
results.currentSweep = sweep;
results.warnings = warns;

fid = fopen(outJsonPath, 'w');
fwrite(fid, jsonencode(results));
fclose(fid);
fprintf('MechID: wrote %s\n', outJsonPath);
end

% =======================================================================
function M = getMotor(motors, i)
if iscell(motors)
    M = motors{i};
else
    M = motors(i);
end
M.t = double(M.t(:)); M.pos = double(M.pos(:)); M.vel = double(M.vel(:));
M.acc = double(M.acc(:)); M.volts = double(M.volts(:));
M.seg_kind = double(M.seg_kind(:)); M.seg_id = double(M.seg_id(:));
end

function g = gravitySeries(model, pos, horizontal)
switch model
    case 'CONSTANT'
        g = ones(size(pos));
    case 'COSINE'
        g = cos(2 * pi * (pos - horizontal));
    otherwise
        g = zeros(size(pos));
end
end

function q = quantileLocal(x, p)
x = sort(x(:));
q = x(max(1, min(numel(x), ceil(p * numel(x)))));
end

function idx = downsampleIdx(n, maxN)
idx = unique(round(linspace(1, n, min(n, maxN))));
end

function v = nanOr(x)
if isempty(x), v = NaN; else, v = x; end
end

% =======================================================================
function [ff, warns] = fitFF(M, A, kaComp)
warns = {};
qs = M.seg_kind == 1;
vel = M.vel(qs); acc = M.acc(qs); pos = M.pos(qs);
volts = M.volts(qs) - kaComp * acc;
if nnz(qs) < 50, error('MechID:fitFF', 'only %d quasistatic samples', nnz(qs)); end
vmax = max(abs(vel));
if vmax <= 0, error('MechID:fitFF', 'mechanism never moved during quasistatic test'); end

% Velocity floor: proportional to top speed, but never below 8x the measurement
% noise - noise-ambiguous signs corrupt the kS (sign(v)) regressor.
dv = diff(vel);
sigma = 1.4826 * median(abs(dv - median(dv))) / sqrt(2);
moving = abs(vel) > max(0.02 * vmax, 8 * sigma);
if nnz(moving) > 20
    th = quantileLocal(abs(acc(moving)), 0.80);
else
    th = inf;
end
mask = moving & abs(acc) <= th;
if nnz(mask) < 30
    warns{end+1} = sprintf('%s: few steady quasistatic samples (%d)', char(M.name), nnz(mask));
    mask = moving;
end

v = vel(mask); u = volts(mask); p = pos(mask);
model = char(A.gravityModel);
hasG = any(strcmp(model, {'CONSTANT', 'COSINE'}));
if hasG
    X = [sign(v), gravitySeries(model, p, A.horizontal), v];
else
    X = [sign(v), v];
end
coef = X \ u;

ff = struct();
ff.kS = coef(1);
if hasG
    ff.kG = coef(2); ff.kV = coef(3);
else
    ff.kG = 0; ff.kV = coef(2);
end
if ff.kS < 0
    warns{end+1} = sprintf('%s: regression gave kS=%.3f < 0; clamped to 0', char(M.name), ff.kS);
    ff.kS = 0;
end
if ff.kV <= 0, error('MechID:fitFF', 'kV <= 0; data unusable'); end

res = u - X * coef;
ssTot = sum((u - mean(u)).^2);
if ssTot > 0, ff.r2 = 1 - sum(res.^2) / ssTot; else, ff.r2 = 0; end
ff.n_samples = nnz(mask);
ff.max_velocity_seen = vmax;

idx = downsampleIdx(numel(v), 1500);
ff.scatter = struct('vel', v(idx), 'volts', u(idx), 'fit', X(idx, :) * coef);
end

% =======================================================================
function [dyn, warns] = fitDyn(M, ff, A)
warns = {};
model = char(A.gravityModel);
dynMask = M.seg_kind == 2;
if nnz(dynMask) < 50, error('MechID:fitDyn', 'only %d dynamic samples', nnz(dynMask)); end
Ts = median(diff(M.t));

uEffAll = M.volts - ff.kS * sign(M.vel) - ff.kG * gravitySeries(model, M.pos, A.horizontal);

dyn = struct('kA', NaN, 'kV_dynamic', NaN, 'tfestFitPercent', NaN);
usedTfest = false;
try
    segids = unique(M.seg_id(dynMask));
    exps = {};
    for s = segids'
        sel = (M.seg_id == s) & dynMask;
        if nnz(sel) < 30, continue; end
        exps{end+1} = iddata(M.vel(sel), uEffAll(sel), Ts); %#ok<AGROW>
        if numel(exps) >= 20, break; end
    end
    if isempty(exps), error('MechID:fitDyn', 'no usable dynamic segments'); end
    if numel(exps) > 1
        data = merge(exps{:});
    else
        data = exps{1};
    end
    sys = tfest(data, 1, 0);
    b = sys.Numerator(end);
    a1 = sys.Denominator(end);
    if b > 0 && a1 > 0
        dyn.kA = 1 / b;
        dyn.kV_dynamic = a1 / b;
        fitPct = sys.Report.Fit.FitPercent;
        dyn.tfestFitPercent = mean(fitPct(:));
        usedTfest = true;
    end
catch err
    warns{end+1} = sprintf('%s: tfest failed (%s); using acceleration regression', ...
        char(M.name), err.message);
end

if ~usedTfest
    % Fallback: regress recorded acceleration on [u_eff, v] like the Python engine.
    X = [uEffAll(dynMask), M.vel(dynMask)];
    coef = X \ M.acc(dynMask);
    if coef(1) <= 0, error('MechID:fitDyn', 'non-physical kA from regression'); end
    dyn.kA = 1 / coef(1);
    dyn.kV_dynamic = -coef(2) / coef(1);
end

if ff.kV > 0 && abs(dyn.kV_dynamic - ff.kV) > 0.30 * ff.kV
    warns{end+1} = sprintf(['%s: dynamic kV (%.3f) differs from quasistatic kV (%.3f) ' ...
        'by >30%% - model may be nonlinear (backlash, load variation)'], ...
        char(M.name), dyn.kV_dynamic, ff.kV);
end

% Regression-style r2 on acceleration prediction, comparable across engines.
accPred = (uEffAll(dynMask) - dyn.kV_dynamic * M.vel(dynMask)) / dyn.kA;
accMeas = M.acc(dynMask);
ssTot = sum((accMeas - mean(accMeas)).^2);
if ssTot > 0
    dyn.r2 = 1 - sum((accMeas - accPred).^2) / ssTot;
else
    dyn.r2 = 0;
end
dyn.n_samples = nnz(dynMask);
end

% =======================================================================
function fs = frictionSplit(M, ff, kA)
coast = M.seg_kind == 3;
fs = NaN; % -> null in JSON
if nnz(coast) < 30, return; end
vel = M.vel(coast); acc = M.acc(coast);
vmax = max(abs(vel));
moving = abs(vel) > 0.02 * max(vmax, 1e-9);
if nnz(moving) < 20, return; end
X = [sign(vel(moving)), vel(moving)];
coef = X \ (-kA * acc(moving));
fs = struct('coulomb_volts', coef(1), 'viscous_volts_per_ups', coef(2), ...
    'back_emf_volts_per_ups', ff.kV - coef(2));
end

% =======================================================================
function pid = lqrGains(domain, kV, kA, p)
if strcmp(domain, 'POSITION')
    Am = [0 1; 0 -kV / kA];
    Bm = [0; 1 / kA];
    Q = diag([1 / p.max_position_error^2, 1 / p.max_velocity_error^2]);
    R = 1 / p.max_effort^2;
    K = lqr(Am, Bm, Q, R);
    pid = struct('kP', K(1), 'kI', 0, 'kD', K(2));
else
    Am = -kV / kA;
    Bm = 1 / kA;
    Q = 1 / p.max_velocity_error^2;
    R = 1 / p.max_effort^2;
    K = lqr(Am, Bm, Q, R);
    pid = struct('kP', K(1), 'kI', 0, 'kD', 0);
end
end

function mm = motionMagic(c, gravityModel, vbat)
if any(strcmp(gravityModel, {'CONSTANT', 'COSINE'})), kgMax = abs(c.kG); else, kgMax = 0; end
headroom = vbat - c.kS - kgMax;
if headroom <= 0 || c.kV <= 0
    mm = struct('cruiseVelocity', 0, 'acceleration', 0);
    return;
end
cruise = 0.85 * headroom / c.kV;
accelAvail = (headroom - c.kV * cruise) / c.kA;
mm = struct('cruiseVelocity', cruise, 'acceleration', max(0.70 * accelAvail, 0));
end

% =======================================================================
function sweep = currentSweep(c, A, ratio, motion)
phys = A.phys;
rMotor = phys.resistance;
keMech = phys.ke_v_per_rotor_rps * ratio;
vbat = A.vbat;
nMotors = A.nMotors;
model = char(A.gravityModel);
horiz = A.horizontal;
kind = char(motion.kind);

top = min(phys.stall_current, 150);
limits = 10:5:top;

% Performance is relative to the highest realistic limit (top of the sweep), not
% stall current - the question is "how low can the limit go before it hurts".
baseT = simMove(limits(end), false);
nL = numel(limits);
times = nan(1, nL); perfs = zeros(1, nL); peaks = zeros(1, nL);
traces = {};
for i = 1:nL
    [tMove, tr, pk] = simMove(limits(i), true);
    times(i) = tMove;
    if isfinite(tMove) && isfinite(baseT), perfs(i) = 100 * baseT / tMove; end
    peaks(i) = pk;
    traces{end+1} = tr; %#ok<AGROW>
end

sweep = struct();
sweep.motion = motion;
sweep.limitsAmps = limits;
sweep.moveTimeSec = fillNan(times);
sweep.performancePercent = perfs;
sweep.peakSupplyAmpsTotal = peaks;
if isfinite(baseT), sweep.baselineTimeSec = baseT; else, sweep.baselineTimeSec = NaN; end
rec = struct();
rec.amps90 = firstAt(90); rec.amps95 = firstAt(95); rec.amps98 = firstAt(98);
rec.target = firstAt(100 * A.perf_target);
rec.targetPercent = 100 * A.perf_target;
sweep.recommended = rec;
sweep.traces = traces;

    function lim = firstAt(threshold)
        ii = find(perfs >= threshold, 1);
        if isempty(ii), lim = NaN; else, lim = limits(ii); end
    end

    function x = fillNan(x)
        x(~isfinite(x)) = NaN;
    end

    function gv = gOf(p)
        switch model
            case 'CONSTANT', gv = 1;
            case 'COSINE', gv = cos(2 * pi * (p - horiz));
            otherwise, gv = 0;
        end
    end

    function [tOut, trace, peakSupply] = simMove(iLim, record)
        dt = 1e-3;
        v = 0; p = 0; t = 0;
        peakSupply = 0;
        cap = 200;
        trace = struct('t', zeros(1, 0), 'vel', zeros(1, 0), 'pos', zeros(1, 0), ...
            'stator', zeros(1, 0), 'supply', zeros(1, 0));
        raw = zeros(5, 30000);
        nRaw = 0;
        tOut = NaN;
        for k = 1:30000
            emf = keMech * v;
            vHi = min(vbat, emf + iLim * rMotor);
            vLo = max(-vbat, emf - iLim * rMotor);
            braking = false;
            if strcmp(kind, 'p2p') && v > 0
                aBrake = (vLo - c.kS * sign(v) - c.kG * gOf(p) - c.kV * v) / c.kA;
                if aBrake < 0
                    braking = (motion.distance - p) <= v * v / (2 * -aBrake);
                end
            end
            if braking, vApp = vLo; else, vApp = vHi; end
            a = (vApp - c.kS * sign(v) - c.kG * gOf(p) - c.kV * v) / c.kA;
            if ~braking && a <= 0 && v <= 1e-9
                tOut = NaN; % cannot overcome friction/gravity at this limit
                break;
            end
            iStator = (vApp - emf) / rMotor;
            iSupply = abs(iStator) * min(abs(vApp) / vbat, 1) * nMotors;
            peakSupply = max(peakSupply, iSupply);
            if record
                nRaw = nRaw + 1;
                raw(:, nRaw) = [t; v; p; abs(iStator); iSupply];
            end
            v = max(v + a * dt, 0);
            p = p + v * dt;
            t = t + dt;
            done = false;
            if any(strcmp(kind, {'p2p', 'sprint'})) && p >= motion.distance, done = true; end
            if strcmp(kind, 'spinup') && v >= motion.target_vel, done = true; end
            if done
                tOut = t;
                break;
            end
        end
        if record && nRaw > 0
            idx = downsampleIdx(nRaw, cap);
            trace = struct('t', raw(1, idx), 'vel', raw(2, idx), 'pos', raw(3, idx), ...
                'stator', raw(4, idx), 'supply', raw(5, idx));
        end
    end
end

% =======================================================================
function out = validateSegs(M, ff, kA, A)
model = char(A.gravityModel);
out = {};
dt = median(diff(M.t));
segids = unique(M.seg_id(M.seg_kind == 2));
for s = segids'
    sel = (M.seg_id == s) & (M.seg_kind == 2);
    if nnz(sel) < 20, continue; end
    t = M.t(sel); vel = M.vel(sel); volts = M.volts(sel); pos = M.pos(sel);
    sim = zeros(size(vel));
    sim(1) = vel(1);
    for k = 1:numel(vel) - 1
        a = (volts(k) - ff.kS * sign(sim(k)) ...
            - ff.kG * gravitySeries(model, pos(k), A.horizontal) ...
            - ff.kV * sim(k)) / kA;
        sim(k + 1) = sim(k) + a * dt;
    end
    denom = norm(vel - mean(vel));
    if denom > 0, fitPct = 100 * (1 - norm(vel - sim) / denom); else, fitPct = 0; end
    idx = downsampleIdx(numel(t), 300);
    seg = struct();
    firstIdx = find(sel, 1);
    seg.id = sprintf('segment_%d_t%.1fs', s, M.t(firstIdx));
    seg.fitPercent = fitPct;
    seg.t = t(idx) - t(1);
    seg.measured = vel(idx);
    seg.simulated = sim(idx);
    out{end+1} = seg; %#ok<AGROW>
    if numel(out) >= 6, break; end
end
end
