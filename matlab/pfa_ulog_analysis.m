%% PFA / TVMD ULog analysis (MATLAB R2023a+；不需工具箱，xregion 需 R2023a)
% 直接讀 .ulg（同資料夾的 pfa_ulogread.m），畫出：
%   1. 姿態：setpoint（姿態讓步之後）vs 實際
%   2. 位置：俯視軌跡、位置誤差換到機頭方向座標（往前 / 往右）
%   3. 舵機 η_x / η_y（每個推力器）與馬達推力
%   4. 控制器要求的推力換到機頭方向座標（往前 / 往右 / 往上，N）
% 並在命令列印出所選時間窗的平均值（偏差、舵機中位、左右 / 前後推力）。
%
% 座標：位置 NED、機身 FRD（PX4 慣例）。η_x 正 = 推力往右、η_y 正 = 推力往前。
% 推力器：M0 左前、M1 右前、M2 左後、M3 右後；舵機 2i = η_x、2i+1 = η_y。

clear; clc;

%% ── 設定 ────────────────────────────────────────────────────────────────
logFile   = 'log_413_2026-11-4-22-27-18.ulg';   % ← 改成你的 log
statWin   = [7 16];        % 統計用時間窗 (s，相對 log 開始)；[] = 整段在空中的時間
thrustA   = 1.17;          % 推力模型 f = A + B·u (N)
thrustB   = 7.845;

%% ── 讀檔 ────────────────────────────────────────────────────────────────
% 用 pfa_ulogread（純 MATLAB，不需工具箱）：UAV Toolbox R2023a 的 ulogreader
% 讀這版 PX4 的 log 會出錯（Colon operands must be in the range of the data type）
topics = {'vehicle_attitude','vehicle_attitude_setpoint','vehicle_local_position', ...
          'trajectory_setpoint','actuator_servos','actuator_motors', ...
          'vehicle_thrust_setpoint','vehicle_status'};
L      = pfa_ulogread(logFile, topics);
topics = topics(isfield(L.topic, topics));
tsec   = @(T) T.t;                         % s，相對第一筆 vehicle_status
getT   = @(name) L.topic.(name);

par   = @(name, def) paramOr(L.params, name, def);
Fmax  = par('PFA_MAX_THR', 36.06);
mass  = par('PFA_VEH_MASS', 1.5);
svMax = par('CA_SV_TL0_MAXA', 30);

att = getT('vehicle_attitude');          ta = tsec(att);
lp  = getT('vehicle_local_position');    tl = tsec(lp);
q   = col(att, 'q', 4);
[roll, pitch, yaw] = quatToEuler(q);

hasSp = any(strcmp(topics, 'vehicle_attitude_setpoint'));
if hasSp
    asp = getT('vehicle_attitude_setpoint'); tsp = tsec(asp);
    [rs, ps, ys] = quatToEuler(col(asp, 'q_d', 4));
end

%% ── 共同時間軸：vehicle_local_position ─────────────────────────────────
N  = lp.x;  E = lp.y;  D = lp.z;
yawL   = interpAngle(ta, yaw, tl);
rollL  = interpPrev(ta, roll,  tl, 'extrap');
pitchL = interpPrev(ta, pitch, tl, 'extrap');

hasTraj = any(strcmp(topics, 'trajectory_setpoint'));
if hasTraj
    tr  = getT('trajectory_setpoint'); ttr = tsec(tr);
    pos = col(tr, 'position', 3);
    spN = interpPrev(ttr, pos(:,1), tl, NaN);
    spE = interpPrev(ttr, pos(:,2), tl, NaN);
    spD = interpPrev(ttr, pos(:,3), tl, NaN);
    % 位置誤差（實際 − setpoint）換到機頭方向座標
    dN = N - spN;  dE = E - spE;
    errFwd   =  dN .* cos(yawL) + dE .* sin(yawL);
    errRight = -dN .* sin(yawL) + dE .* cos(yawL);
end

% 統計時間窗
if isempty(statWin)
    inAir = -D > 0.25;
    statWin = [min(tl(inAir)) max(tl(inAir))];
end
w = tl >= statWin(1) & tl <= statWin(2);

%% ── 圖 1：姿態 ──────────────────────────────────────────────────────────
figure('Name', '姿態', 'Color', 'w');
lab = {'roll (°)', 'pitch (°)', 'yaw (°)'};
act = {roll, pitch, yaw};
if hasSp, spv = {rs, ps, ys}; end
for k = 1:3
    subplot(3,1,k); hold on; grid on;
    if hasSp, plot(tsp, rad2deg(spv{k}), '--', 'LineWidth', 1.2); end
    plot(ta, rad2deg(act{k}), 'LineWidth', 1.2);
    ylabel(lab{k}); xregion(statWin(1), statWin(2), 'FaceAlpha', 0.06);
    if k == 1, legend('setpoint（讓步後）', '實際', 'Location', 'best'); end
end
xlabel('t (s)');

%% ── 圖 2：位置 ──────────────────────────────────────────────────────────
if hasTraj
    figure('Name', '位置', 'Color', 'w');
    subplot(2,2,[1 3]); hold on; grid on; axis equal;
    plot(spE, spN, 'k--', 'LineWidth', 1.0);
    plot(E, N, 'LineWidth', 1.2);
    xlabel('E (m)'); ylabel('N (m)'); title('俯視（東右、北上）');
    legend('setpoint', '實際', 'Location', 'best');

    subplot(2,2,2); hold on; grid on;
    plot(tl, errFwd * 100, tl, errRight * 100, 'LineWidth', 1.1);
    ylabel('誤差 (cm)'); legend('往前', '往右', 'Location', 'best');
    title('實際 − setpoint（機頭方向座標）');
    xregion(statWin(1), statWin(2), 'FaceAlpha', 0.06);

    subplot(2,2,4); hold on; grid on;
    plot(tl, -spD, 'k--', tl, -D, 'LineWidth', 1.1);
    ylabel('高度 (m)'); xlabel('t (s)'); legend('setpoint', '實際', 'Location', 'best');
end

%% ── 圖 3：舵機與馬達 ────────────────────────────────────────────────────
names = {'M0 左前', 'M1 右前', 'M2 左後', 'M3 右後'};
if any(strcmp(topics, 'actuator_servos'))
    sv = getT('actuator_servos'); tsv = tsec(sv);
    c  = col(sv, 'control', 8) * svMax;              % deg
    etaX = c(:, 1:2:8);  etaY = c(:, 2:2:8);
    figure('Name', '舵機與馬達', 'Color', 'w');
    subplot(3,1,1); plot(tsv, etaX, 'LineWidth', 1.0); grid on;
    ylabel('η_x (°) 右 +'); legend(names, 'Location', 'best'); yline([svMax -svMax], ':');
    subplot(3,1,2); plot(tsv, etaY, 'LineWidth', 1.0); grid on;
    ylabel('η_y (°) 前 +'); yline([svMax -svMax], ':');
end
if any(strcmp(topics, 'actuator_motors'))
    mo = getT('actuator_motors'); tmo = tsec(mo);
    u  = col(mo, 'control', 4);
    f  = (thrustA + thrustB * u) .* (u > 0.02);
    subplot(3,1,3); plot(tmo, f, 'LineWidth', 1.0); grid on;
    ylabel('推力 (N)'); xlabel('t (s)');
end

%% ── 圖 4：控制器要求的推力（機頭方向座標）──────────────────────────────
if any(strcmp(topics, 'vehicle_thrust_setpoint'))
    th = getT('vehicle_thrust_setpoint'); tth = tsec(th);
    fb = col(th, 'xyz', 3) * Fmax;                   % 機體 FRD (N)
    qi = interpQuat(ta, q, tth);
    fn = rotFRDtoNED(qi, fb);                        % NED (N)
    [~, ~, yi] = quatToEuler(qi);
    fFwd   =  fn(:,1) .* cos(yi) + fn(:,2) .* sin(yi);
    fRight = -fn(:,1) .* sin(yi) + fn(:,2) .* cos(yi);
    fUp    = -fn(:,3);
    figure('Name', '要求推力', 'Color', 'w');
    plot(tth, fFwd, tth, fRight, tth, fUp - mass * 9.81, 'LineWidth', 1.1); grid on;
    legend('往前', '往右', '往上 − 重量', 'Location', 'best');
    ylabel('N'); xlabel('t (s)'); title('控制器要求的推力（機頭方向座標）');
    xregion(statWin(1), statWin(2), 'FaceAlpha', 0.06);
end

%% ── 統計 ────────────────────────────────────────────────────────────────
fprintf('\n=== %s  統計時間窗 %.1f–%.1f s ===\n', logFile, statWin);
fprintf('姿態平均  roll %+.1f°  pitch %+.1f°\n', rad2deg(mean(rollL(w))), rad2deg(mean(pitchL(w))));
if hasTraj
    fprintf('位置偏差（實際 − setpoint）往前 %+.1f cm  往右 %+.1f cm\n', ...
        100 * mean(errFwd(w), 'omitnan'), 100 * mean(errRight(w), 'omitnan'));
end
if exist('etaX', 'var')
    ws = tsv >= statWin(1) & tsv <= statWin(2);
    for i = 1:4
        fprintf('%s  η_x %+5.1f°  η_y %+5.1f°\n', names{i}, mean(etaX(ws,i)), mean(etaY(ws,i)));
    end
end
if exist('f', 'var')
    wm = tmo >= statWin(1) & tmo <= statWin(2); fm = mean(f(wm,:), 1);
    fprintf('推力  左 %.2f / 右 %.2f N   前 %.2f / 後 %.2f N   對角 M0+M3 %.2f / M1+M2 %.2f N\n', ...
        fm(1)+fm(3), fm(2)+fm(4), fm(1)+fm(2), fm(3)+fm(4), fm(1)+fm(4), fm(2)+fm(3));
end
if exist('fFwd', 'var')
    wt = tth >= statWin(1) & tth <= statWin(2);
    fprintf('要求推力  往前 %+.2f N  往右 %+.2f N  往上 %.1f N（重量 %.1f N）\n', ...
        mean(fFwd(wt)), mean(fRight(wt)), mean(fUp(wt)), mass * 9.81);
end

%% ── helpers ─────────────────────────────────────────────────────────────
function v = paramOr(P, name, def)
    if isfield(P, name), v = P.(name); else, v = def; end
end

function vi = interpPrev(t, v, ti, fill)
% 前一筆取值（zero-order hold）；ULog 可能有重複時間戳，先去重
    [t, ia] = unique(t, 'last');
    vi = interp1(t, v(ia, :), ti, 'previous', fill);
end

function M = col(tt, name, n)
% 陣列欄位：ulogreader 會給 N×n 的變數 name；若是展開成 name_0… / name[0]… 也支援
    vars = tt.Properties.VariableNames;
    if any(strcmp(vars, name))
        M = double(tt.(name));
        if size(M, 2) >= n, M = M(:, 1:n); return; end
    end
    M = zeros(height(tt), n);
    for i = 1:n
        cand = {sprintf('%s_%d', name, i-1), sprintf('%s[%d]', name, i-1), sprintf('%s_%d_', name, i-1)};
        hit  = cand(ismember(cand, vars));
        if isempty(hit), error('找不到欄位 %s（第 %d 個）', name, i-1); end
        M(:, i) = double(tt.(hit{1}));
    end
end

function [r, p, y] = quatToEuler(q)          % q = [w x y z]，PX4 ZYX Euler (rad)
    w = q(:,1); x = q(:,2); yq = q(:,3); z = q(:,4);
    r = atan2(2*(w.*x + yq.*z), 1 - 2*(x.^2 + yq.^2));
    p = asin(max(-1, min(1, 2*(w.*yq - z.*x))));
    y = atan2(2*(w.*z + x.*yq), 1 - 2*(yq.^2 + z.^2));
end

function a = interpAngle(t, ang, ti)
    a = atan2(interpPrev(t, sin(ang), ti, 'extrap'), interpPrev(t, cos(ang), ti, 'extrap'));
end

function qi = interpQuat(t, q, ti)            % 取最近的前一筆（足夠密）
    idx = interpPrev(t, (1:numel(t))', ti, 'extrap');
    idx(isnan(idx)) = 1;
    idx = max(1, min(numel(t), round(idx)));
    qi  = q(idx, :);
end

function vn = rotFRDtoNED(q, vb)
    w = q(:,1); x = q(:,2); y = q(:,3); z = q(:,4);
    vn = zeros(size(vb));
    vn(:,1) = (1-2*(y.^2+z.^2)).*vb(:,1) + 2*(x.*y - w.*z).*vb(:,2) + 2*(x.*z + w.*y).*vb(:,3);
    vn(:,2) = 2*(x.*y + w.*z).*vb(:,1) + (1-2*(x.^2+z.^2)).*vb(:,2) + 2*(y.*z - w.*x).*vb(:,3);
    vn(:,3) = 2*(x.*z - w.*y).*vb(:,1) + 2*(y.*z + w.*x).*vb(:,2) + (1-2*(x.^2+y.^2)).*vb(:,3);
end
