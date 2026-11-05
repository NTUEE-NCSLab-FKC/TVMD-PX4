%% =========================================================================
%  pfa_replay_plots.m
%  TVMD / PFA 飛行 log 繪圖（與「TVMD 飛行重播」相同的圖），ROS 座標：
%    世界 ENU（x 東、y 北、z 上），機體 FLU（x 前、y 左、z 上）
%  直接讀 .ulg（同資料夾的 pfa_ulogread.m），不需轉 CSV、不需工具箱。
%
%  圖：
%    1  位置 p 與 setpoint                 8  推力器推力 f_i = a + b u_i
%    2  位置誤差 e = p_sp - p 與 |e|       9  舵機角度 eta_x（實線）/ eta_y（虛線）
%    3  速度 v 與 setpoint                10  舵機 eta_x（推力往左右）
%    4  姿態 roll / pitch / yaw 與 setpoint 11 舵機 eta_y（推力往前後）
%    5  角速度 omega                      12  電壓 / 電流
%    6  角加速度 d(omega)/dt              13  XY 平面軌跡（顏色 = 時間）
%    7  力 F、力矩 M：控制器要求 vs 分配後 vs 由推力器重建
%                                         14  XYZ 三維軌跡（顏色 = 時間）
%   15  推力器推力方向俯視投影（4 格依機身位置排列，顏色 = 時間）
%   16  姿態追蹤：參考 ref(t)、送出指令、讓步後 setpoint、實際（、真值）
%   17  傾斜平面 roll × pitch：參考 vs 實際（顏色 = 時間）
%  SITL：log 有 *_groundtruth 時，位置、速度、姿態、角速度與軌跡圖多畫「真值」
%        （黑色點線）；實機沒有這些 topic，自動略過。
%  參考姿態 ref(t)：由 debug_vect 還原（需 SDLOG_PROFILE 含 32，腳本送
%        "att_cmd" = ref(z + lead) − b 與 "att_meta" = (b_roll, b_pitch, lead)）：
%        ref(z + lead) = (x, y) + b，時間 = z + lead + min(timestamp − z)。
%  軌跡與投影都畫成連續線，顏色沿線隨時間變化（jet 色階）。
%
%  座標換算（PX4 NED/FRD -> ROS ENU/FLU）：
%    p_ENU = (E, N, -D)；v_FLU = (x, -y, -z)（角速度、力、力矩）
%    roll_FLU = roll，pitch_FLU = -pitch（機頭下為正），yaw_ENU = 90 deg - yaw_NED
%  control_allocation_meta_data 的 control_sp / allocated_control 本來就是
%  FLU、N 與 N·m，順序 [Mx My Mz Fx Fy Fz]。
%  推力器 i：方向（FLU）= (cos ex sin ey, -sin ex, cos ex cos ey)，
%  ex = 舵機 2i（推力往右為正）、ey = 舵機 2i+1（推力往前為正）。
% =========================================================================
clear; clc; close all;

%% ── 使用者參數 ───────────────────────────────────────────────────────────
LOG_FILE   = 'log_416_2026-11-4-22-38-08.ulg';
T_START    = 0;          % 起始時間 [s]（相對第一筆 vehicle_status）
T_END      = inf;        % 結束時間 [s]，inf = 到最後
ZERO_TIME  = true;       % true：圖的時間軸從 T_START 起算為 0
THRUST_A   = 1.17;       % 推力模型 f = A + B u [N]
THRUST_B   = 7.845;
SHADE_MODE = true;       % 非 OFFBOARD（或未解鎖）時段以灰底標示
EXPORT_PDF = false;      % 匯出向量 PDF
OUT_DIR    = 'fig';
FIG_W      = 16;         % 圖寬 [cm]
FIG_H      = 11;         % 圖高 [cm]（3 列子圖時自動加高）
CMAP       = 'jet';      % 軌跡圖時間色階
COL_XYZ    = [1 0 0; 0 1 0; 0 0 1];          % x / y / z：正紅、正綠、正藍
COL_MOD    = [0.85 0.33 0.10; 0.93 0.69 0.13; 0.49 0.18 0.56; 0.30 0.75 0.93];
MOD_NAME   = {'M$_0$ (FL)', 'M$_1$ (FR)', 'M$_2$ (RL)', 'M$_3$ (RR)'};
%% ─────────────────────────────────────────────────────────────────────────

%% 論文格式：LaTeX、向量輸出
set(groot, 'defaultTextInterpreter',              'latex');
set(groot, 'defaultAxesTickLabelInterpreter',     'latex');
set(groot, 'defaultLegendInterpreter',            'latex');
set(groot, 'defaultColorbarTickLabelInterpreter', 'latex');
set(groot, 'defaultFigureRenderer',  'painters');
set(groot, 'defaultAxesFontSize',    10);
set(groot, 'defaultTextFontSize',    10);
set(groot, 'defaultAxesLineWidth',   0.6);
set(groot, 'defaultLineLineWidth',   1.1);
set(groot, 'defaultFigureColor',     'w');
set(groot, 'defaultAxesColor',       'w');
set(groot, 'defaultAxesBox',         'on');
set(groot, 'defaultAxesXGrid',       'on');
set(groot, 'defaultAxesYGrid',       'on');

%% 讀檔
topics = {'vehicle_status','vehicle_local_position','trajectory_setpoint', ...
          'vehicle_attitude','vehicle_attitude_setpoint','vehicle_angular_velocity', ...
          'control_allocation_meta_data','vehicle_thrust_setpoint','vehicle_torque_setpoint', ...
          'actuator_motors','actuator_servos','battery_status','debug_vect', ...
          'vehicle_local_position_groundtruth','vehicle_attitude_groundtruth', ...
          'vehicle_angular_velocity_groundtruth'};
L   = pfa_ulogread(LOG_FILE, topics);
has = @(n) isfield(L.topic, n);
P   = L.params;
prm = @(n, d) getParam(P, n, d);
svMax = prm('CA_SV_TL0_MAXA', 30);
Fmax  = prm('PFA_MAX_THR', 36.06);
Tmax  = prm('PFA_MAX_TOR', 2.51);
modPos = zeros(4, 2);                          % 推力器位置（FLU，x 前 y 左）[m]
defPos = [0.16 0.16; 0.16 -0.16; -0.16 0.16; -0.16 -0.16];
for i = 1:4
    modPos(i, :) = [prm(sprintf('CA_MD%d_PX', i-1), defPos(i,1)), prm(sprintf('CA_MD%d_PY', i-1), defPos(i,2))];
end

tz  = 0; if ZERO_TIME && isfinite(T_START), tz = T_START; end
win = @(T) T(T.t >= T_START & T.t <= T_END, :);
tt  = @(T) T.t - tz;

%% 狀態（模式、解鎖）
if has('vehicle_status')
    vs = L.topic.vehicle_status;
    armedFn = @(t) interpPrev(vs.t, double(vs.arming_state == 2), t, 0);
    spans = modeSpans(vs, T_START, T_END, tz);
else
    armedFn = @(t) ones(size(t)); spans = zeros(0, 2);
end
shade = @(ax) shadeSpans(ax, spans, SHADE_MODE);

%% 位置、速度（ENU）
lp  = win(L.topic.vehicle_local_position);  t_lp = tt(lp);
p   = [lp.y, lp.x, -lp.z];                     % ENU
v   = [lp.vy, lp.vx, -lp.vz];
p_sp = nan(size(p)); v_sp = nan(size(v)); yaw_sp = [];
if has('trajectory_setpoint')
    tr  = L.topic.trajectory_setpoint;
    pn  = tr.position;  vn = tr.velocity;
    p_sp = interpPrev(tr.t, [pn(:,2), pn(:,1), -pn(:,3)], lp.t, NaN);
    v_sp = interpPrev(tr.t, [vn(:,2), vn(:,1), -vn(:,3)], lp.t, NaN);
    trw  = win(tr);
    yaw_sp = [tt(trw), rad2deg(wrapPi(pi/2 - trw.yaw))];
end
e = p_sp - p;
hasGT = has('vehicle_local_position_groundtruth');      % SITL 才有
if hasGT
    gt = win(L.topic.vehicle_local_position_groundtruth);  t_gt = tt(gt);
    p_gt = [gt.y, gt.x, -gt.z];  v_gt = [gt.vy, gt.vx, -gt.vz];
end
COL_GT = [0 0 0];  GT_STYLE = {':', 'Color', COL_GT, 'LineWidth', 1.3, 'DisplayName', 'ground truth'};

%% 姿態（FLU / ENU）
att = win(L.topic.vehicle_attitude);  t_att = tt(att);
eul = nedEulerToFlu(att.q);                    % [roll pitch yaw] rad
if has('vehicle_attitude_setpoint')
    asp = win(L.topic.vehicle_attitude_setpoint);  t_asp = tt(asp);
    eul_sp = nedEulerToFlu(asp.q_d);
end
if has('vehicle_attitude_groundtruth')
    agt = win(L.topic.vehicle_attitude_groundtruth);  t_agt = tt(agt);
    eul_gt = nedEulerToFlu(agt.q);
end

%% 參考姿態 ref(t)（由 debug_vect 還原；PX4 roll/pitch 慣例 deg -> FLU）
hasRef = false;
if has('debug_vect')
    dv  = L.topic.debug_vect;
    nm  = cellfun(@(c) c(1:find([c char(0)] == char(0), 1) - 1), cellstr(dv.name), 'UniformOutput', false);
    isC = strcmp(nm, 'att_cmd');  isM = strcmp(nm, 'att_meta');
    if any(isC)
        if any(isM)
            k = find(isM, 1, 'last');  bR = dv.x(k);  bP = dv.y(k);  lead = dv.z(k);
        else
            bR = 0; bP = 0; lead = 0;
            warning('debug_vect 沒有 att_meta：假設偏差 b = 0、lead = 0（ref 會等於送出的指令）');
        end
        zc   = dv.z(isC);  trx = dv.t(isC);
        off  = min(trx - zc);                         % 腳本時鐘 -> log 時間（去掉傳送延遲的抖動）
        tref = zc + lead + off;
        ref  = [dv.x(isC) + bR, -(dv.y(isC) + bP)];   % [roll, pitch] FLU (deg)
        cmd  = [dv.x(isC), -dv.y(isC)];               % 送出的指令（FLU，在收到的時間）
        inW  = tref >= T_START & tref <= T_END;
        t_ref = tref(inW) - tz;  ref = ref(inW, :);
        inW2 = trx >= T_START & trx <= T_END;
        t_cmd = trx(inW2) - tz;  cmd = cmd(inW2, :);
        hasRef = ~isempty(t_ref);
        fprintf('ref(t): %d samples, b = (%+.2f, %+.2f) deg, lead = %.2f s, clock offset %.3f s\n', ...
            numel(t_ref), bR, bP, lead, off);
    end
end

%% 角速度、角加速度（FLU）
if has('vehicle_angular_velocity')
    av = win(L.topic.vehicle_angular_velocity);  t_av = tt(av);
    w  = av.xyz .* [1 -1 -1];
    if has('vehicle_angular_velocity_groundtruth')
        avg = win(L.topic.vehicle_angular_velocity_groundtruth);  t_avg = tt(avg);
        w_gt = avg.xyz .* [1 -1 -1];
    end
    if any(strcmp(av.Properties.VariableNames, 'xyz_derivative'))
        wd = av.xyz_derivative .* [1 -1 -1];
    else
        wd = [];
    end
end

%% 舵機、馬達、推力器重建（FLU）
hasAct = has('actuator_servos') && has('actuator_motors');
if hasAct
    sv  = win(L.topic.actuator_servos);  t_sv = tt(sv);
    eta = sv.control(:, 1:8) * svMax;                     % deg
    etaX = eta(:, 1:2:8);  etaY = eta(:, 2:2:8);
    mo  = L.topic.actuator_motors;
    u   = interpPrev(mo.t, mo.control(:, 1:4), sv.t, 0);  % 對齊舵機時間
    f   = (THRUST_A + THRUST_B * u) .* (u > 0.02) .* armedFn(sv.t);
    ex  = deg2rad(etaX);  ey = deg2rad(etaY);
    fx  = cos(ex) .* sin(ey) .* f;  fy = -sin(ex) .* f;  fz = cos(ex) .* cos(ey) .* f;
    Frec = [sum(fx, 2), sum(fy, 2), sum(fz, 2)];
    Mrec = [fz * modPos(:,2), -(fz * modPos(:,1)), fy * modPos(:,1) - fx * modPos(:,2)];
end

%% 控制器要求的 wrench（FLU，N / N·m）
Fcmd = []; Mcmd = []; Falc = []; Malc = [];
if has('control_allocation_meta_data')
    md = win(L.topic.control_allocation_meta_data);  t_md = tt(md);
    Mcmd = md.control_sp(:, 1:3);  Fcmd = md.control_sp(:, 4:6);
    if any(strcmp(md.Properties.VariableNames, 'allocated_control'))
        Malc = md.allocated_control(:, 1:3);  Falc = md.allocated_control(:, 4:6);
    end
elseif has('vehicle_thrust_setpoint') && has('vehicle_torque_setpoint')
    th = win(L.topic.vehicle_thrust_setpoint);  t_md = tt(th);
    tq = L.topic.vehicle_torque_setpoint;
    Fcmd = th.xyz .* [1 -1 -1] * Fmax;
    Mcmd = interpPrev(tq.t, tq.xyz .* [1 -1 -1] * Tmax, th.t, NaN);
end

%% ── 圖 1：位置 ───────────────────────────────────────────────────────────
lbl = {'$x$ [m]', '$y$ [m]', '$z$ [m]'};
fig = newFig('position', FIG_W, FIG_H * 1.25);
tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:3
    ax = nexttile(tl); hold(ax, 'on'); shade(ax);
    plot(ax, t_lp, p(:,k), '-', 'Color', COL_XYZ(k,:), 'DisplayName', 'actual');
    plot(ax, t_lp, p_sp(:,k), '--', 'Color', 0.25 + 0.5 * COL_XYZ(k,:), 'DisplayName', 'setpoint');
    if hasGT, plot(ax, t_gt, p_gt(:,k), GT_STYLE{:}); end
    ylabel(ax, lbl{k});
    if k == 1, title(ax, 'Position (ENU)'); legend(ax, 'Location', 'best'); end
end
xlabel(ax, '$t$ [s]');

%% ── 圖 2：位置誤差 ───────────────────────────────────────────────────────
fig = newFig('position_error', FIG_W, FIG_H * 0.7); ax = axes(fig); hold(ax, 'on'); shade(ax);
axn = 'xyz';
for k = 1:3
    plot(ax, t_lp, e(:,k), 'Color', COL_XYZ(k,:), 'DisplayName', sprintf('$e_%s$', axn(k)));
end
plot(ax, t_lp, vecnorm(e, 2, 2), 'k', 'DisplayName', '$\|e\|$');
if hasGT                                         % 真實誤差（setpoint − 真值）
    e_gt = interpPrev(lp.t, p_sp, gt.t, NaN) - p_gt;
    plot(ax, t_gt, vecnorm(e_gt, 2, 2), 'k:', 'LineWidth', 1.3, 'DisplayName', '$\|e\|$ ground truth');
end
yline(ax, 0, 'k:', 'HandleVisibility', 'off');
ylabel(ax, '$e = p_{sp} - p$ [m]'); xlabel(ax, '$t$ [s]');
title(ax, 'Position Error (ENU)'); legend(ax, 'Location', 'best');

%% ── 圖 3：速度 ───────────────────────────────────────────────────────────
lbl = {'$v_x$ [m/s]', '$v_y$ [m/s]', '$v_z$ [m/s]'};
fig = newFig('velocity', FIG_W, FIG_H * 1.25);
tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:3
    ax = nexttile(tl); hold(ax, 'on'); shade(ax);
    plot(ax, t_lp, v(:,k), '-', 'Color', COL_XYZ(k,:), 'DisplayName', 'actual');
    plot(ax, t_lp, v_sp(:,k), '--', 'Color', 0.25 + 0.5 * COL_XYZ(k,:), 'DisplayName', 'setpoint');
    if hasGT, plot(ax, t_gt, v_gt(:,k), GT_STYLE{:}); end
    yline(ax, 0, 'k:', 'HandleVisibility', 'off'); ylabel(ax, lbl{k});
    if k == 1, title(ax, 'Velocity (ENU)'); legend(ax, 'Location', 'best'); end
end
xlabel(ax, '$t$ [s]');

%% ── 圖 4：姿態 ───────────────────────────────────────────────────────────
lbl = {'$\phi$ [deg]', '$\theta$ [deg]', '$\psi$ [deg]'};
fig = newFig('attitude', FIG_W, FIG_H * 1.25);
tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:3
    ax = nexttile(tl); hold(ax, 'on'); shade(ax);
    plot(ax, t_att, rad2deg(eul(:,k)), '-', 'Color', COL_XYZ(k,:), 'DisplayName', 'actual');
    if exist('eul_sp', 'var')
        plot(ax, t_asp, rad2deg(eul_sp(:,k)), '--', 'Color', 0.25 + 0.5 * COL_XYZ(k,:), 'DisplayName', 'setpoint');
    end
    if k == 3 && ~isempty(yaw_sp)
        plot(ax, yaw_sp(:,1), yaw_sp(:,2), '-.', 'Color', [0.4 0.4 0.4], 'DisplayName', 'trajectory yaw');
    end
    if k <= 2 && hasRef
        plot(ax, t_ref, ref(:,k), '-', 'Color', [0 0 0], 'LineWidth', 0.8, 'DisplayName', 'reference $r(t)$');
    end
    if exist('eul_gt', 'var'), plot(ax, t_agt, rad2deg(eul_gt(:,k)), GT_STYLE{:}); end
    yline(ax, 0, 'k:', 'HandleVisibility', 'off'); ylabel(ax, lbl{k});
    if k == 1, title(ax, 'Attitude (FLU / ENU)'); end
    legend(ax, 'Location', 'best');
end
xlabel(ax, '$t$ [s]');

%% ── 圖 5、6：角速度、角加速度 ────────────────────────────────────────────
if exist('w', 'var')
    fig = newFig('angular_velocity', FIG_W, FIG_H * 0.7); ax = axes(fig); hold(ax, 'on'); shade(ax);
    nm = {'$p$', '$q$', '$r$'};
    for k = 1:3, plot(ax, t_av, rad2deg(w(:,k)), 'Color', COL_XYZ(k,:), 'DisplayName', nm{k}); end
    if exist('w_gt', 'var')
        for k = 1:3
            plot(ax, t_avg, rad2deg(w_gt(:,k)), ':', 'Color', 0.5 * COL_XYZ(k,:), 'LineWidth', 1.2, ...
                'DisplayName', [nm{k} ' ground truth']);
        end
    end
    yline(ax, 0, 'k:', 'HandleVisibility', 'off');
    ylabel(ax, '$\omega$ [deg/s]'); xlabel(ax, '$t$ [s]');
    title(ax, 'Angular Velocity (FLU)'); legend(ax, 'Location', 'best');
    if ~isempty(wd)
        fig = newFig('angular_acceleration', FIG_W, FIG_H * 0.7); ax = axes(fig); hold(ax, 'on'); shade(ax);
        nm = {'$\dot p$', '$\dot q$', '$\dot r$'};
        for k = 1:3, plot(ax, t_av, rad2deg(wd(:,k)), 'Color', COL_XYZ(k,:), 'DisplayName', nm{k}); end
        yline(ax, 0, 'k:', 'HandleVisibility', 'off');
        ylabel(ax, '$\dot\omega$ [deg/s$^2$]'); xlabel(ax, '$t$ [s]');
        title(ax, 'Angular Acceleration (FLU)'); legend(ax, 'Location', 'best');
    end
end

%% ── 圖 7：力、力矩 ───────────────────────────────────────────────────────
if ~isempty(Fcmd)
    axF = wrenchFig('force', 'Force (body FLU)', {'$F_x$ [N]', '$F_y$ [N]', '$F_z$ [N]'}, ...
        t_md, Fcmd, Falc, COL_XYZ, shade, FIG_W, FIG_H);
    axM = wrenchFig('torque', 'Torque (body FLU)', {'$M_x$ [N$\cdot$m]', '$M_y$ [N$\cdot$m]', '$M_z$ [N$\cdot$m]'}, ...
        t_md, Mcmd, Malc, COL_XYZ, shade, FIG_W, FIG_H);
    if hasAct
        addRec(axF, t_sv, Frec, COL_XYZ);
        addRec(axM, t_sv, Mrec, COL_XYZ);
    end
    legend(axF(1), 'Location', 'best'); legend(axM(1), 'Location', 'best');
end

%% ── 圖 8–11：推力器、舵機 ────────────────────────────────────────────────
if hasAct
    fig = newFig('thruster_thrust', FIG_W, FIG_H * 0.7); ax = axes(fig); hold(ax, 'on'); shade(ax);
    for i = 1:4, plot(ax, t_sv, f(:,i), 'Color', COL_MOD(i,:), 'DisplayName', MOD_NAME{i}); end
    ylabel(ax, '$f_i$ [N]'); xlabel(ax, '$t$ [s]');
    title(ax, sprintf('Thruster Thrust $f = %.2f + %.3f\\,u$', THRUST_A, THRUST_B));
    legend(ax, 'Location', 'best'); ylim(ax, [0, ceil(THRUST_A + THRUST_B + 0.5)]);

    fig = newFig('servo_angles', FIG_W, FIG_H * 0.7); ax = axes(fig); hold(ax, 'on'); shade(ax);
    for i = 1:4
        plot(ax, t_sv, etaX(:,i), '-',  'Color', COL_MOD(i,:), 'DisplayName', [MOD_NAME{i} ' $\eta_x$']);
        plot(ax, t_sv, etaY(:,i), '--', 'Color', COL_MOD(i,:), 'DisplayName', [MOD_NAME{i} ' $\eta_y$']);
    end
    servoLimits(ax, svMax);
    ylabel(ax, '$\eta$ [deg]'); xlabel(ax, '$t$ [s]');
    title(ax, 'Servo Angles: $\eta_x$ (solid), $\eta_y$ (dashed)'); legend(ax, 'Location', 'bestoutside');

    servoFig('servo_eta_x', '$\eta_x$ [deg]', 'Servo $\eta_x$ (thrust to the right $+$)', t_sv, etaX, COL_MOD, MOD_NAME, svMax, shade, FIG_W, FIG_H);
    servoFig('servo_eta_y', '$\eta_y$ [deg]', 'Servo $\eta_y$ (thrust forward $+$)',     t_sv, etaY, COL_MOD, MOD_NAME, svMax, shade, FIG_W, FIG_H);
end

%% ── 圖 12：電源 ──────────────────────────────────────────────────────────
if has('battery_status')
    bt = win(L.topic.battery_status);  t_bt = tt(bt);
    fig = newFig('power', FIG_W, FIG_H * 0.7); ax = axes(fig); hold(ax, 'on'); shade(ax);
    yyaxis(ax, 'left');  plot(ax, t_bt, bt.voltage_v, '-', 'Color', COL_XYZ(3,:)); ylabel(ax, '$V$ [V]');
    yyaxis(ax, 'right'); plot(ax, t_bt, bt.current_a, '-', 'Color', COL_XYZ(1,:)); ylabel(ax, '$I$ [A]');
    ax.YAxis(1).Color = COL_XYZ(3,:); ax.YAxis(2).Color = COL_XYZ(1,:);
    xlabel(ax, '$t$ [s]'); title(ax, 'Battery');
end

%% ── 圖 13：XY 平面軌跡（顏色 = 時間）────────────────────────────────────
fig = newFig('trajectory_xy', FIG_W * 0.75, FIG_W * 0.65); ax = axes(fig); hold(ax, 'on');
ok = all(isfinite(p_sp(:,1:2)), 2);
plot(ax, p_sp(ok,1), p_sp(ok,2), 'k--', 'LineWidth', 0.9, 'DisplayName', 'setpoint');
colorLine(ax, p(:,1), p(:,2), [], t_lp, 1.6, 'actual');
if hasGT, plot(ax, p_gt(:,1), p_gt(:,2), ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.0, 'DisplayName', 'ground truth'); end
plot(ax, p(1,1), p(1,2), 'o', 'Color', [0 0.6 0], 'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', 'start');
plot(ax, p(end,1), p(end,2), 's', 'Color', [0.8 0 0], 'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', 'end');
colormap(ax, CMAP); cb = colorbar(ax); cb.Label.String = '$t$ [s]'; cb.Label.Interpreter = 'latex';
axis(ax, 'equal'); xlabel(ax, '$x$ (East) [m]'); ylabel(ax, '$y$ (North) [m]');
title(ax, '$XY$ Trajectory'); legend(ax, 'Location', 'best');

%% ── 圖 14：XYZ 三維軌跡（顏色 = 時間）───────────────────────────────────
fig = newFig('trajectory_xyz', FIG_W * 0.8, FIG_W * 0.65); ax = axes(fig); hold(ax, 'on');
ok = all(isfinite(p_sp), 2);
plot3(ax, p_sp(ok,1), p_sp(ok,2), p_sp(ok,3), 'k--', 'LineWidth', 0.9, 'DisplayName', 'setpoint');
colorLine(ax, p(:,1), p(:,2), p(:,3), t_lp, 1.6, 'actual');
if hasGT, plot3(ax, p_gt(:,1), p_gt(:,2), p_gt(:,3), ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.0, 'DisplayName', 'ground truth'); end
plot3(ax, p(1,1), p(1,2), p(1,3), 'o', 'Color', [0 0.6 0], 'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', 'start');
plot3(ax, p(end,1), p(end,2), p(end,3), 's', 'Color', [0.8 0 0], 'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', 'end');
colormap(ax, CMAP); cb = colorbar(ax); cb.Label.String = '$t$ [s]'; cb.Label.Interpreter = 'latex';
axis(ax, 'equal'); grid(ax, 'on'); view(ax, 45, 30);
xlabel(ax, '$x$ (East) [m]'); ylabel(ax, '$y$ (North) [m]'); zlabel(ax, '$z$ (Up) [m]');
title(ax, '$XYZ$ Trajectory'); legend(ax, 'Location', 'best');

%% ── 圖 15：推力器推力方向俯視投影（顏色 = 時間）────────────────────────
% 推力方向單位向量的俯視投影（機頭朝上）：
%   橫軸 = 往右的分量 sin(eta_x)（= -y_FLU），縱軸 = 往前的分量 cos(eta_x) sin(eta_y)（= x_FLU）
% 外框 = sin(svMax)（單軸舵機極限），虛線框 = sin(15 deg)；點在中心 = 推力垂直機身
if hasAct
    fig = newFig('thruster_direction', FIG_W, FIG_W * 0.9);
    tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    E  = sind(svMax);  E15 = sind(15);
    for i = 1:4                                     % 依機身位置：左前、右前、左後、右後
        ax = nexttile(tl, i); hold(ax, 'on');
        rgt = sin(ex(:,i));                         % 往右
        fwd = cos(ex(:,i)) .* sin(ey(:,i));         % 往前
        rectangle(ax, 'Position', [-E -E 2*E 2*E], 'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 0.8);
        rectangle(ax, 'Position', [-E15 -E15 2*E15 2*E15], 'EdgeColor', [0.6 0.6 0.6], 'LineStyle', '--');
        plot(ax, [-E E], [0 0], ':', 'Color', [0.5 0.5 0.5]); plot(ax, [0 0], [-E E], ':', 'Color', [0.5 0.5 0.5]);
        colorLine(ax, rgt, fwd, [], t_sv, 1.2, '');
        plot(ax, mean(rgt), mean(fwd), 'k+', 'MarkerSize', 10, 'LineWidth', 1.5);   % 平均方向
        axis(ax, 'equal'); xlim(ax, [-1.1 1.1] * E); ylim(ax, [-1.1 1.1] * E);
        colormap(ax, CMAP); clim(ax, [min(t_sv) max(t_sv)]);
        title(ax, sprintf('%s: mean $\\eta_x$ %+.1f$^\\circ$, $\\eta_y$ %+.1f$^\\circ$', ...
            MOD_NAME{i}, mean(etaX(:,i)), mean(etaY(:,i))));
        if i >= 3, xlabel(ax, 'right $\sin\eta_x$'); end
        if mod(i, 2) == 1, ylabel(ax, 'forward $\cos\eta_x\sin\eta_y$'); end
    end
    cb = colorbar(ax); cb.Layout.Tile = 'east'; cb.Label.String = '$t$ [s]'; cb.Label.Interpreter = 'latex';
    title(tl, 'Thrust Direction, Top View (nose up)', 'Interpreter', 'latex');
end

%% ── 圖 16：姿態追蹤（參考 / 送出指令 / setpoint / 實際 / 真值）────────────
if hasRef
    lbl = {'$\phi$ [deg]', '$\theta$ [deg]'};
    fig = newFig('attitude_tracking', FIG_W, FIG_H);
    tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ra = interpPrev(t_att, rad2deg(eul(:,1:2)), t_ref, NaN);     % 實際，在 ref 的時間
    for k = 1:2
        ax = nexttile(tl); hold(ax, 'on'); shade(ax);
        plot(ax, t_ref, ref(:,k), 'k-', 'LineWidth', 1.0, 'DisplayName', 'reference $r(t)$');
        plot(ax, t_cmd, cmd(:,k), ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'DisplayName', 'command $r(t+\tau)-b$');
        if exist('eul_sp', 'var')
            plot(ax, t_asp, rad2deg(eul_sp(:,k)), '--', 'Color', 0.25 + 0.5 * COL_XYZ(k,:), 'DisplayName', 'setpoint (after yield)');
        end
        plot(ax, t_att, rad2deg(eul(:,k)), '-', 'Color', COL_XYZ(k,:), 'DisplayName', 'actual');
        if exist('eul_gt', 'var'), plot(ax, t_agt, rad2deg(eul_gt(:,k)), GT_STYLE{:}); end
        ylabel(ax, lbl{k}); xlim(ax, [min(t_ref) max(t_ref)]);
        if k == 1, title(ax, 'Attitude Tracking (FLU)'); legend(ax, 'Location', 'bestoutside'); end
    end
    ax = nexttile(tl); hold(ax, 'on'); shade(ax);
    er = ref - ra;
    plot(ax, t_ref, er(:,1), 'Color', COL_XYZ(1,:), 'DisplayName', '$r_\phi - \phi$');
    plot(ax, t_ref, er(:,2), 'Color', COL_XYZ(2,:), 'DisplayName', '$r_\theta - \theta$');
    yline(ax, 0, 'k:', 'HandleVisibility', 'off');
    ylabel(ax, 'error [deg]'); xlabel(ax, '$t$ [s]'); legend(ax, 'Location', 'bestoutside');
    xlim(ax, [min(t_ref) max(t_ref)]);
    % 統計：誤差 RMS / 平均、剩餘延遲（實際相對參考）
    ok = all(isfinite(er), 2);
    lags = -1:0.01:1;  J = zeros(size(lags));
    for k = 1:numel(lags)
        rs = interpPrev(t_ref, ref, t_ref - lags(k), NaN);   % r(t - lag)
        d = rs - ra;  d = d(all(isfinite(d), 2), :);
        J(k) = sqrt(mean(d(:).^2));
    end
    [Jmin, kb] = min(J);
    fprintf('attitude tracking: RMS roll %.2f / pitch %.2f deg, mean %+.2f / %+.2f deg, residual lag %+.2f s (RMS %.2f deg after removing it)\n', ...
        sqrt(mean(er(ok,1).^2)), sqrt(mean(er(ok,2).^2)), mean(er(ok,1)), mean(er(ok,2)), lags(kb), Jmin);

    %% ── 圖 17：傾斜平面（roll × pitch）───────────────────────────────────
    fig = newFig('attitude_tilt_plane', FIG_W * 0.7, FIG_W * 0.62); ax = axes(fig); hold(ax, 'on');
    plot(ax, ref(:,1), ref(:,2), 'k-', 'LineWidth', 1.0, 'DisplayName', 'reference');
    if exist('eul_sp', 'var')
        ws = t_asp >= min(t_ref) & t_asp <= max(t_ref);
        plot(ax, rad2deg(eul_sp(ws,1)), rad2deg(eul_sp(ws,2)), '--', 'Color', [0.6 0.6 0.6], 'DisplayName', 'setpoint');
    end
    wa = t_att >= min(t_ref) & t_att <= max(t_ref);
    colorLine(ax, rad2deg(eul(wa,1)), rad2deg(eul(wa,2)), [], t_att(wa), 1.4, 'actual');
    yt = prm('PFA_YIELD_TILT', 20);
    th = linspace(0, 2*pi, 181);
    plot(ax, yt * cos(th), yt * sin(th), ':', 'Color', [0.4 0.4 0.4], 'DisplayName', sprintf('yield %.0f$^\\circ$', yt));
    colormap(ax, CMAP); cb = colorbar(ax); cb.Label.String = '$t$ [s]'; cb.Label.Interpreter = 'latex';
    axis(ax, 'equal'); xlabel(ax, '$\phi$ (roll) [deg]'); ylabel(ax, '$\theta$ (pitch, FLU) [deg]');
    title(ax, 'Tilt Plane'); legend(ax, 'Location', 'bestoutside');
end

%% ── 匯出 ─────────────────────────────────────────────────────────────────
if EXPORT_PDF
    if ~exist(OUT_DIR, 'dir'), mkdir(OUT_DIR); end
    [~, base] = fileparts(LOG_FILE);
    figs = findall(groot, 'Type', 'figure');
    for k = 1:numel(figs)
        fn = fullfile(OUT_DIR, sprintf('%s_%s.pdf', base, figs(k).Name));
        exportgraphics(figs(k), fn, 'ContentType', 'vector', 'BackgroundColor', 'white');
        fprintf('Exported: %s\n', fn);
    end
end

%% =========================================================================
%  helpers
%% =========================================================================
function v = getParam(P, name, def)
    if isfield(P, name), v = P.(name); else, v = def; end
end

function vi = interpPrev(t, v, ti, fill)
% zero-order hold；ULog 可能有重複時間戳，先去重
    [t, ia] = unique(t(:), 'last');
    vi = interp1(t, v(ia, :), ti(:), 'previous', fill);
    first = ti(:) < t(1);  vi(first, :) = fill;
end

function a = wrapPi(a)
    a = mod(a + pi, 2*pi) - pi;
end

function e = nedEulerToFlu(q)
% PX4 四元數（NED/FRD，[w x y z]）-> ROS ENU/FLU 的 [roll pitch yaw]（rad）
    w = q(:,1); x = q(:,2); y = q(:,3); z = q(:,4);
    roll  = atan2(2*(w.*x + y.*z), 1 - 2*(x.^2 + y.^2));
    pitch = asin(max(-1, min(1, 2*(w.*y - z.*x))));
    yaw   = atan2(2*(w.*z + x.*y), 1 - 2*(y.^2 + z.^2));
    e = [roll, -pitch, wrapPi(pi/2 - yaw)];
end

function spans = modeSpans(vs, t0, t1, tz)
% 非 OFFBOARD（nav_state ~= 14）或未解鎖的時段 [a b]（已扣 tz）
    bad = vs.nav_state ~= 14 | vs.arming_state ~= 2;
    t = vs.t; spans = zeros(0, 2); k = 1;
    while k <= numel(t)
        if bad(k)
            j = k; while j < numel(t) && bad(j + 1), j = j + 1; end
            b = t(min(j + 1, numel(t)));
            a = max(t(k), t0); b = min(b, t1);
            if b > a, spans(end + 1, :) = [a b] - tz; end %#ok<AGROW>
            k = j + 1;
        else
            k = k + 1;
        end
    end
end

function shadeSpans(ax, spans, on)
    if ~on || isempty(spans), return; end
    for k = 1:size(spans, 1)
        xregion(ax, spans(k,1), spans(k,2), 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.08, ...
            'HandleVisibility', 'off');
    end
end

function h = colorLine(ax, x, y, z, c, lw, name)
% 連續線，顏色沿線依 c（時間）變化：patch 的 EdgeColor = 'interp'
    x = x(:); y = y(:); c = c(:);
    if isempty(z), z = zeros(size(x)); end
    z = z(:);
    ok = isfinite(x) & isfinite(y) & isfinite(z) & isfinite(c);
    x(~ok) = NaN; y(~ok) = NaN; z(~ok) = NaN;      % 斷點保持斷開
    h = patch(ax, [x; NaN], [y; NaN], [z; NaN], [c; NaN], ...
        'EdgeColor', 'interp', 'FaceColor', 'none', 'LineWidth', lw);
    if isempty(name), h.HandleVisibility = 'off'; else, h.DisplayName = name; end
end

function fig = newFig(name, w, h)
    fig = figure('Name', name, 'Units', 'centimeters', 'Color', 'w');
    fig.Position(3:4) = [w h];
end

function servoLimits(ax, lim)
    yline(ax,  lim, 'k--', 'LineWidth', 0.7, 'HandleVisibility', 'off');
    yline(ax, -lim, 'k--', 'LineWidth', 0.7, 'HandleVisibility', 'off');
    yline(ax, 0, 'k:', 'HandleVisibility', 'off');
    ylim(ax, [-1.15 1.15] * lim);
end

function servoFig(name, ylab, ttl, t, eta, col, names, lim, shade, W, H)
    fig = newFig(name, W, H * 0.7); ax = axes(fig); hold(ax, 'on'); shade(ax);
    for i = 1:4, plot(ax, t, eta(:,i), 'Color', col(i,:), 'DisplayName', names{i}); end
    servoLimits(ax, lim);
    ylabel(ax, ylab); xlabel(ax, '$t$ [s]'); title(ax, ttl); legend(ax, 'Location', 'best');
end

function axs = wrenchFig(name, ttl, lbl, t, cmd, alc, col, shade, W, H)
% 控制器要求（實線）、分配後（點線）；推力器重建（虛線）由 addRec 加上
    fig = newFig(name, W, H * 1.25);
    tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    axs = gobjects(3, 1);
    for k = 1:3
        ax = nexttile(tl); hold(ax, 'on'); shade(ax); axs(k) = ax;
        plot(ax, t, cmd(:,k), '-', 'Color', col(k,:), 'DisplayName', 'setpoint');
        if ~isempty(alc)
            plot(ax, t, alc(:,k), ':', 'Color', 0.3 + 0.4 * col(k,:), 'LineWidth', 1.3, 'DisplayName', 'allocated');
        end
        yline(ax, 0, 'k:', 'HandleVisibility', 'off'); ylabel(ax, lbl{k});
        if k == 1, title(ax, ttl); end
    end
    xlabel(ax, '$t$ [s]');
end

function addRec(axs, t, rec, col)
% 由 4 個推力器的推力與舵機角度重建的 wrench（虛線，不含馬達反扭力）
    for k = 1:3
        plot(axs(k), t, rec(:,k), '--', 'Color', 0.25 + 0.5 * col(k,:), 'DisplayName', 'from thrusters');
    end
end
