%% robot3.m （第一段：主脚本，工况C：参数漂移模型演化 - 深度优化版）
if evalin('base','exist(''BJ2_MODE'',''var'')') && evalin('base','BJ2_MODE==1')
    clc; close all;
else
    clear; clc; close all;
end

%% 可复现设置
rng(20260408,'twister');
maxNumCompThreads(1);

%% 日志初始化
BO_LOG = struct('iter',[],'f',[],'L',[],'Lref',[],'w',[],'round',[]);
setappdata(0,'BO_LOG',BO_LOG);
setappdata(0,'BO_ROUND_ID',0);
setappdata(0,'BO_TMP',struct('L',[],'Lref',[]));
PERF_LOG = struct('round',[],'perf',[],'goal',[]);
setappdata(0,'PERF_LOG',PERF_LOG);

%% 0. 数字孪生参数 (工况C专属配置)
twin.online_mode = 'drift_evolution'; 
twin.safety_margin = 0.03;

% 机理模型初始失配
twin.friction_scale_j = [0.75, 1.30, 0.80, 1.25, 0.90, 1.15]; 
twin.friction_min = 0.50; 
twin.friction_max = 2.00; 

% 物理硬约束边界
twin.ground_clearance = 0.08; 
twin.ground_penalty_k = 5e4;  
twin.joint_limit = 0.90 * pi; 
twin.joint_limit_penalty_k = 5e4; 

% 向物理实体注入强时变漂移因子与随机测量噪声
plant.bias = [0.030, -0.025, 0.040, 0.020, -0.015, 0.010]; 
plant.drift_rate = [0.015, -0.010, 0.018, 0.012, -0.008, 0.005]; 
plant.noise_std = 0.015; 
plant.delay_steps = 2;
lambda_fx = 0.5;

%% 1. 机械臂模型
p = [
    0,       0,      0.1265;  % J1
    0,       0.035,  0;       % J2
    0,       0,      0.146;   % J3
    0,      -0.052,  0;       % J4
    0,       0,      0.117;   % J5
    0,       0,      0.0755   % J6
];
ax = [
     0  0  1;  % J1: Yaw
     0  1  0;  % J2: Pitch
     0  1  0;  % J3: Pitch
     0  0  1;  % J4: Roll
     0  1  0;  % J5: Pitch
     0  0  1   % J6: Roll
];
robot = rigidBodyTree('DataFormat','row','MaxNumBodies',6);
parentName = robot.BaseName;
for i = 1:6
    body = rigidBody(sprintf('link%d', i));
    jnt  = rigidBodyJoint(sprintf('Joint%d', i), 'revolute');
    jnt.JointAxis = ax(i,:);
    setFixedTransform(jnt, trvec2tform(p(i,:)));
    body.Joint = jnt;
    addBody(robot, body, parentName);
    parentName = body.Name;
end

%% 1.1 动力学参数
dyn.robot = robot;
dyn.robot.Gravity = [0 0 -9.81];
dyn.enable = true;
dyn.w_tau = 0.02;
dyn.w_power = 0.005;

%% 2. 起点 + 目标点IK
q_start = [0,0,0,0,0,0];
p_target = [0.15, 0.05, 0.20];
T0 = getTransform(robot, q_start, 'link6');
R_target = T0(1:3,1:3);
T_target = eye(4);
T_target(1:3,1:3) = R_target;
T_target(1:3,4) = p_target(:);
[q_end, okIK, infoIK] = ik_solve_with_rbt(robot, q_start, T_target);
if ~okIK
    searchOpt.range = 0.03; searchOpt.step = 0.003; searchOpt.keepSameZFirst = true;
    [q_end, p_target_used, okSearch, ~] = ik_fallback_nearby_search(robot, q_start, p_target, R_target, searchOpt);
    if okSearch, p_target_used = p_target_used; end
else
    p_target_used = p_target;
end
q_end = max(-pi, min(pi, q_end));

%% 2.1 终点与工作空间约束
twin.goal_pos = p_target_used;
twin.goal_tol = 0.005;
twin.goal_penalty_k = 5e4;
twin.ws_x = [-0.40, 0.40];
twin.ws_y = [-0.40, 0.40];
twin.ws_z = [ 0.05, 0.60];
twin.ws_penalty_k = 2e3;

%% 3. 障碍物
q_mid_linear = (q_start + q_end) / 2; 
T_mid = kinematics_URDF_model(q_mid_linear, p, ax);
obs_center = T_mid(1:3, 4)'; 
obs.centers = obs_center;
obs.radii = 0.035; 
obs.num = 1;

%% 4. 优化设置 (核心修改区：收紧搜索范围)
t_total = 4; 
q_via_guess = q_mid_linear;
vars = [
    optimizableVariable('x1',[-5,-1],'Type','real')
    optimizableVariable('x2',[-1,1.4],'Type','real')
    optimizableVariable('x3',[-4,0],'Type','real')
];

% 【优化修改】：将途径点搜索空间收紧至插值中点附近 (±0.8弧度)，禁止出现瞎甩臂的夸张轨迹
limit_range = 0.8; 
lb_q = max(-pi*ones(1,6), q_via_guess - limit_range);
ub_q = min( pi*ones(1,6), q_via_guess + limit_range);

pso_opt_main = optimoptions('particleswarm','Display','off','SwarmSize',70,'MaxIterations',140,...
    'MaxStallIterations',35,'FunctionTolerance',1e-6,'SelfAdjustmentWeight',1.49,...
    'SocialAdjustmentWeight',1.49,'InertiaRange',[0.1 1.1],'UseVectorized',false,'UseParallel',false);
pso_opt_inner = optimoptions('particleswarm','Display','off','SwarmSize',36,'MaxIterations',60,...
    'MaxStallIterations',20,'FunctionTolerance',1e-4,'SelfAdjustmentWeight',1.49,...
    'SocialAdjustmentWeight',1.49,'InertiaRange',[0.1 1.1],'UseVectorized',false,'UseParallel',false);
surr_opt.nLHS  = 30;
surr_opt.nCand = 3000;
surr_opt.seed  = 20260408;

%% 5. 第1轮
disp('================================================================');
disp('第1轮：失配孪生模型预测');
setappdata(0,'BO_ROUND_ID',1);
setappdata(0,'BO_TMP',struct('L',[],'Lref',[]));
[w_best1, ~] = surrogate_opt_lhs_rbf( ...
    q_start, q_end, t_total, p, ax, obs, q_via_guess, twin, dyn, lb_q, ub_q, ...
    pso_opt_inner, vars, lambda_fx, surr_opt, 1);
[q_via_opt1, fval1, exitflag1, out1] = pso_solve_qvia( ...
    @(qv) cost_function_weighted_twin(qv,q_start,q_end,t_total,p,ax,obs,w_best1,twin,dyn), ...
    q_via_guess, lb_q, ub_q, pso_opt_main);

%% 6. 同步校准
disp('正在收集物理实体残差，执行在线反向校准...');
[q_opt_t1, dq_opt_t1, ~] = trajectory_SDPOA(q_start, q_via_opt1, q_end, t_total, 0:1e-4:t_total);
[q_meas_t, dq_meas_t] = simulate_real_plant(q_opt_t1, dq_opt_t1, plant, 1e-4); 
err = sync_real_data(q_opt_t1, dq_opt_t1, q_meas_t, dq_meas_t);
twin = update_twin_params(twin, err);

%% 7. 第2轮
disp('第2轮：基于补偿后孪生模型的二次演化优化');
setappdata(0,'BO_ROUND_ID',2);
setappdata(0,'BO_TMP',struct('L',[],'Lref',[]));
[w_best2, ~] = surrogate_opt_lhs_rbf( ...
    q_start, q_end, t_total, p, ax, obs, q_via_opt1, twin, dyn, lb_q, ub_q, ...
    pso_opt_inner, vars, lambda_fx, surr_opt, 2);

%% 7.1 取最终轨迹
BO_LOG = getappdata(0,'BO_LOG');
idxR2 = find(BO_LOG.round == 2);
if ~isempty(idxR2)
    fLog2 = BO_LOG.f(idxR2); LLog2 = BO_LOG.L(idxR2); wLog2 = BO_LOG.w(idxR2,:);
    score = lambda_fx*normalize01(fLog2) + (1-lambda_fx)*normalize01(LLog2);
    [~, kSel] = min(score); w_final = wLog2(kSel,:);
else
    w_final = w_best2;
end
[q_via_final, fval_final, ef_final, ~] = pso_solve_qvia( ...
    @(qv) cost_function_weighted_twin(qv,q_start,q_end,t_total,p,ax,obs,w_final,twin,dyn), ...
    q_via_opt1, lb_q, ub_q, pso_opt_main);

%% 8. 生成最终轨迹并导出
t_sample = 1e-4; t = 0:t_sample:t_total;
[q_actual_t, dq_actual_t, ddq_actual_t] = trajectory_SDPOA(q_start, q_via_final, q_end, t_total, t);
m_final = evaluate_metrics_twin(q_via_final,q_start,q_end,t_total,p,ax,obs,twin,dyn);
disp('主流程完成。生成绘图...');

%% 提取绘图变量
N = numel(t);
[q_bad_t, ~, ~] = trajectory_SDPOA(q_start, q_mid_linear, q_end, t_total, t);
ds_idx = 1:50:N; ds_N = length(ds_idx);
ds_pos_opt = zeros(ds_N,3); ds_pos_bad = zeros(ds_N,3);
for i = 1:ds_N
    k = ds_idx(i);
    Ttmp1 = kinematics_URDF_model(q_actual_t(k,:), p, ax); ds_pos_opt(i,:) = Ttmp1(1:3,4)';
    Ttmp2 = kinematics_URDF_model(q_bad_t(k,:), p, ax);    ds_pos_bad(i,:) = Ttmp2(1:3,4)';
end
ds_t = t(ds_idx);
pos_actual_t = [interp1(ds_t, ds_pos_opt(:,1), t, 'spline')', interp1(ds_t, ds_pos_opt(:,2), t, 'spline')', interp1(ds_t, ds_pos_opt(:,3), t, 'spline')'];
pos_bad_t = [interp1(ds_t, ds_pos_bad(:,1), t, 'spline')', interp1(ds_t, ds_pos_bad(:,2), t, 'spline')', interp1(ds_t, ds_pos_bad(:,3), t, 'spline')'];

%% ===================== 13. 图表输出 =====================
figure('Name','最优轨迹关节角度变化','Position',[50,550,450,350]);
plot(t, q_actual_t*180/pi, 'LineWidth',1.5);
title('工况C：漂移补偿后的关节角度'); xlabel('时间 (s)'); ylabel('角度 (°)');
legend('J1','J2','J3','J4','J5','J6','Location','best'); grid on;

figure('Name','轨迹三视图投影','Position',[100, 200, 1200, 400]);
th = linspace(0, 2*pi, 180); 
subplot(1,3,1);
plot(pos_bad_t(:,1),pos_bad_t(:,2),'r--','LineWidth',1); hold on;
plot(pos_actual_t(:,1),pos_actual_t(:,2),'b-','LineWidth',2);
plot(obs.centers(1,1)+obs.radii(1)*cos(th), obs.centers(1,2)+obs.radii(1)*sin(th), 'k-', 'LineWidth',1.5);
title('XY 平面投影'); xlabel('X (m)'); ylabel('Y (m)'); grid on; axis equal;
legend('线性插值轨迹','补偿后最优轨迹','障碍物','Location','best');

subplot(1,3,2);
plot(pos_bad_t(:,2),pos_bad_t(:,3),'r--','LineWidth',1); hold on;
plot(pos_actual_t(:,2),pos_actual_t(:,3),'g-','LineWidth',2);
plot(obs.centers(1,2)+obs.radii(1)*cos(th), obs.centers(1,3)+obs.radii(1)*sin(th), 'k-', 'LineWidth',1.5);
title('YZ 平面投影'); xlabel('Y (m)'); ylabel('Z (m)'); grid on; axis equal;

subplot(1,3,3);
plot(pos_bad_t(:,1),pos_bad_t(:,3),'r--','LineWidth',1); hold on;
plot(pos_actual_t(:,1),pos_actual_t(:,3),'m-','LineWidth',2);
plot(obs.centers(1,1)+obs.radii(1)*cos(th), obs.centers(1,3)+obs.radii(1)*sin(th), 'k-', 'LineWidth',1.5);
title('XZ 平面投影'); xlabel('X (m)'); ylabel('Z (m)'); grid on; axis equal;

%% ===================== 14. 3D动画 =====================
h_fig3 = figure('Name','在线代理优化最优轨迹整机动画','Position',[1050,200,750,700]);
ax_3d = axes('Parent',h_fig3); hold(ax_3d,'on');
plot3(ax_3d,pos_bad_t(:,1),pos_bad_t(:,2),pos_bad_t(:,3),'r--','LineWidth',1.5,'DisplayName','未优化线性轨迹');
plot3(ax_3d,pos_actual_t(:,1),pos_actual_t(:,2),pos_actual_t(:,3),'b-','LineWidth',2.5,'DisplayName','演化后最优轨迹');
[sox,soy,soz] = sphere(28);
surf(ax_3d, obs.centers(1,1)+obs.radii(1)*sox, obs.centers(1,2)+obs.radii(1)*soy, obs.centers(1,3)+obs.radii(1)*soz, ...
    'FaceColor',[0.1 0.1 0.1],'EdgeColor','none','FaceAlpha',0.25,'DisplayName','球障碍物');
plot3(ax_3d,p_target_used(1),p_target_used(2),p_target_used(3),'kp','MarkerSize',12,'MarkerFaceColor','y','DisplayName','目标点');
title(ax_3d,'模型演化：3D 轨迹与整机动画');
xlabel(ax_3d,'X轴(m)'); ylabel(ax_3d,'Y轴(m)'); zlabel(ax_3d,'Z轴(m)');
grid(ax_3d,'on'); axis(ax_3d,'equal'); view(ax_3d,135,30);
xlim(ax_3d,[-0.4 0.4]); ylim(ax_3d,[-0.4 0.4]); zlim(ax_3d,[0 0.6]);
legend(ax_3d,'Location','northeast');
show(robot,q_actual_t(1,:),'Parent',ax_3d,'Frames','off','Visuals','on');

%% ===================== 16. 最优指标J曲线图 =====================
plot_J_convergence_from_log();

%% ================================ 函数区 ================================
function [xbest, fbest, exitflag, out] = pso_solve_qvia(fun, x0, lb, ub, psoOpt)
nvars = numel(x0); swarmN = psoOpt.SwarmSize; S = repmat(x0(:)', swarmN, 1);
if swarmN > 1
    jitter = 0.20 * (2*rand(swarmN-1, nvars)-1); S(2:end,:) = S(2:end,:) + jitter;
end
S = min(repmat(ub,swarmN,1), max(repmat(lb,swarmN,1), S));
psoOpt2 = psoOpt; psoOpt2.InitialSwarmMatrix = S;
[xbest, fbest, exitflag, out] = particleswarm(fun, nvars, lb, ub, psoOpt2);
end

function y = normalize01(x)
xmin = min(x); xmax = max(x);
if abs(xmax-xmin)<1e-12, y = zeros(size(x)); else, y = (x-xmin)/(xmax-xmin); end
end

function [q_sol, ok, info] = ik_solve_with_rbt(robot, q_seed, tformTarget)
persistent IK_OBJ
if isempty(IK_OBJ), IK_OBJ = inverseKinematics('RigidBodyTree', robot); end
weights = [0.1 0.1 0.1 1 1 1];
seeds = [q_seed; q_seed + [ 0.25 0.15 -0.15 0.08 -0.08 0.15]; q_seed + [-0.25 -0.15 0.15 -0.08 0.08 -0.15]];
seeds = max(-pi,min(pi,seeds)); ok = false; bestErr = inf; q_sol = q_seed; bestStatus = '';
for i = 1:size(seeds,1)
    [q_i, solInfo] = IK_OBJ('link6', tformTarget, weights, seeds(i,:));
    T_i = getTransform(robot, q_i, 'link6'); posErr = norm(T_i(1:3,4)-tformTarget(1:3,4));
    if posErr < bestErr, bestErr = posErr; q_sol = q_i; bestStatus = solInfo.Status; end
    if strcmpi(solInfo.Status,'success') && posErr < 1e-3, ok = true; q_sol = q_i; bestErr = posErr; bestStatus = solInfo.Status; break; end
end
if ~ok && bestErr < 2e-3, ok = true; end
info.posErr = bestErr; info.status = bestStatus;
end

function [q_best, p_used, ok, info] = ik_fallback_nearby_search(robot, q_seed, p_target, R_target, opt)
if opt.step > opt.range, opt.step = opt.range/3; end
r = opt.range; step = opt.step; gridv = -r:step:r; cand = [];
if opt.keepSameZFirst, for dx = gridv, for dy = gridv, cand = [cand; p_target + [dx,dy,0]]; end, end, end
for dx = gridv, for dy = gridv, for dz = gridv, cand = [cand; p_target + [dx,dy,dz]]; end, end, end
cand = unique(round(cand,6),'rows');
ok = false; q_best = q_seed; p_used = p_target; bestDp = inf; bestPosErr = inf;
for i = 1:size(cand,1)
    p_try = cand(i,:); T_try = eye(4); T_try(1:3,1:3)=R_target; T_try(1:3,4)=p_try(:);
    [q_i, ok_i, info_i] = ik_solve_with_rbt(robot, q_seed, T_try);
    if ~ok_i, continue; end
    dp = norm(p_try - p_target); posErr = info_i.posErr;
    better = (dp < bestDp - 1e-12) || (abs(dp-bestDp)<=1e-12 && posErr < bestPosErr);
    if better
        bestDp = dp; bestPosErr = posErr; q_best = q_i; p_used = p_try; ok = true;
        if dp < 1e-12 && posErr < 1e-3, break; end
    end
end
info.bestDp = bestDp; info.bestPosErr = bestPosErr;
end

function [w_best, surr] = surrogate_opt_lhs_rbf(q_start, q_end, t_total, p, ax, obs, q_via_guess, twin, dyn, lb_q, ub_q, pso_opt_inner, vars, lambda_fx, surr_opt, round_id)
lbx = [vars(1).Range(1), vars(2).Range(1), vars(3).Range(1)]; ubx = [vars(1).Range(2), vars(2).Range(2), vars(3).Range(2)];
dim = 3; rng(surr_opt.seed + round_id, 'twister');
Xn = lhsdesign(surr_opt.nLHS, dim, 'criterion','maximin','iterations',50);
X  = lbx + Xn.*(ubx-lbx); Y = zeros(surr_opt.nLHS,1); L = zeros(surr_opt.nLHS,1); Lref = zeros(surr_opt.nLHS,1); W = zeros(surr_opt.nLHS,3);
for i = 1:surr_opt.nLHS
    x = X(i,:); w = 10.^x; W(i,:) = w;
    [qv_opt, ~] = pso_solve_qvia(@(qv) cost_function_weighted_twin(qv,q_start,q_end,t_total,p,ax,obs,w,twin,dyn), q_via_guess, lb_q, ub_q, pso_opt_inner);
    m = evaluate_metrics_twin(qv_opt,q_start,q_end,t_total,p,ax,obs,twin,dyn);
    
    % 【优化修改】：大幅提高路径长度惩罚，像橡皮筋一样把机械臂拉近障碍物
    alpha = [0.20, 0.20, 0.40, 0.20]; 
    
    Jacc = m.J_acc/(m.J_acc+1); Jobs = m.J_obs/(m.J_obs+1); Jlen = m.J_len/(m.J_len+1); Jeng = m.J_energy/(m.J_energy+1);
    Jtau = m.J_tau/(m.J_tau+1); Jpow = m.J_power/(m.J_power+1); Jgnd = m.J_ground/(m.J_ground+1); Jlim = m.J_joint_limit/(m.J_joint_limit+1);
    Jgol = m.J_goal/(m.J_goal+1); Jwsp = m.J_workspace/(m.J_workspace+1);
    P_safe = 0;
    if isfield(m,'d_min_surface') && isfinite(m.d_min_surface) && m.d_min_surface < twin.safety_margin
        P_safe = 2e3*(twin.safety_margin-m.d_min_surface)^2;
    end
    f = alpha(1)*Jacc + alpha(2)*Jobs + alpha(3)*Jlen + alpha(4)*Jeng + ...
        dyn.w_tau*Jtau + dyn.w_power*Jpow + ...
        1.5*Jgnd + 1.5*Jlim + 0.60*Jgol + 0.40*Jwsp + P_safe;
    Y(i) = f; L(i) = m.L_withObs; Lref(i) = m.L_ref_noObs;
    push_bo_log(round_id, i, f, L(i), Lref(i), w);
end
goal_rbf = 1e-5; spread_rbf = 1.0; max_neuron = 200; df_add = 1;
[net, ~] = newrb_with_hist(X', Y', goal_rbf, spread_rbf, max_neuron, df_add);
Xcand_n = rand(surr_opt.nCand, dim); Xcand = lbx + Xcand_n.*(ubx-lbx); Yhat = sim(net, Xcand')';
K = min(60, size(Xcand,1)); [~, idxSort] = sort(Yhat, 'ascend'); idxTop = idxSort(1:K);
f_top = Yhat(idxTop); L_top = zeros(K,1);
for k = 1:K, xt = Xcand(idxTop(k),:); [~, nn] = min(sum((X-xt).^2,2)); L_top(k) = L(nn); end
fn = normalize01(f_top); Ln = normalize01(L_top); score = lambda_fx*fn + (1-lambda_fx)*Ln;
[~, kk] = min(score); x_best = Xcand(idxTop(kk),:); w_best = 10.^x_best;
surr.net = net; surr.w_best = w_best;
end

function push_bo_log(round_id, iter_id, fval, L, Lref, w)
BO_LOG = getappdata(0,'BO_LOG');
BO_LOG.round(end+1,1)=round_id; BO_LOG.iter(end+1,1)=iter_id; BO_LOG.f(end+1,1)=fval;
BO_LOG.L(end+1,1)=L; BO_LOG.Lref(end+1,1)=Lref; BO_LOG.w(end+1,:)=w;
setappdata(0,'BO_LOG',BO_LOG);
end

function J = cost_function_weighted_twin(q_via,q_start,q_end,t_total,p,ax,obs,w,twin,dyn)
m = evaluate_metrics_twin(q_via,q_start,q_end,t_total,p,ax,obs,twin,dyn);
J = w(1)*m.J_acc + w(2)*m.J_obs + w(3)*m.J_len + ...
    0.05*m.J_energy + dyn.w_tau*m.J_tau + dyn.w_power*m.J_power + ...
    1.0*m.J_ground + 1.0*m.J_joint_limit + 1.0*m.J_goal + 1.0*m.J_workspace;
end

function m = evaluate_metrics_twin(q_via,q_start,q_end,t_total,p,ax,obs,twin,dyn)
t_eval = linspace(0,t_total,20);
[q_eval,dq_eval,ddq_eval] = trajectory_SDPOA(q_start,q_via,q_end,t_total,t_eval);
m.J_acc = sum(sum(ddq_eval.^2));
m.J_energy = sum(sum((dq_eval.^2).*twin.friction_scale_j));
q_abs = abs(q_eval); lim_violation = max(0,q_abs - twin.joint_limit);
m.J_joint_limit = twin.joint_limit_penalty_k * sum(lim_violation(:).^2);
ee = zeros(length(t_eval),3); J_ground = 0; d_min_surface = inf; J_tau = 0; J_power = 0;
for i = 1:length(t_eval)
    q_i = q_eval(i,:); dq_i = dq_eval(i,:); ddq_i = ddq_eval(i,:);
    T_i = kinematics_URDF_model(q_i,p,ax); ee(i,:) = T_i(1:3,4)';
    jointPos = forward_all_joint_positions(q_i,p,ax);
    z_violation = max(0,twin.ground_clearance - jointPos(:,3));
    J_ground = J_ground + twin.ground_penalty_k * sum(z_violation.^2);
    if dyn.enable, tau_i = inverseDynamics(dyn.robot, q_i, dq_i, ddq_i); J_tau = J_tau + sum(tau_i.^2); J_power = J_power + sum(abs(tau_i .* dq_i)); end
end
J_obs = 0;
if isfield(obs,'num') && obs.num > 0
    k_safe = 2e3; k_coll = 1e5;
    for i = 1:size(ee,1)
        p_i = ee(i,:);
        for j = 1:obs.num
            c = obs.centers(j,:); r = obs.radii(j); d = norm(p_i - c) - r;
            d_min_surface = min(d_min_surface, d);
            if d < twin.safety_margin, J_obs = J_obs + k_safe * (twin.safety_margin - d)^2; end
            if d < 0, J_obs = J_obs + k_coll * (0 - d)^2; end
        end
    end
end
seg = diff(ee,1,1); L_withObs = sum(sqrt(sum(seg.^2,2))); L_ref_noObs = norm(ee(end,:)-ee(1,:));
J_len = L_withObs + 8*max(0,L_withObs - 1.2*L_ref_noObs)^2;
p_end_traj = ee(end,:); goal_err = norm(p_end_traj - twin.goal_pos);
goal_excess = max(0, goal_err - twin.goal_tol); J_goal = twin.goal_penalty_k * goal_excess^2;
x = ee(:,1); y = ee(:,2); z = ee(:,3);
vx = max(0, twin.ws_x(1)-x) + max(0, x-twin.ws_x(2)); vy = max(0, twin.ws_y(1)-y) + max(0, y-twin.ws_y(2)); vz = max(0, twin.ws_z(1)-z) + max(0, z-twin.ws_z(2));
J_workspace = twin.ws_penalty_k * sum(vx.^2 + vy.^2 + vz.^2);
m.J_obs = J_obs; m.J_len = J_len; m.J_ground = J_ground; m.L_withObs = L_withObs;
m.L_ref_noObs = L_ref_noObs; m.d_min_surface = d_min_surface; m.J_tau = J_tau;
m.J_power = J_power; m.J_goal = J_goal; m.J_workspace = J_workspace; m.goal_err = goal_err;
end

function jointPos = forward_all_joint_positions(q,p,ax)
T = eye(4); jointPos = zeros(6,3);
for i = 1:6
    T_trans = eye(4); T_trans(1:3,4)=p(i,:)'; u = ax(i,:); u = u/norm(u); ux=u(1); uy=u(2); uz=u(3);
    th=q(i); c=cos(th); s=sin(th); v=1-c;
    R = [ux^2*v+c, ux*uy*v-uz*s, ux*uz*v+uy*s; ux*uy*v+uz*s, uy^2*v+c, uy*uz*v-ux*s; ux*uz*v-uy*s, uy*uz*v+ux*s, uz^2*v+c];
    T_rot = eye(4); T_rot(1:3,1:3)=R; T = T*(T_trans*T_rot); jointPos(i,:) = T(1:3,4)';
end
end

function [q_meas_t,dq_meas_t] = simulate_real_plant(q_cmd_t,dq_cmd_t,plant,dt)
N = size(q_cmd_t,1); q_meas_t = q_cmd_t; dq_meas_t = dq_cmd_t;
for i = 1:N
    tt = (i-1)*dt; q_meas_t(i,:) = q_cmd_t(i,:) + plant.bias + plant.drift_rate*tt + plant.noise_std*randn(1,6);
end
dq_meas_t(2:end,:) = diff(q_meas_t,1,1)/dt; dq_meas_t(1,:) = dq_meas_t(2,:);
d = max(0,plant.delay_steps);
if d>0 && d<N
    q_meas_t = [repmat(q_meas_t(1,:),d,1); q_meas_t(1:end-d,:)]; dq_meas_t = [repmat(dq_meas_t(1,:),d,1); dq_meas_t(1:end-d,:)];
end
end

function err = sync_real_data(q_twin,dq_twin,q_real,dq_real)
dq = q_twin-q_real; ddq = dq_twin-dq_real;
err.q_rmse_global = sqrt(mean(dq(:).^2)); err.dq_rmse_global = sqrt(mean(ddq(:).^2));
err.q_rmse_joint = sqrt(mean(dq.^2,1)); err.dq_rmse_joint = sqrt(mean(ddq.^2,1));
end

function twin = update_twin_params(twin,err)
g = 0.25; lambda = 0.05; ref_dq = 0.03;
e = err.dq_rmse_joint - ref_dq; scale = max(1e-6, median(abs(e))); delta = g*(e/scale);
twin.friction_scale_j = twin.friction_scale_j + delta - lambda*(twin.friction_scale_j-1.0);
twin.friction_scale_j = min(twin.friction_max, max(twin.friction_min, twin.friction_scale_j));
end

function T_end = kinematics_URDF_model(q,p,ax)
T_end = eye(4);
for i = 1:6
    T_trans = eye(4); T_trans(1:3,4)=p(i,:)'; u = ax(i,:); u = u/norm(u); ux=u(1); uy=u(2); uz=u(3);
    th=q(i); c=cos(th); s=sin(th); v=1-c;
    R = [ux^2*v+c, ux*uy*v-uz*s, ux*uz*v+uy*s; ux*uy*v+uz*s, uy^2*v+c, uy*uz*v-ux*s; ux*uz*v-uy*s, uy*uz*v+ux*s, uz^2*v+c];
    T_rot = eye(4); T_rot(1:3,1:3)=R; T_end = T_end*(T_trans*T_rot);
end
end

function [q_ref,dq_ref,ddq_ref] = trajectory_SDPOA(q_start,q_via,q_end,t_total,t)
N = length(t); q_ref = zeros(N,6); dq_ref = zeros(N,6); ddq_ref = zeros(N,6);
t_m = t_total/2; t_f = t_total;
M = [1,0,0,0,0,0,0; 0,1,0,0,0,0,0; 0,0,2,0,0,0,0;
     1,t_m,t_m^2,t_m^3,t_m^4,t_m^5,t_m^6; 1,t_f,t_f^2,t_f^3,t_f^4,t_f^5,t_f^6;
     0,1,2*t_f,3*t_f^2,4*t_f^3,5*t_f^4,6*t_f^5; 0,0,2,6*t_f,12*t_f^2,20*t_f^3,30*t_f^4];
for j = 1:6
    B = [q_start(j);0;0;q_via(j);q_end(j);0;0]; C = M\B;
    q_ref(:,j)   = C(1)+C(2)*t+C(3)*t.^2+C(4)*t.^3+C(5)*t.^4+C(6)*t.^5+C(7)*t.^6;
    dq_ref(:,j)  = C(2)+2*C(3)*t+3*C(4)*t.^2+4*C(5)*t.^3+5*C(6)*t.^4+6*C(7)*t.^5;
    ddq_ref(:,j) = 2*C(3)+6*C(4)*t+12*C(5)*t.^2+20*C(6)*t.^3+30*C(7)*t.^4;
end
end

function plot_J_convergence_from_log()
BO_LOG = getappdata(0,'BO_LOG');
if isempty(BO_LOG) || ~isfield(BO_LOG,'iter') || isempty(BO_LOG.iter), return; end
figure('Name','最优指标J收敛曲线','Position',[250,120,1060,420]);
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
nexttile; hold on; grid on; idx1 = find(BO_LOG.round==1);
if ~isempty(idx1)
    J1 = BO_LOG.f(idx1); i1=(1:numel(J1))';
    plot(i1, J1, '-o', 'Color',[0.55 0.55 0.55], 'DisplayName','失配模型采样');
    plot(i1, cummin(J1), '-s', 'LineWidth',2.0, 'Color',[0.00 0.45 0.74], 'DisplayName','失配收敛底线');
end
% 统一Y轴，方便直观对比
ylim_min = min(BO_LOG.f) * 0.95; ylim_max = max(BO_LOG.f) * 1.05; ylim([ylim_min, ylim_max]);
title('第1轮：失配孪生模型预测'); xlabel('样本编号'); ylabel('综合代价值 Score (J)'); legend;

nexttile; hold on; grid on; idx2 = find(BO_LOG.round==2);
if ~isempty(idx2)
    J2 = BO_LOG.f(idx2); i2=(1:numel(J2))';
    plot(i2, J2, '-o', 'Color',[0.60 0.60 0.60], 'DisplayName','校准后采样');
    plot(i2, cummin(J2), '-s', 'LineWidth',2.0, 'Color',[0.85 0.33 0.10], 'DisplayName','演化后更优收敛');
end
ylim([ylim_min, ylim_max]); % 同步Y轴
title('第2轮：残差补偿后的模型演化'); xlabel('样本编号'); ylabel('综合代价值 Score (J)'); legend;
end

function [net, perf_hist] = newrb_with_hist(P, T, goal, spread, mn, df)
if nargin < 6, df = 1; end
perf_hist = []; k = max(1,df);
[~, net] = evalc('newrb(P, T, goal, spread, k, df)'); Y = sim(net, P); sse = sum((T(:)-Y(:)).^2); perf_hist(end+1,1) = sse;
while (sse > goal) && (k < mn)
    k = min(k + df, mn); [~, net_new] = evalc('newrb(P, T, goal, spread, k, df)');
    Y = sim(net_new, P); sse_new = sum((T(:)-Y(:)).^2); perf_hist(end+1,1) = sse_new;
    net = net_new; sse = sse_new;
end
end