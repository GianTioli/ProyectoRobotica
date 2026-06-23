%% SCARA HMI: seleccion de hoyos -> trayectoria -> CSV
clear; clc; close all;

%% Convencion de coordenadas
% Origen: esquina izquierda/frontal de la mesa, sobre la superficie.
% x: hacia la derecha, y: hacia el fondo, z: hacia arriba.

%% Parametros ajustables de montaje y herramienta [mm]
base_x_desde_izquierda_mm = 200.0;
base_y_desde_frente_mm = 0.0;

z_eje_retraido_mm = 345.0;
z_eje_extendido_mm = 495.0;
tool_tip_offset_down_mm = 275.0;
sentido_extension_z = 1.0;

%% Robot
robot.L1 = 140.0;
robot.L2 = 120.0;
robot.base = [base_x_desde_izquierda_mm, base_y_desde_frente_mm];
robot.codo = "codo_abajo";

robot.home_theta_deg = [0, 0];

robot.theta_limits_deg = [
    -180.0, 180.0;
    -120.0, 120.0
];

robot.enforce_joint_limits = true;

%% Actuador lineal
actuator.min_mm = 0.0;
actuator.max_mm = z_eje_extendido_mm - z_eje_retraido_mm;
actuator.z_axis_retracted_mm = z_eje_retraido_mm;
actuator.tool_tip_offset_down_mm = tool_tip_offset_down_mm;
actuator.extension_sign = sentido_extension_z;
actuator.ms_per_mm = 15000.0/127.0;

%% Mesa, tubo y placa: coordenadas fijas respecto a la mesa [mm]
mesa.length = 400.0;
mesa.width = 150.0;

placa.width = 85.4;
placa.height = 127.6;
placa.x0 = mesa.length - placa.width;
placa.y0 = 0.0;
placa.hole_diameter = 30.0;
placa.hole_pitch = 39.0;
placa.hole_margin_x = 23.2;
placa.hole_margin_y = 24.8;
placa.hole_rim_z = 70.0;

tubo.center = [64.6, 130.0];
tubo.diameter = 30.0;
tubo.rim_z = 115.0;
tubo.insert_depth = 35.0;

task.safe_tcp_z = 90.0;
task.tube_tcp_z = tubo.rim_z - tubo.insert_depth;
task.plate_tcp_z = placa.hole_rim_z;
task.dwell_asp_s = 0.15;
task.dwell_disp_s = 0.15;

%% Planeacion
cfg.dt_validacion = 0.025;      % Solo se usa internamente para validar velocidades.
cfg.num_muestras_csv = 400;    % Filas totales aproximadas/exactas del CSV.
cfg.tf_min = 0.60;
cfg.v_xyz_max = 80.0;
cfg.v_d_max = 1000.0/actuator.ms_per_mm;
cfg.v_th_max = deg2rad(100.0);
cfg.robot = robot;
cfg.actuator = actuator;

%% Pozos fijos de la placa
pozos_x = placa.x0 + [placa.hole_margin_x, placa.hole_margin_x + placa.hole_pitch];
pozos_y = placa.y0 + [placa.hole_margin_y, placa.hole_margin_y + placa.hole_pitch, ...
    placa.hole_margin_y + 2.0*placa.hole_pitch];

pozos = [
    pozos_x(1), pozos_y(1)  % 1 inferior izquierda
    pozos_x(2), pozos_y(1)  % 2 inferior derecha
    pozos_x(1), pozos_y(2)  % 3 medio izquierda
    pozos_x(2), pozos_y(2)  % 4 medio derecha
    pozos_x(1), pozos_y(3)  % 5 superior izquierda
    pozos_x(2), pozos_y(3)  % 6 superior derecha
];

% Seleccion HMI.
etiquetas_pozos = { ...
    '1 inferior izquierda', ...
    '2 inferior derecha', ...
    '3 medio izquierda', ...
    '4 medio derecha', ...
    '5 superior izquierda', ...
    '6 superior derecha'};

orden_seguro = [5 3 1 2 4 6];
modo = menu('Modo de operacion', ...
    'Todos los hoyos', ...
    'Un solo hoyo', ...
    'Subgrupo de hoyos');

if modo == 0
    error('Operacion cancelada por el usuario.');
elseif modo == 1
    orden = orden_seguro;
    modo_txt = 'Todos los hoyos';
elseif modo == 2
    [idx, ok] = listdlg('PromptString','Seleccione un hoyo:', ...
        'SelectionMode','single', ...
        'ListString', etiquetas_pozos);
    if ~ok || isempty(idx)
        error('Operacion cancelada por el usuario.');
    end
    orden = idx;
    modo_txt = 'Un solo hoyo';
elseif modo == 3
    [idx, ok] = listdlg('PromptString','Seleccione los hoyos:', ...
        'SelectionMode','multiple', ...
        'ListString', etiquetas_pozos);
    if ~ok || isempty(idx)
        error('Operacion cancelada por el usuario.');
    end
    orden = orden_seguro(ismember(orden_seguro, idx));
    modo_txt = 'Subgrupo de hoyos';
end

pozos_orden = pozos(orden,:);

%% Construccion de puntos y trayectoria
[P, names, dwell] = construirPuntos(robot, actuator, tubo, pozos_orden, task, orden);
validarPuntos(P, cfg);

traj = planificarSegmentosSCARA(P, dwell, names, cfg);
Q = traj.Q;

%% CSV final
% Columna 1: angulo del primer motor [deg]
% Columna 2: angulo del segundo motor [deg]
% Columna 3: desplazamiento del actuador lineal [mm]
tabla_csv = [rad2deg(Q(:,2)), rad2deg(Q(:,3)), Q(:,1)];

CSV_OUT = 'trayectoria_robot.csv';
writematrix(tabla_csv, CSV_OUT);

fprintf('Modo seleccionado: %s\n', modo_txt);
fprintf('Hoyos seleccionados: %s\n', mat2str(orden));
fprintf('CSV generado: %s\n', CSV_OUT);
fprintf('Columnas: theta1_deg, theta2_deg, desplazamiento_actuador_mm\n');
fprintf('Filas exportadas: %d\n', size(tabla_csv,1));
fprintf('Muestras configuradas: %d\n', cfg.num_muestras_csv);

%% =========================================================
% FUNCIONES LOCALES
% =========================================================

function [P, names, dwell] = construirPuntos(robot, actuator, tubo, pozos, task, pozo_ids)

    if nargin < 6
        pozo_ids = 1:size(pozos,1);
    end

    q_home_low = [actuator.min_mm, deg2rad(robot.home_theta_deg)];
    home_low = fkSCARA(q_home_low, robot, actuator);
    home_safe = home_low;
    home_safe(3) = task.safe_tcp_z;

    P = home_low;
    names = {'Home retraido'};
    dwell = 0;

    [P, names, dwell] = agregarPunto(P, names, dwell, home_safe, 'Home seguro', 0);
    [P, names, dwell] = agregarPunto(P, names, dwell, ...
        [tubo.center, task.safe_tcp_z], 'Sobre tubo', 0);
    [P, names, dwell] = agregarPunto(P, names, dwell, ...
        [tubo.center, task.tube_tcp_z], 'Aspiracion', task.dwell_asp_s);
    [P, names, dwell] = agregarPunto(P, names, dwell, ...
        [tubo.center, task.safe_tcp_z], 'Salida tubo', 0);

    for i = 1:size(pozos,1)
        [P, names, dwell] = agregarPunto(P, names, dwell, ...
            [pozos(i,:), task.safe_tcp_z], sprintf('Sobre pozo %d', pozo_ids(i)), 0);
        [P, names, dwell] = agregarPunto(P, names, dwell, ...
            [pozos(i,:), task.plate_tcp_z], sprintf('Dispensacion pozo %d', pozo_ids(i)), task.dwell_disp_s);
        [P, names, dwell] = agregarPunto(P, names, dwell, ...
            [pozos(i,:), task.safe_tcp_z], sprintf('Salida pozo %d', pozo_ids(i)), 0);
    end

    [P, names, dwell] = agregarPunto(P, names, dwell, home_safe, 'Retorno home seguro', 0);
    [P, names, dwell] = agregarPunto(P, names, dwell, home_low, 'Home final retraido', 0);
end

function [P, names, dwell] = agregarPunto(P, names, dwell, point, name, dwell_s)

    P(end+1,:) = point;
    names{end+1} = name;
    dwell(end+1) = dwell_s;
end

function validarPuntos(P, cfg)

    z_limits = sort([
        tcpZFromExtension(cfg.actuator.min_mm, cfg.actuator), ...
        tcpZFromExtension(cfg.actuator.max_mm, cfg.actuator)
    ]);

    for i = 1:size(P,1)
        radial = norm(P(i,1:2) - cfg.robot.base);

        if radial < abs(cfg.robot.L1-cfg.robot.L2) || radial > cfg.robot.L1+cfg.robot.L2
            error('Punto %d fuera del alcance planar: radio %.2f mm.', i, radial);
        end

        if P(i,3) < z_limits(1) || P(i,3) > z_limits(2)
            error('Punto %d fuera del alcance vertical [%.2f, %.2f] mm.', ...
                i, z_limits(1), z_limits(2));
        end
    end
end

function validarLimitesArticulares(Q, robot, names, context)

    if ~isfield(robot, 'enforce_joint_limits') || ~robot.enforce_joint_limits
        return;
    end

    limits = robot.theta_limits_deg;
    theta_deg = rad2deg(Q(:,2:3));
    tol_deg = 1e-6;

    for joint = 1:2
        theta_min = limits(joint,1);
        theta_max = limits(joint,2);
        outside = theta_deg(:,joint) < theta_min - tol_deg | ...
                  theta_deg(:,joint) > theta_max + tol_deg;

        if any(outside)
            idx = find(outside, 1, 'first');

            if joint == 1
                joint_name = 'theta1';
            else
                joint_name = 'theta2';
            end

            if ~isempty(names) && numel(names) >= idx
                point_info = sprintf(' Punto: %s.', names{idx});
            else
                point_info = sprintf(' Indice: %d.', idx);
            end

            error(['Limite articular excedido en %s durante %s.%s ' ...
                   'Valor = %.3f deg. Limite permitido = [%.3f, %.3f] deg.'], ...
                   joint_name, context, point_info, ...
                   theta_deg(idx,joint), theta_min, theta_max);
        end
    end
end

function traj = planificarSegmentosSCARA(P, dwell, names, cfg)

    n_points = size(P,1);
    Qway = zeros(n_points,3);

    for i = 1:n_points
        Qway(i,:) = ikSCARA(P(i,:), cfg.robot, cfg.actuator);
    end

    validarLimitesArticulares(Qway, cfg.robot, names, 'puntos de paso');

    empty_segment = struct('duration_s', 0, 'q0', zeros(1,3), 'q1', zeros(1,3));
    segments = repmat(empty_segment, 0, 1);

    for i = 1:n_points-1
        q0 = Qway(i,:);
        q1 = Qway(i+1,:);
        tf = calcularDuracion(q0, q1, cfg);
        segments(end+1,1) = crearSegmento(tf, q0, q1);

        if dwell(i+1) > 0
            segments(end+1,1) = crearSegmento(dwell(i+1), q1, q1);
        end
    end

    [T, Q] = muestrearSegmentos(segments, cfg.num_muestras_csv);
    validarLimitesArticulares(Q, cfg.robot, [], 'trayectoria muestreada');

    traj.T = T;
    traj.Q = Q;
end

function segment = crearSegmento(duration_s, q0, q1)

    segment.duration_s = duration_s;
    segment.q0 = q0;
    segment.q1 = q1;
end

function tf = calcularDuracion(q0, q1, cfg)

    delta = abs(q1-q0);

    tf = max([
        cfg.tf_min, ...
        1.5*delta(1)/cfg.v_d_max, ...
        1.5*delta(2)/cfg.v_th_max, ...
        1.5*delta(3)/cfg.v_th_max
    ]);

    for iteration = 1:30
        t = linspace(0, tf, max(25, ceil(tf/cfg.dt_validacion)+1))';
        u = t/tf;
        s = 3.0*u.^2 - 2.0*u.^3;
        Q = q0 + s.*(q1-q0);
        X = fkSCARA(Q, cfg.robot, cfg.actuator);
        Xd = derivar(X, t);

        if max(vecnorm(Xd,2,2)) <= cfg.v_xyz_max*(1.0+1e-6)
            return;
        end

        tf = 1.15*tf;
    end

    error('No se pudo encontrar una duracion valida para un segmento.');
end

function [T, Q] = muestrearSegmentos(segments, num_muestras_total)

    n_segments = numel(segments);

    if num_muestras_total < n_segments + 1
        error('num_muestras_csv debe ser al menos %d para conservar los puntos de segmento.', n_segments + 1);
    end

    duraciones = [segments.duration_s]';
    total_s = sum(duraciones);

    % Cada segmento conserva al menos su punto inicial y final.
    % Las muestras extra se reparten proporcionalmente a la duracion.
    intervalos = ones(n_segments,1);
    extras = num_muestras_total - (n_segments + 1);

    if extras > 0
        pesos = duraciones / total_s;
        extra_real = extras * pesos;
        extra_base = floor(extra_real);
        intervalos = intervalos + extra_base;

        faltantes = extras - sum(extra_base);
        [~, orden] = sort(extra_real - extra_base, 'descend');
        intervalos(orden(1:faltantes)) = intervalos(orden(1:faltantes)) + 1;
    end

    T = [];
    Q = [];
    t_offset = 0.0;

    for i = 1:n_segments
        duration_s = segments(i).duration_s;
        n = intervalos(i);
        u = ((0:n)'/n);
        t_local = u * duration_s;

        if i > 1
            u = u(2:end);
            t_local = t_local(2:end);
        end

        s = 3.0*u.^2 - 2.0*u.^3;
        q_local = segments(i).q0 + s.*(segments(i).q1-segments(i).q0);

        T = [T; t_offset+t_local];
        Q = [Q; q_local];
        t_offset = t_offset + duration_s;
    end
end

function q = ikSCARA(P, robot, actuator)

    d = extensionFromTcpZ(P(3), actuator);
    x = P(1) - robot.base(1);
    y = P(2) - robot.base(2);
    D = (x^2+y^2-robot.L1^2-robot.L2^2)/(2.0*robot.L1*robot.L2);

    if abs(D) > 1.0+1e-9
        error('Punto fuera del espacio de trabajo planar. D = %.5f.', D);
    end

    D = min(1.0, max(-1.0, D));

    if robot.codo == "codo_arriba"
        s2 = sqrt(1.0-D^2);
    else
        s2 = -sqrt(1.0-D^2);
    end

    th2 = atan2(s2,D);
    th1 = atan2(y,x) - atan2(robot.L2*sin(th2), robot.L1+robot.L2*cos(th2));

    th1 = atan2(sin(th1), cos(th1));

    q = [d, th1, th2];
end

function X = fkSCARA(Q, robot, actuator)

    if isvector(Q)
        Q = reshape(Q,1,[]);
    end

    th1 = Q(:,2);
    th2 = Q(:,3);

    X = zeros(size(Q));
    X(:,1) = robot.base(1) + robot.L1*cos(th1) + robot.L2*cos(th1+th2);
    X(:,2) = robot.base(2) + robot.L1*sin(th1) + robot.L2*sin(th1+th2);
    X(:,3) = tcpZFromExtension(Q(:,1), actuator);
end

function d = extensionFromTcpZ(z_tcp, actuator)

    d = (z_tcp + actuator.tool_tip_offset_down_mm - actuator.z_axis_retracted_mm) ...
        /actuator.extension_sign;

    if any(d < actuator.min_mm-1e-6 | d > actuator.max_mm+1e-6)
        error('La altura TCP %.2f mm exige una extension fuera de [%.2f, %.2f] mm.', ...
            z_tcp, actuator.min_mm, actuator.max_mm);
    end
end

function z_tcp = tcpZFromExtension(d, actuator)

    z_tcp = actuator.z_axis_retracted_mm + actuator.extension_sign*d ...
        - actuator.tool_tip_offset_down_mm;
end

function dY = derivar(Y, t)

    dY = zeros(size(Y));

    if numel(t) < 2
        return;
    end

    for j = 1:size(Y,2)
        dY(:,j) = gradient(Y(:,j), t);
    end
end
