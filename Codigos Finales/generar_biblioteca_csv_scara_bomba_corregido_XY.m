%% SCARA: generar biblioteca de CSV precomputados con columna de mL
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
robot.codo = "codo_arriba";

robot.home_theta_deg = [0, -10];

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
% Origen de mesa: esquina inferior izquierda/frontal.
% x: derecha, y: fondo, z: arriba.
mesa.length = 400.0;
mesa.width = 150.0;

% Placa de pie en la esquina inferior derecha.
% La placa ocupa x = 314.6 a 400.0 mm, y = 0.0 a 127.6 mm.
placa.width = 85.4;      % dimension en x
placa.height = 127.6;    % dimension en y
placa.x0 = mesa.length - placa.width;
placa.y0 = 0.0;
placa.hole_diameter = 30.0;
placa.hole_pitch = 39.0;
placa.hole_rim_z = 70.0;

% Punto extremo del plato usado para ubicar el tubo.
p_extremo_plato = [placa.x0, placa.y0 + placa.height];  % [314.6, 127.6]

% Tubo a 250 mm del punto extremo del plato.
tubo.center = [p_extremo_plato(1) - 250.0, p_extremo_plato(2)];  % [64.6, 127.6]
tubo.diameter = 30.0;
tubo.rim_z = 115.0;
tubo.insert_depth = 35.0;

% Alturas TCP.
task.plate_tcp_z = placa.hole_rim_z;
task.safe_tcp_z = task.plate_tcp_z + 115.0;    % altura de viaje: 95 mm sobre la placa
task.tube_tcp_z = task.plate_tcp_z + 47.5;    % altura de succion: 47.5 mm sobre la placa
task.dwell_asp_s = 0.15;
task.dwell_disp_s = 0.15;
task.ml_succion = 6.0;        % siempre se aspiran 6 mL en el tubo
task.ml_disp_por_pozo = 1.0;   % se dispensa 1 mL por hoyo seleccionado

%% Planeacion
cfg.dt_validacion = 0.025;      % Solo se usa internamente para validar velocidades.
cfg.num_muestras_csv = 400;    % Filas totales aproximadas/exactas del CSV.
cfg.tf_min = 0.60;
cfg.v_xyz_max = 80.0;
cfg.v_d_max = 1000.0/actuator.ms_per_mm;
cfg.v_th_max = deg2rad(100.0);
cfg.robot = robot;
cfg.actuator = actuator;

%% Pozos fijos de la placa [x, y] mm
% Coordenadas XY manuales respecto a la mesa.
% Para ajustar un punto, cambie solo su par [x, y].
pozo_1_xy = [337.8,  24.8];   % 1 inferior izquierda
pozo_2_xy = [376.8,  24.8];   % 2 inferior derecha
pozo_3_xy = [337.8,  63.8];   % 3 medio izquierda
pozo_4_xy = [376.8,  63.8];   % 4 medio derecha
pozo_5_xy = [337.8, 102.8];   % 5 superior izquierda
pozo_6_xy = [376.8, 102.8];   % 6 superior derecha

pozos = [
    pozo_1_xy;
    pozo_2_xy;
    pozo_3_xy;
    pozo_4_xy;
    pozo_5_xy;
    pozo_6_xy
];

% Biblioteca de trayectorias precomputadas.
% Cada pozo individual parte de Home, aspira, dispensa y vuelve a Home.
orden_seguro = [5 3 1 2 4 6];
CSV_DIR = 'trayectorias_precalculadas';
MUESTRAS_TODOS = 400;
MUESTRAS_POZO = 160;

if exist(CSV_DIR, 'dir')
    delete(fullfile(CSV_DIR, '*.csv'));
else
    mkdir(CSV_DIR);
end

generarCSV('todos.csv', orden_seguro, MUESTRAS_TODOS, pozos, robot, actuator, tubo, task, cfg, CSV_DIR);

for pozo_id = 1:6
    generarCSV(sprintf('pozo_%d.csv', pozo_id), pozo_id, MUESTRAS_POZO, pozos, robot, actuator, tubo, task, cfg, CSV_DIR);
end

for n_hoyos = 2:5
    combinaciones = nchoosek(1:6, n_hoyos);

    for k = 1:size(combinaciones,1)
        seleccion = orden_seguro(ismember(orden_seguro, combinaciones(k,:)));
        muestras = min(MUESTRAS_TODOS, MUESTRAS_POZO*numel(seleccion));
        generarCSV(nombreGrupo(seleccion), seleccion, muestras, pozos, robot, actuator, tubo, task, cfg, CSV_DIR);
    end
end

fprintf('\nBiblioteca generada en: %s\n', CSV_DIR);
fprintf('Archivos: todos.csv, pozo_1.csv, ..., pozo_6.csv y grupos de 2 a 5 hoyos.\n');

%% =========================================================
% FUNCIONES LOCALES
% =========================================================


function nombre = nombreGrupo(orden)

    texto = sprintf('%d_', orden);
    nombre = ['grupo_', texto(1:end-1), '.csv'];
end

function generarCSV(nombre_archivo, orden, muestras, pozos, robot, actuator, tubo, task, cfg, csv_dir)

    cfg.num_muestras_csv = muestras;
    pozos_orden = pozos(orden,:);

    % La succion inicial depende de la cantidad de pozos seleccionados
    % Ejemplo: 1 pozo -> 1 mL, 3 pozos -> 3 mL, 6 pozos -> 6 mL
    task.ml_succion = numel(orden) * task.ml_disp_por_pozo;

    [P, names, dwell, ml_way] = construirPuntos(robot, actuator, tubo, pozos_orden, task, orden);

    traj = planificarSegmentosSCARA(P, dwell, names, cfg);
    Q = traj.Q;
    ml_csv = mapearML(Q, P, ml_way, robot, actuator);

    idx_ml = find(ml_csv ~= 0);
    
    if ~isempty(idx_ml)
        ml_csv(idx_ml) = abs(ml_csv(idx_ml));
        ml_csv(idx_ml(1)) = -abs(ml_csv(idx_ml(1)));
    end

   % CSV: theta1_deg, theta2_deg, desplazamiento_actuador_mm, mL_bomba
    tabla_csv = [rad2deg(Q(:,2)), rad2deg(Q(:,3)), Q(:,1), ml_csv];

    % Offset de Home: todos los CSV inician en [0, 0, 0].
    % No se modifica la trayectoria calculada; solo se exporta relativa al primer punto.
    offset_home = tabla_csv(1, 1:3);
    tabla_csv(:, 1:3) = tabla_csv(:, 1:3) - offset_home;
    tabla_csv(1, 1:3) = [0, 0, 0];

    csv_path = fullfile(csv_dir, nombre_archivo);
    writematrix(tabla_csv, csv_path);

    fprintf('CSV generado: %-18s | hoyos: %-15s | filas: %d | aspira: %.1f mL | dispensa: %.1f mL\n', ...
        nombre_archivo, mat2str(orden), size(tabla_csv,1), task.ml_succion, numel(orden)*task.ml_disp_por_pozo);
end


function [P, names, dwell, ml_way] = construirPuntos(robot, actuator, tubo, pozos, task, pozo_ids)

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

    z_movimiento_pozos = task.plate_tcp_z + 10.0;

    for i = 1:size(pozos,1)
    
        if i == 1
            % Primer pozo:
            % Desde la salida del tubo se mueve primero XY a 115 mm,
            % sin mover el actuador lineal.
            z_sobre_pozo = task.safe_tcp_z;
        else
            % Siguientes pozos:
            % Se mueve XY entre pozos manteniendo solo 20 mm de altura.
            z_sobre_pozo = z_movimiento_pozos;
        end
    
        [P, names, dwell] = agregarPunto(P, names, dwell, ...
            [pozos(i,:), z_sobre_pozo], sprintf('Sobre pozo %d', pozo_ids(i)), 0);
    
        [P, names, dwell] = agregarPunto(P, names, dwell, ...
            [pozos(i,:), task.plate_tcp_z], sprintf('Dispensacion pozo %d', pozo_ids(i)), task.dwell_disp_s);
    
        [P, names, dwell] = agregarPunto(P, names, dwell, ...
            [pozos(i,:), z_movimiento_pozos], sprintf('Salida pozo %d', pozo_ids(i)), 0);
    
    end

    home_return = home_low;
    home_return(3) = task.plate_tcp_z + 10.0;
    
    [P, names, dwell] = agregarPunto(P, names, dwell, home_return, 'Retorno home seguro', 0);
    [P, names, dwell] = agregarPunto(P, names, dwell, home_low, 'Home final retraido', 0);

    ml_way = zeros(size(dwell));
    ml_way(contains(names, 'Aspiracion')) = task.ml_succion;
    ml_way(contains(names, 'Dispensacion')) = task.ml_disp_por_pozo;
end

function [P, names, dwell] = agregarPunto(P, names, dwell, point, name, dwell_s)

    P(end+1,:) = point;
    names{end+1} = name;
    dwell(end+1) = dwell_s;
end

function ml_csv = mapearML(Q, P, ml_way, robot, actuator)

    ml_csv = zeros(size(Q,1),1);
    idx_inicio = 1;

    for i = 1:numel(ml_way)
        if ml_way(i) == 0
            continue;
        end

        q_objetivo = ikSCARA(P(i,:), robot, actuator);
        dist = vecnorm(Q(idx_inicio:end,:) - q_objetivo, 2, 2);
        [~, idx_rel] = min(dist);
        idx = idx_inicio + idx_rel - 1;

        ml_csv(idx) = ml_way(i);
        idx_inicio = min(idx + 1, size(Q,1));
    end
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