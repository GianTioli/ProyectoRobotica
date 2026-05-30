%% Simulacion SCARA compacta con trayectoria cartesiana
clear; clc; close all;

%% Robot
L1 = 140;        % mm
L2 = 120;        % mm
base = [200 0];  % mm

db_min = 0;
db_max = 150;

%% Mesa y placa
mesa_L = 400;
mesa_H = 150;

placa_W = 85.4;
placa_H = 127.6;
placa_x0 = mesa_L - placa_W;
placa_y0 = 0;

%% Tubo
dist_base_tubo = 187.7;
dist_tubo_borde_placa = 250;

tubo_x = placa_x0 - dist_tubo_borde_placa;
dx_tubo = tubo_x - base(1);
tubo_y = sqrt(dist_base_tubo^2 - dx_tubo^2);
tubo = [tubo_x tubo_y];

%% Pozos
usar_placa_24 = false;

if usar_placa_24
    nCols = 4;
    nRows = 6;

    pitch_x = placa_W/nCols;
    pitch_y = placa_H/nRows;

    pozos_x = placa_x0 + pitch_x*((1:nCols) - 0.5);
    pozos_y = placa_y0 + pitch_y*((1:nRows) - 0.5);

    pozos_orden = generarOrdenPozos(pozos_x, pozos_y, true);
else
    diam_pozo = 30;
    paso_pozo = 39;
    margen_x = 23.2;
    margen_y = 24.8;

    pozos_x = placa_x0 + [margen_x, margen_x + paso_pozo];
    pozos_y = placa_y0 + [margen_y, margen_y + paso_pozo, margen_y + 2*paso_pozo];

    pozos_orden = [
        pozos_x(1), pozos_y(3)
        pozos_x(2), pozos_y(3)
        pozos_x(1), pozos_y(2)
        pozos_x(2), pozos_y(2)
        pozos_x(1), pozos_y(1)
        pozos_x(2), pozos_y(1)
    ];
end

%% Alturas
z_safe = 140;
z_asp  = 80;
z_disp = 20;

%% Configuracion de trayectoria
dt = 0.025;                 % s
tf_min = 0.6;               % s
v_xyz_max = 80;             % mm/s
v_db_max = 30;              % mm/s
v_th_max = deg2rad(100);    % rad/s

dwell_asp = 0.15;
dwell_disp = 0.15;

timeLaw = "lspb";           % "cubic" o "lspb"
tb_ratio = 0.25;            % solo para LSPB

%% Construccion de puntos de tarea
[P, names, dwell] = construirPuntos(tubo, pozos_orden, z_safe, z_asp, z_disp, dwell_asp, dwell_disp);

%% Planeacion cartesiana
cfg.L1 = L1;
cfg.L2 = L2;
cfg.base = base;
cfg.db_min = db_min;
cfg.db_max = db_max;
cfg.dt = dt;
cfg.tf_min = tf_min;
cfg.v_xyz_max = v_xyz_max;
cfg.v_db_max = v_db_max;
cfg.v_th_max = v_th_max;
cfg.timeLaw = timeLaw;
cfg.tb_ratio = tb_ratio;
cfg.config = "codo_abajo";

traj = planificarSCARA(P, dwell, cfg);

T = traj.T;
X = traj.X;
Xd = traj.Xd;
Q = traj.Q;
Qd = traj.Qd;
Qdd = traj.Qdd;
segID = traj.segID;

%% Graficas previas de planeacion
graficarPlaneacion(P, X, Xd, Q, Qd, Qdd, T, segID);

disp('Revise las trayectorias generadas.');
input('Presione Enter para animar el robot...');

%% Animacion simple
animarSCARA(Q, T, L1, L2, base, P, tubo, pozos_orden, placa_x0, placa_y0, placa_W, placa_H, mesa_L, mesa_H);

%% Exportacion opcional para ESP32
% tabla = [T, Q(:,1), Q(:,2), Q(:,3), Qd(:,1), Qd(:,2), Qd(:,3)];
% writematrix(tabla, 'trayectoria_scara.csv');

%% =========================================================
% FUNCIONES LOCALES
% =========================================================

function pozos_orden = generarOrdenPozos(pozos_x, pozos_y, serpentino)

    nCols = numel(pozos_x);
    nRows = numel(pozos_y);

    pozos_orden = [];

    for r = nRows:-1:1

        if serpentino && mod(nRows-r,2)==1
            cols = nCols:-1:1;
        else
            cols = 1:nCols;
        end

        for c = cols
            pozos_orden(end+1,:) = [pozos_x(c), pozos_y(r)];
        end
    end
end

function [P, names, dwell] = construirPuntos(tubo, pozos, z_safe, z_asp, z_disp, dwell_asp, dwell_disp)

    P = [];
    names = {};
    dwell = [];

    P(end+1,:) = [tubo z_safe];
    names{end+1} = 'Sobre tubo';
    dwell(end+1) = 0;

    P(end+1,:) = [tubo z_asp];
    names{end+1} = 'Aspiracion';
    dwell(end+1) = dwell_asp;

    P(end+1,:) = [tubo z_safe];
    names{end+1} = 'Salida tubo';
    dwell(end+1) = 0;

    for i = 1:size(pozos,1)

        P(end+1,:) = [pozos(i,:) z_safe];
        names{end+1} = sprintf('Sobre pozo %d', i);
        dwell(end+1) = 0;

        P(end+1,:) = [pozos(i,:) z_disp];
        names{end+1} = sprintf('Dispensacion pozo %d', i);
        dwell(end+1) = dwell_disp;

        P(end+1,:) = [pozos(i,:) z_safe];
        names{end+1} = sprintf('Salida pozo %d', i);
        dwell(end+1) = 0;
    end
end

function traj = planificarSCARA(P, dwell, cfg)

    X = [];
    Xd = [];
    Q = [];
    Qd = [];
    Qdd = [];
    T = [];
    segID = [];

    t_acum = 0;

    for i = 1:size(P,1)-1

        P0 = P(i,:);
        Pf = P(i+1,:);

        dist = norm(Pf-P0);
        tf = max([dist/cfg.v_xyz_max, abs(Pf(3)-P0(3))/cfg.v_db_max, cfg.tf_min]);

        valido = false;

        while ~valido

            [Xs, Xds, ts] = segmentoCartesiano(P0, Pf, tf, cfg.dt, cfg.timeLaw, cfg.tb_ratio);

            Qs = zeros(size(Xs));

            for k = 1:size(Xs,1)
                Qs(k,:) = ikSCARA(Xs(k,:), cfg);
            end

            Qds = derivar(Qs, ts);
            Qdds = derivar(Qds, ts);

            max_db = max(abs(Qds(:,1)));
            max_th = max(max(abs(Qds(:,2:3))));

            if max_db <= cfg.v_db_max && max_th <= cfg.v_th_max
                valido = true;
            else
                tf = tf*1.2;
            end
        end

        if i > 1
            Xs = Xs(2:end,:);
            Xds = Xds(2:end,:);
            Qs = Qs(2:end,:);
            Qds = Qds(2:end,:);
            Qdds = Qdds(2:end,:);
            ts = ts(2:end);
        end

        Tg = t_acum + ts;

        X = [X; Xs];
        Xd = [Xd; Xds];
        Q = [Q; Qs];
        Qd = [Qd; Qds];
        Qdd = [Qdd; Qdds];
        T = [T; Tg];
        segID = [segID; i*ones(numel(Tg),1)];

        t_acum = T(end);

        if dwell(i+1) > 0
            n = round(dwell(i+1)/cfg.dt);

            X = [X; repmat(X(end,:),n,1)];
            Xd = [Xd; zeros(n,3)];
            Q = [Q; repmat(Q(end,:),n,1)];
            Qd = [Qd; zeros(n,3)];
            Qdd = [Qdd; zeros(n,3)];
            T = [T; t_acum + (1:n)'*cfg.dt];
            segID = [segID; i*ones(n,1)];

            t_acum = T(end);
        end
    end

    traj.X = X;
    traj.Xd = Xd;
    traj.Q = Q;
    traj.Qd = Qd;
    traj.Qdd = Qdd;
    traj.T = T;
    traj.segID = segID;
end

function [X, Xd, t] = segmentoCartesiano(P0, Pf, tf, dt, timeLaw, tb_ratio)

    t = (0:dt:tf)';

    if t(end) < tf
        t = [t; tf];
    end

    tau = t/tf;

    switch timeLaw
        case "cubic"
            s = 3*tau.^2 - 2*tau.^3;
            sd = (6*tau - 6*tau.^2)/tf;

        case "lspb"
            tb = tb_ratio*tf;
            [s, sd] = lspbNormalizada(t, tf, tb);

        otherwise
            error('Ley temporal no reconocida.');
    end

    dP = Pf - P0;

    X = P0 + s.*dP;
    Xd = sd.*dP;
end

function [s, sd] = lspbNormalizada(t, tf, tb)

    V = 1/(tf - tb);
    a = V/tb;

    s = zeros(size(t));
    sd = zeros(size(t));

    for k = 1:numel(t)

        tk = t(k);

        if tk <= tb
            s(k) = 0.5*a*tk^2;
            sd(k) = a*tk;

        elseif tk <= tf - tb
            s(k) = V*(tk - tb/2);
            sd(k) = V;

        else
            tr = tf - tk;
            s(k) = 1 - 0.5*a*tr^2;
            sd(k) = a*tr;
        end
    end
end

function q = ikSCARA(P, cfg)

    x = P(1) - cfg.base(1);
    y = P(2) - cfg.base(2);
    z = P(3);

    if z < cfg.db_min || z > cfg.db_max
        error('d_b fuera de rango: %.2f mm', z);
    end

    D = (x^2 + y^2 - cfg.L1^2 - cfg.L2^2)/(2*cfg.L1*cfg.L2);

    if abs(D) > 1
        error('Punto fuera del espacio de trabajo. D = %.3f', D);
    end

    switch cfg.config
        case "codo_arriba"
            s2 = sqrt(1-D^2);
        case "codo_abajo"
            s2 = -sqrt(1-D^2);
        otherwise
            error('Configuracion no valida.');
    end

    th2 = atan2(s2,D);

    k1 = cfg.L1 + cfg.L2*cos(th2);
    k2 = cfg.L2*sin(th2);

    th1 = atan2(y,x) - atan2(k2,k1);

    q = [z th1 th2];
end

function dY = derivar(Y, t)

    dY = zeros(size(Y));

    for j = 1:size(Y,2)
        dY(:,j) = gradient(Y(:,j), t);
    end
end

function graficarPlaneacion(P, X, Xd, Q, Qd, Qdd, T, segID)

    figure('Color',[0.1 0.1 0.12]);

    subplot(2,2,1);
    plot3(X(:,1),X(:,2),X(:,3),'LineWidth',1.8); hold on;
    plot3(P(:,1),P(:,2),P(:,3),'o--','LineWidth',1.1);
    grid on; axis equal;
    xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]');
    title('Trayectoria cartesiana total');

    subplot(2,2,2);
    hold on;
    nSeg = max(segID);
    for i = 1:nSeg
        idx = segID == i;
        plot3(X(idx,1),X(idx,2),X(idx,3),'LineWidth',1.4);
    end
    grid on; axis equal;
    xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]');
    title('Trayectoria por segmentos');

    subplot(2,2,3);
    plot(T,X(:,1),T,X(:,2),T,X(:,3),'LineWidth',1.3);
    grid on;
    xlabel('Tiempo [s]');
    ylabel('Posicion [mm]');
    legend('x','y','z');
    title('Componentes cartesianas');

    subplot(2,2,4);
    plot(T,Xd(:,1),T,Xd(:,2),T,Xd(:,3),'LineWidth',1.3);
    grid on;
    xlabel('Tiempo [s]');
    ylabel('Velocidad [mm/s]');
    legend('xdot','ydot','zdot');
    title('Velocidades cartesianas');

    figure('Color',[0.1 0.1 0.12]);

    subplot(3,1,1);
    plot(T,Q(:,1),T,rad2deg(Q(:,2)),T,rad2deg(Q(:,3)),'LineWidth',1.3);
    grid on;
    ylabel('Posicion');
    legend('d_b [mm]','theta1 [deg]','theta2 [deg]');
    title('Variables articulares');

    subplot(3,1,2);
    plot(T,Qd(:,1),T,rad2deg(Qd(:,2)),T,rad2deg(Qd(:,3)),'LineWidth',1.3);
    grid on;
    ylabel('Velocidad');
    legend('dbdot [mm/s]','theta1dot [deg/s]','theta2dot [deg/s]');
    title('Velocidades articulares');

    subplot(3,1,3);
    plot(T,Qdd(:,1),T,rad2deg(Qdd(:,2)),T,rad2deg(Qdd(:,3)),'LineWidth',1.3);
    grid on;
    xlabel('Tiempo [s]');
    ylabel('Aceleracion');
    legend('dbddot [mm/s2]','theta1ddot [deg/s2]','theta2ddot [deg/s2]');
    title('Aceleraciones articulares');
end

function animarSCARA(Q, T, L1, L2, base, P, tubo, pozos, placa_x0, placa_y0, placa_W, placa_H, mesa_L, mesa_H)

    figure('Color',[0.1 0.1 0.12]);
    ax = axes;
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');

    xlim([-60 440]);
    ylim([-40 190]);
    zlim([0 170]);
    view(42,26);

    xlabel('x [mm]');
    ylabel('y [mm]');
    zlabel('z [mm]');
    title('Movimiento SCARA con trayectoria cartesiana');

    patch([0 mesa_L mesa_L 0],[0 0 mesa_H mesa_H],[0 0 0 0], ...
        [0.8 0.8 0.8],'FaceAlpha',0.08,'EdgeColor',[1 1 1]);

    rectangle('Position',[placa_x0 placa_y0 placa_W placa_H], ...
        'EdgeColor',[1 0.5 0.2],'LineWidth',1.2);

    plot3(P(:,1),P(:,2),P(:,3),'c--','LineWidth',1.2);
    plot3(tubo(1),tubo(2),115,'go','MarkerFaceColor','g');

    for i = 1:size(pozos,1)
        plot3(pozos(i,1),pozos(i,2),20,'o','Color',[1 0.6 0.3]);
    end

    h1 = plot3([0 0],[0 0],[0 0],'y-','LineWidth',5);
    h2 = plot3([0 0],[0 0],[0 0],'y-','LineWidth',5);
    h3 = plot3(0,0,0,'ro','MarkerFaceColor','r');
    trace = animatedline('Color','c','LineWidth',1.2);

    for k = 1:2:size(Q,1)

        q = Q(k,:);

        db = q(1);
        th1 = q(2);
        th2 = q(3);

        J1 = [base(1), base(2), db];
        J2 = [base(1)+L1*cos(th1), base(2)+L1*sin(th1), db];
        TCP = [J2(1)+L2*cos(th1+th2), J2(2)+L2*sin(th1+th2), db];

        set(h1,'XData',[J1(1) J2(1)],'YData',[J1(2) J2(2)],'ZData',[J1(3) J2(3)]);
        set(h2,'XData',[J2(1) TCP(1)],'YData',[J2(2) TCP(2)],'ZData',[J2(3) TCP(3)]);
        set(h3,'XData',TCP(1),'YData',TCP(2),'ZData',TCP(3));

        addpoints(trace,TCP(1),TCP(2),TCP(3));

        title(sprintf('t = %.2f s',T(k)));

        drawnow;
    end
end