%% VISUALIZE_ROBOT_3D  3D CAD-style animation of the two-wheeled balancing robot
% Run this AFTER two_wheeled_robot_simulation.m — it reuses the (t, x, K,
% x_ref, params) variables already sitting in your workspace. No
% Simulink/Simscape needed.
%
% v3 additions over v2:
%   - Camera angle lowered/adjusted so both wheels clearly read as
%     side-by-side under the chassis instead of diagonally offset
%   - Optional MP4 video export via VideoWriter (toggle SAVE_VIDEO below)

if ~exist('t', 'var') || ~exist('x', 'var') || ~exist('params', 'var') || ~exist('K', 'var') || ~exist('x_ref', 'var')
    error('Run two_wheeled_robot_simulation.m first — this script needs t, x, K, x_ref, params in the workspace.');
end

%% --- Video export settings ---
SAVE_VIDEO = true;              % set false to just watch it live, no file written
VIDEO_FILENAME = 'robot_balance_recovery.avi';
VIDEO_FPS = 30;
% NOTE: using 'Motion JPEG AVI' below, not 'MPEG-4'. MATLAB Online runs in
% a cloud sandbox without the OS-level codec 'MPEG-4' needs, which is
% exactly what throws "The specified profile is not valid." Motion JPEG
% AVI has no such dependency and works reliably in MATLAB Online. If you
% need an .mp4 for a website/resume link afterward, download the .avi and
% convert it locally (e.g. with ffmpeg: `ffmpeg -i in.avi out.mp4`) or with
% any free online converter — the video content itself is identical.

x_pos = x(:,1);
theta = x(:,3);

u_hist = zeros(length(t),1);
for k = 1:length(t)
    u_hist(k) = -K * (x(k,:)' - x_ref);
end

R  = params.R;      % wheel radius
L  = params.l;      % chassis length (axle to center of mass)
trackW = 0.25;       % distance between the two wheels
wheelW = 0.035;      % wheel thickness

%% Build reusable geometry templates (centered at local origin)

[cx, cz, cy] = cylinder(R, 28);
cy = (cy - 0.5) * wheelW;
wheel_verts = [cx(:), cy(:), cz(:)];
wheel_faces = reshape(1:numel(cx), size(cx));

[hcx, hcz, hcy] = cylinder(R*0.35, 16);
hcy = (hcy - 0.5) * (wheelW*1.2);
hub_verts = [hcx(:), hcy(:), hcz(:)];
hub_faces = reshape(1:numel(hcx), size(hcx));

w_base = 0.05; w_tip = 0.035;
box_verts = [
    -w_base/2 -w_base/2 0;  w_base/2 -w_base/2 0;
    -w_base/2  w_base/2 0;  w_base/2  w_base/2 0;
    -w_tip/2  -w_tip/2  L;  w_tip/2  -w_tip/2  L;
    -w_tip/2   w_tip/2  L;  w_tip/2   w_tip/2  L];
box_faces = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];

[sx, sy, sz] = sphere(20);
sph_r = 0.04;
sph_verts = sph_r*[sx(:), sy(:), sz(:)];
sph_faces = convhull(sph_verts(:,1), sph_verts(:,2), sph_verts(:,3));

%% Figure setup

fig = figure('Name', 'Two-Wheeled Robot — 3D Visualization', 'Color', [0.95 0.96 0.98]);
ax = axes('Parent', fig);
hold(ax, 'on'); axis(ax, 'equal'); axis(ax, 'off');
view(ax, [18 15]);              % lowered azimuth so wheels read as side-by-side
camproj(ax, 'perspective');
camva(ax, 7);
lighting(ax, 'gouraud');
camlight(ax, 'headlight');
light(ax, 'Position', [0 -1 1], 'Style', 'infinite', 'Color', [0.3 0.3 0.35]);
material(ax, 'metal');
set(ax, 'AmbientLightColor', [0.45 0.45 0.5]);

% --- Checkerboard floor ---
tileSz = 0.15;
xRange = [min(x_pos)-1, max(x_pos)+1];
nX = ceil(diff(xRange)/tileSz);
nY = ceil(0.6/tileSz);
for ix = 0:nX-1
    for iy = 0:nY-1
        x0 = xRange(1) + ix*tileSz;
        y0 = -0.3 + iy*tileSz;
        if mod(ix+iy, 2) == 0
            col = [0.82 0.84 0.88];
        else
            col = [0.65 0.68 0.74];
        end
        patch(ax, [x0 x0+tileSz x0+tileSz x0], [y0 y0 y0+tileSz y0+tileSz], [0 0 0 0], ...
            col, 'EdgeColor', 'none');
    end
end

% --- hgtransform groups ---
hL = hgtransform('Parent', ax);
hR = hgtransform('Parent', ax);
hC = hgtransform('Parent', ax);

wheelColor = [0.12 0.12 0.14];
hubColor   = [0.55 0.56 0.6];
chassisColor = [0.15 0.35 0.75];
massColor = [0.85 0.15 0.15];

patch('Parent', hL, 'Vertices', wheel_verts, 'Faces', wheel_faces, ...
    'FaceColor', wheelColor, 'EdgeColor', 'none', 'SpecularStrength', 0.6);
patch('Parent', hL, 'Vertices', hub_verts, 'Faces', hub_faces, ...
    'FaceColor', hubColor, 'EdgeColor', 'none', 'SpecularStrength', 0.8);

patch('Parent', hR, 'Vertices', wheel_verts, 'Faces', wheel_faces, ...
    'FaceColor', wheelColor, 'EdgeColor', 'none', 'SpecularStrength', 0.6);
patch('Parent', hR, 'Vertices', hub_verts, 'Faces', hub_faces, ...
    'FaceColor', hubColor, 'EdgeColor', 'none', 'SpecularStrength', 0.8);

patch('Parent', hC, 'Vertices', box_verts, 'Faces', box_faces, ...
    'FaceColor', chassisColor, 'EdgeColor', [0.05 0.1 0.3], 'LineWidth', 0.5, ...
    'SpecularStrength', 0.5);
patch('Parent', hC, 'Vertices', sph_verts + [0 0 L], 'Faces', sph_faces, ...
    'FaceColor', massColor, 'EdgeColor', 'none', 'SpecularStrength', 0.7);

trailLine = plot3(ax, NaN, NaN, NaN, '-', 'Color', [0.85 0.15 0.15 0.35], 'LineWidth', 1.5);
trailX = []; trailZ = [];

titleH = title(ax, '', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.15 0.15 0.2]);
hudH = annotation(fig, 'textbox', [0.02 0.85 0.4 0.1], 'EdgeColor', 'none', ...
    'FontSize', 10, 'FontName', 'Consolas', 'Color', [0.2 0.2 0.25]);

%% Video writer setup

if SAVE_VIDEO
    vw = VideoWriter(VIDEO_FILENAME, 'Motion JPEG AVI');
    vw.FrameRate = VIDEO_FPS;
    open(vw);
end

%% Animate

for i = 1:5:length(t)
    curr_x = x_pos(i);
    curr_theta = theta(i);
    curr_u = u_hist(i);

    phi = curr_x / R;
    TL = makehgtform('translate', [curr_x, +trackW/2, R]) * makehgtform('xrotate', phi);
    TR = makehgtform('translate', [curr_x, -trackW/2, R]) * makehgtform('xrotate', phi);
    set(hL, 'Matrix', TL);
    set(hR, 'Matrix', TR);

    TC = makehgtform('translate', [curr_x, 0, R]) * makehgtform('yrotate', -curr_theta);
    set(hC, 'Matrix', TC);

    tipX = curr_x + L*sin(curr_theta);
    tipZ = R + L*cos(curr_theta);
    trailX(end+1) = tipX; %#ok<SAGROW>
    trailZ(end+1) = tipZ; %#ok<SAGROW>
    set(trailLine, 'XData', trailX, 'YData', zeros(size(trailX)), 'ZData', trailZ);

    xlim(ax, [curr_x - 0.55, curr_x + 0.55]);
    zlim(ax, [-0.02, R + L + 0.12]);
    ylim(ax, [-0.3 0.3]);

    set(titleH, 'String', sprintf('t = %.2f s', t(i)));
    set(hudH, 'String', sprintf('pos:   %+.3f m\nangle: %+.2f deg\ntorque:%+.3f N·m', ...
        curr_x, rad2deg(curr_theta), curr_u));

    drawnow;

    if SAVE_VIDEO
        frame = getframe(fig);
        writeVideo(vw, frame);
    end
end

if SAVE_VIDEO
    close(vw);
    fprintf('Video saved to: %s\n', fullfile(pwd, VIDEO_FILENAME));
end

fprintf('Animation done.\n');
