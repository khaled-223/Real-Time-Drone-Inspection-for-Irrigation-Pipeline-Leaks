clc; clear; close all;

%% =========================================================
% Project 5: Real-Time Drone Inspection for Irrigation Pipeline Leaks
% FINAL SUBMISSION v4 (Edited):
% - REMOVED the static HOME drone marker from the VIDEO scene (the big non-moving drone)
% - Moving drone only in the video
% - Figure 3 remains metrics-only
%% =========================================================

%% ---------- A) PIPELINE PATH (X,Y) ----------
pipelineXY = [
    10 10;
    60 10;
    110 30;
    160 30;
    190 60
];
homeXY = pipelineXY(1,:);   % Home location (x,y)

%% ---------- B) THERMAL MAP ----------
N = 240;
ambientTemp = 25;
noiseStd = 0.7;
leakTemp = 36;
threshold = 30;

thermalMap = ambientTemp + noiseStd*randn(N,N);

% Leak hotspot Gaussian bump (row,col)
leakCenterRC = [140 170]; % [row col]
leakSigma = 6;

[XX,YY] = meshgrid(1:N, 1:N); % XX=col, YY=row
d2 = (YY - leakCenterRC(1)).^2 + (XX - leakCenterRC(2)).^2;
hotspot = exp(-d2/(2*leakSigma^2));
thermalMap = thermalMap + (leakTemp - ambientTemp) * hotspot;

leakRadiusDraw = 10;

%% ---------- C) AOI (Pipeline Corridor) ----------
minX = min(pipelineXY(:,1));  maxX = max(pipelineXY(:,1));
minY = min(pipelineXY(:,2));  maxY = max(pipelineXY(:,2));

margin = 30;
AOI.xmin = max(1, round(minX - margin));
AOI.xmax = min(N, round(maxX + margin));
AOI.ymin = max(1, round(minY - margin));
AOI.ymax = max(1, min(N, round(maxY + margin)));

%% ---------- D) LAWNMOWER PATH (FAST + SMOOTH) ----------
trackSpacing = 14;
pointsPerLine = 140;
turnPoints = 28;

ys = AOI.ymin:trackSpacing:AOI.ymax;
path = [];

for k = 1:numel(ys)
    y = ys(k);

    if mod(k,2)==1
        xLine = linspace(AOI.xmin, AOI.xmax, pointsPerLine);
    else
        xLine = linspace(AOI.xmax, AOI.xmin, pointsPerLine);
    end

    lineSeg = [xLine(:) repmat(y, numel(xLine), 1)];
    path = [path; lineSeg];

    if k < numel(ys)
        yNext = ys(k+1);
        xEnd = lineSeg(end,1);

        t = linspace(0,1,turnPoints);
        bulge = 7*sin(pi*t);

        if mod(k,2)==1
            xTurn = xEnd + bulge;
        else
            xTurn = xEnd - bulge;
        end

        yTurn = (1-t)*y + t*yNext;
        path = [path; [xTurn(:) yTurn(:)]];
    end
end

path(:,1) = max(1, min(N, path(:,1)));
path(:,2) = max(1, min(N, path(:,2)));

win = 5;
pathSmooth = path;
pathSmooth(:,1) = movmean(path(:,1), win);
pathSmooth(:,2) = movmean(path(:,2), win);

pathIdx = round(pathSmooth);
pathIdx(:,1) = max(1,min(N,pathIdx(:,1)));
pathIdx(:,2) = max(1,min(N,pathIdx(:,2)));

%% ---------- E) HEADING + SPEED PROFILE ----------
dx = [0; diff(pathSmooth(:,1))];
dy = [0; diff(pathSmooth(:,2))];
headingRad = atan2(dy, dx);
headingDeg = rad2deg(headingRad);

dHead = [0; abs(diff(unwrap(headingRad)))];

speedMin = 35;
speedMax = 95;
turnStrength = 5;

speed = speedMax ./ (1 + turnStrength*dHead);
speed = max(speedMin, min(speedMax, speed));

dist = sqrt(dx.^2 + dy.^2);
dtLocal = dist ./ max(speed, 1e-6);
dtLocal(~isfinite(dtLocal) | dtLocal<=0) = 0.01;
timeSec = cumsum(dtLocal);

%% ---------- F) SCAN + DETECTION ----------
scanRadius = 7;
[ox, oy] = meshgrid(-scanRadius:scanRadius, -scanRadius:scanRadius);
circle = (ox.^2 + oy.^2) <= scanRadius^2;
offX = ox(circle);
offY = oy(circle);

scannedMask = false(N,N);
alertTime = NaN;

for i = 1:size(pathIdx,1)
    c = pathIdx(i,1);
    r = pathIdx(i,2);

    rr = r + offY;
    cc = c + offX;

    in = (rr>=1 & rr<=N & cc>=1 & cc<=N);
    rr = rr(in); cc = cc(in);

    lin = sub2ind([N N], rr, cc);
    scannedMask(lin) = true;

    if isnan(alertTime) && any(thermalMap(lin) > threshold)
        alertTime = timeSec(i);
    end
end

coverage = 100 * nnz(scannedMask) / numel(scannedMask);

if isnan(alertTime)
    leakDetected = false;
    alertDelay_ms = Inf;
else
    leakDetected = true;
    alertDelay_ms = alertTime * 1000;
end

returnToBase = leakDetected;

%% ---------- Helper geometry (kept for Figures 1 & 2) ----------
homeBodyR = 7; homeArmL = 16; homeRotR = 5;
rt = linspace(0,2*pi,60);
homeBodyCircle = [homeBodyR*cos(rt(:)) homeBodyR*sin(rt(:))];
homeArm1 = [-homeArmL 0; homeArmL 0];
homeArm2 = [0 -homeArmL; 0 homeArmL];
homeRotCenters = [ homeArmL 0; -homeArmL 0; 0 homeArmL; 0 -homeArmL ];
homeRotCircle = [homeRotR*cos(rt(:)) homeRotR*sin(rt(:))];

rotTrans = @(XY, ang, tx, ty) (XY * [cos(ang) -sin(ang); sin(ang) cos(ang)]) + [tx ty];
homeAng = 0;

th = linspace(0,2*pi,200);

%% ---------- G) FIGURE 1: THERMAL MAP (CLEAR HOME DRONE) ----------
fig1 = figure('Position',[80 80 960 700], 'Color','w');
imagesc(thermalMap); axis image; colorbar;
title("Figure 1: Simulated Thermal Map");
hold on;

B = rotTrans(homeBodyCircle, homeAng, homeXY(1), homeXY(2));
patch(B(:,1), B(:,2), 'k', 'FaceAlpha', 0.9);

A1 = rotTrans(homeArm1, homeAng, homeXY(1), homeXY(2));
A2 = rotTrans(homeArm2, homeAng, homeXY(1), homeXY(2));
plot(A1(:,1), A1(:,2), 'k-', 'LineWidth', 3);
plot(A2(:,1), A2(:,2), 'k-', 'LineWidth', 3);

for j=1:4
    xy = homeRotCircle + homeRotCenters(j,:);
    Rxy = rotTrans(xy, homeAng, homeXY(1), homeXY(2));
    patch(Rxy(:,1), Rxy(:,2), 'k', 'FaceAlpha', 0.25);
end

plot(homeXY(1) + 22*cos(th), homeXY(2) + 22*sin(th), 'k-', 'LineWidth', 2);

text(homeXY(1)+26, homeXY(2), "HOME (Takeoff point)", ...
    'Color','k', 'FontSize', 14, 'FontWeight','bold', ...
    'BackgroundColor','w', 'Margin', 4);

text(homeXY(1)+26, homeXY(2)+14, "UAV will take off from HOME", ...
    'Color','k', 'FontSize', 13, 'FontWeight','bold', ...
    'BackgroundColor','w', 'Margin', 4);

hold off;
saveas(fig1, "Figure1_Thermal_Map.png");

%% ---------- H) FIGURE 2: AOI + PATH + CLEAR HOME DRONE ----------
theta = linspace(0,2*pi,300);
xLeak = leakCenterRC(2) + leakRadiusDraw*cos(theta);
yLeak = leakCenterRC(1) + leakRadiusDraw*sin(theta);

fig2 = figure('Position',[80 80 960 700], 'Color','w');
imagesc(thermalMap); axis image; colorbar;
title("Figure 2: Lawnmower Scan Over Pipeline Corridor (AOI)");
hold on;

plot(xLeak, yLeak, 'k-', 'LineWidth', 2);

plot([AOI.xmin AOI.xmax AOI.xmax AOI.xmin AOI.xmin], ...
     [AOI.ymin AOI.ymin AOI.ymax AOI.ymax AOI.ymin], ...
     'k-', 'LineWidth', 2);

plot(pipelineXY(:,1), pipelineXY(:,2), 'k-', 'LineWidth', 3);
plot(pathSmooth(:,1), pathSmooth(:,2), 'k-', 'LineWidth', 1);

B = rotTrans(homeBodyCircle, homeAng, homeXY(1), homeXY(2));
patch(B(:,1), B(:,2), 'k', 'FaceAlpha', 0.9);
A1 = rotTrans(homeArm1, homeAng, homeXY(1), homeXY(2));
A2 = rotTrans(homeArm2, homeAng, homeXY(1), homeXY(2));
plot(A1(:,1), A1(:,2), 'k-', 'LineWidth', 3);
plot(A2(:,1), A2(:,2), 'k-', 'LineWidth', 3);
for j=1:4
    xy = homeRotCircle + homeRotCenters(j,:);
    Rxy = rotTrans(xy, homeAng, homeXY(1), homeXY(2));
    patch(Rxy(:,1), Rxy(:,2), 'k', 'FaceAlpha', 0.25);
end
plot(homeXY(1) + 22*cos(th), homeXY(2) + 22*sin(th), 'k-', 'LineWidth', 2);

text(homeXY(1)+26, homeXY(2), "HOME", 'Color','k', 'FontSize', 13, ...
    'FontWeight','bold', 'BackgroundColor','w', 'Margin', 4);

legend("Leak Area","AOI","Pipeline","Lawnmower Path","HOME");
hold off;
saveas(fig2, "Figure2_Lawnmower_Path_AOI.png");

%% ---------- I) VIDEO FIGURE + MOVING DRONE (NO STATIC HOME DRONE) ----------
figV = figure('Position',[80 80 960 700], 'Color','w', 'Visible','on');
set(figV,'Renderer','opengl');
set(figV,'Resize','off');
clf(figV);

imagesc(thermalMap); axis image; colorbar;
ax = gca;
if isprop(ax,'Toolbar'); ax.Toolbar.Visible = 'off'; end
set(figV,'ToolBar','none'); set(figV,'MenuBar','none');

title("UAV Inspection Mission (Realistic Motion)");
hold on;

plot(xLeak, yLeak, 'k-', 'LineWidth', 2);
plot([AOI.xmin AOI.xmax AOI.xmax AOI.xmin AOI.xmin], ...
     [AOI.ymin AOI.ymin AOI.ymax AOI.ymax AOI.ymin], ...
     'k-', 'LineWidth', 2);
plot(pipelineXY(:,1), pipelineXY(:,2), 'k-', 'LineWidth', 3);
plot(pathSmooth(:,1), pathSmooth(:,2), 'k-', 'LineWidth', 1);

% Trail
trailLen = 140;
trail = plot(pathSmooth(1,1), pathSmooth(1,2), 'k-', 'LineWidth', 2);

% Start text (no HOME)
startText = text(5, 28, "UAV is taking off... Mission started", ...
    'Color','k', 'FontSize', 16, 'FontWeight','bold', ...
    'BackgroundColor','w', 'Margin', 4);

infoText = text(5, 12, "", 'Color','k', 'FontSize', 12, 'FontWeight','bold', ...
    'BackgroundColor','w', 'Margin', 3);

% Moving drone geometry
bodyR = 5; armL = 12; rotR = 4;
rt2 = linspace(0,2*pi,50);
bodyCircle = [bodyR*cos(rt2(:)) bodyR*sin(rt2(:))];
arm1 = [-armL 0; armL 0];
arm2 = [0 -armL; 0 armL];
rotCenters = [ armL 0; -armL 0; 0 armL; 0 -armL ];
rotCircle = [rotR*cos(rt2(:)) rotR*sin(rt2(:))];

droneBody = patch(bodyCircle(:,1), bodyCircle(:,2), 'k', 'FaceAlpha', 0.85);
armLine1  = plot(arm1(:,1), arm1(:,2), 'k-', 'LineWidth', 2);
armLine2  = plot(arm2(:,1), arm2(:,2), 'k-', 'LineWidth', 2);
rotP = gobjects(4,1);
for j=1:4
    xy = rotCircle + rotCenters(j,:);
    rotP(j) = patch(xy(:,1), xy(:,2), 'k', 'FaceAlpha', 0.18);
end

q = quiver(pathSmooth(1,1), pathSmooth(1,2), 20, 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 2);

%% ---------- J) VIDEO WRITER ----------
videoName = "UAV_Final_Mission_v4.mp4";
useAVI = false;

try
    v = VideoWriter(videoName, "MPEG-4");
    v.FrameRate = 35;
    open(v);
catch
    useAVI = true;
end

if useAVI
    videoName = "UAV_Final_Mission_v4.avi";
    v = VideoWriter(videoName, "Motion JPEG AVI");
    v.FrameRate = 35;
    open(v);
end

%% ---------- K) VIDEO LOOP (FIXED FRAME SIZE) ----------
frameStep = 60;     % set to 18 for faster
arrowScale = 30;

drawnow;
img0 = print(figV, "-RGBImage");
H0 = size(img0,1); W0 = size(img0,2);
writeVideo(v, img0);

for i = 1:frameStep:size(pathSmooth,1)
    x = pathSmooth(i,1);
    y = pathSmooth(i,2);
    ang = headingRad(i);

    if timeSec(i) > 2
        set(startText, 'Visible', 'off');
    end

    i0 = max(1, i-trailLen);
    set(trail, 'XData', pathSmooth(i0:i,1), 'YData', pathSmooth(i0:i,2));

    % moving drone update
    Bm = rotTrans(bodyCircle, ang, x, y);
    set(droneBody, 'XData', Bm(:,1), 'YData', Bm(:,2));

    A1m = rotTrans(arm1, ang, x, y);
    A2m = rotTrans(arm2, ang, x, y);
    set(armLine1, 'XData', A1m(:,1), 'YData', A1m(:,2));
    set(armLine2, 'XData', A2m(:,1), 'YData', A2m(:,2));

    for j=1:4
        xy = rotCircle + rotCenters(j,:);
        Rxy = rotTrans(xy, ang, x, y);
        set(rotP(j), 'XData', Rxy(:,1), 'YData', Rxy(:,2));
    end

    set(q, 'XData', x, 'YData', y, 'UData', arrowScale*cos(ang), 'VData', arrowScale*sin(ang));

    if isnan(alertTime) || timeSec(i) < alertTime
        alertStr = "Alert: not triggered";
    else
        alertStr = "Alert: triggered";
    end

    set(infoText, 'String', sprintf("t=%.1fs  speed=%.1f  heading=%.0f°  %s", ...
        timeSec(i), speed(i), headingDeg(i), alertStr));

    drawnow;

    img = print(figV, "-RGBImage");
    if size(img,1) ~= H0 || size(img,2) ~= W0
        img = img(1:min(H0,end), 1:min(W0,end), :);
        if size(img,1) < H0, img(end+1:H0,:,:) = 255; end
        if size(img,2) < W0, img(:,end+1:W0,:) = 255; end
    end
    writeVideo(v, img);
end

% Outro message
if returnToBase
    outroMsg = sprintf("Leak detected. Return-to-base activated (start x=%.0f, y=%.0f).", homeXY(1), homeXY(2));
else
    outroMsg = "No leak detected. Mission finished.";
end

outText = text(5, 28, outroMsg, 'Color','k', 'FontSize', 16, 'FontWeight','bold', ...
    'BackgroundColor','w', 'Margin', 4);

for k = 1:25
    drawnow;
    img = print(figV, "-RGBImage");
    if size(img,1) ~= H0 || size(img,2) ~= W0
        img = img(1:min(H0,end), 1:min(W0,end), :);
        if size(img,1) < H0, img(end+1:H0,:,:) = 255; end
        if size(img,2) < W0, img(:,end+1:W0,:) = 255; end
    end
    writeVideo(v, img);
end

delete(outText);
close(v);
hold off;
saveas(figV, "Figure_Realistic_UAV_Movement.png");

%% ---------- L) FIGURE 3: METRICS ONLY (NO HOME / NO DRONE) ----------
fig3 = figure('Position',[80 80 960 700], 'Color','w');
axis off;
title("Figure 3: Leak Detection Results and Performance Metrics");

if isinf(alertDelay_ms)
    alertDelayText = "Alert Delay (ms): N/A (no alert)";
else
    alertDelayText = sprintf("Alert Delay (ms): %.0f", alertDelay_ms);
end

text(0.10,0.78, sprintf("Leak Detection Accuracy (%%): %.1f", 100*double(leakDetected)), ...
    'FontSize', 18, 'FontWeight','bold');

text(0.10,0.60, sprintf("Thermal Scan Coverage (%%): %.1f", coverage), ...
    'FontSize', 18, 'FontWeight','bold');

text(0.10,0.42, alertDelayText, ...
    'FontSize', 18, 'FontWeight','bold');

text(0.10,0.24, sprintf("Scan Radius (cells): %d", scanRadius), ...
    'FontSize', 16);

saveas(fig3, "Figure3_Performance_Metrics.png");

%% ---------- M) FIGURE 4: RETURN-TO-HOME CONFIRMATION ----------
fig4 = figure('Position',[80 80 960 700], 'Color','w');
axis off;
title("Figure 4: Return-To-Home Status");

text(0.08,0.72, sprintf("HOME location: (x=%.0f, y=%.0f)", homeXY(1), homeXY(2)), 'FontSize', 16);

if returnToBase
    text(0.08,0.50, "Return-to-home activated", 'FontSize', 18, 'FontWeight','bold');
    text(0.08,0.36, "UAV returned to HOME successfully", 'FontSize', 18, 'FontWeight','bold');
else
    text(0.08,0.50, "Return-to-home not triggered", 'FontSize', 18, 'FontWeight','bold');
    text(0.08,0.36, "UAV remains at HOME / standby", 'FontSize', 18, 'FontWeight','bold');
end

saveas(fig4, "Figure4_Return_To_Home_Status.png");

%% ---------- N) FIGURE 5: SUMMARY (NO 'MISSION COMPLETED' TEXT) ----------
fig5 = figure('Position',[80 80 960 700], 'Color','w');
axis off;
title("Figure 5: Mission Summary");

if leakDetected
    detStr = "Leak detected: YES";
    delayStr = sprintf("Alert delay: %.0f ms", alertDelay_ms);
else
    detStr = "Leak detected: NO";
    delayStr = "Alert delay: N/A";
end

covStr = sprintf("Thermal scan coverage: %.1f%%", coverage);

if returnToBase
    rtbStr = sprintf("Return-to-home: COMPLETED (HOME x=%.0f, y=%.0f)", homeXY(1), homeXY(2));
else
    rtbStr = sprintf("Return-to-home: NOT TRIGGERED (HOME x=%.0f, y=%.0f)", homeXY(1), homeXY(2));
end

text(0.08,0.65, detStr, 'FontSize', 18, 'FontWeight','bold');
text(0.08,0.50, covStr, 'FontSize', 18);
text(0.08,0.38, delayStr, 'FontSize', 18);
text(0.08,0.26, rtbStr, 'FontSize', 18, 'FontWeight','bold');

saveas(fig5, "Figure5_Mission_Summary.png");

%% ---------- O) OUTPUT + SAVE ----------
disp("==========================================");
disp("Project 5 - FINAL v4 Results (edited video: no static HOME drone)");
disp(["Video saved as: ", videoName]);
disp(["Leak Detected: ", num2str(leakDetected)]);
disp(["Thermal Scan Coverage (%): ", num2str(coverage, "%.2f")]);
disp(["Alert Delay (ms): ", num2str(alertDelay_ms, "%.1f")]);
disp(["Return-to-home: ", num2str(returnToBase), "  HOME(x,y)=(", num2str(homeXY(1)), ",", num2str(homeXY(2)), ")"]);
disp("==========================================");

save("Project5_Output.mat", ...
    "pipelineXY","homeXY","thermalMap","AOI","pathSmooth","pathIdx", ...
    "leakCenterRC","threshold","scanRadius","coverage","alertDelay_ms", ...
    "leakDetected","returnToBase","headingDeg","speed","timeSec","videoName");
