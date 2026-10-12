% 2DX3 Final Project - Spatial Mapping System
% File: visualization.m
% Author: Shubh Patel
% Student Number: 400562382
% MacID: pates316
% Date: April 3, 2026
% Description: Receives scan data from TM4C1294NCPDT via
%              UART, converts polar to Cartesian coordinates,
%              and renders a real-time 3D point cloud in MATLAB.

% 2DX3 Spatial Scanner - MATLAB 3D Visualization
% Data format from MCU: scan_index,angle_deg,distance_mm
% PJ0 = start scan | PJ1 = undo last scan ring

% ---- CONFIG ----
PORT            = 'COM6';       % serial port the TM4C is connected to (check Device Manager)
BAUDRATE        = 115200;       % must match UART_Init baud rate in C code
DISPLACEMENT_MM = 100;          % 10 cm between each scan ring along x-axis (Deliverable 2 spec)

% ---- CLEANUP ----
% close any leftover serial ports from previous runs so we dont get port busy errors
oldPorts = serialportfind;
if ~isempty(oldPorts)
    delete(oldPorts);
end

% ---- OPEN SERIAL ----
s = serialport(PORT, BAUDRATE);     % open serial connection to TM4C
s.Timeout = 10;                     % wait up to 10 seconds for data before timing out
flush(s);                           % clear any garbage data sitting in the buffer
disp('Serial open on ' + string(PORT));
disp('Press PJ0 to start a scan...');
disp('Press PJ1 to undo the last scan ring.');

% ---- FIGURE ----
% set up the 3D plot window
fig = figure('Name', '2DX3 3D Scan');
ax  = axes('Parent', fig);
xlabel(ax, 'X - Along hallway (mm)');  % x axis = displacement between scans
ylabel(ax, 'Y - Left/Right (mm)');     % y axis = horizontal distance from sensor
zlabel(ax, 'Z - Up/Down (mm)');        % z axis = vertical distance from sensor
title(ax, 'Waiting for scan data...');
grid(ax, 'on');
view(ax, 45, 30);                       % initial 3D viewing angle
hold(ax, 'on');                         % keep all rings on same plot

colours = lines(20);        % 20 distinct colours, one per scan ring
scanPlaneCount = 0;         % counts how many complete rings have been plotted

% ---- BUFFERS ----
all_x = [];  all_y = [];  all_z = [];      % accumulated 3D points across all rings
cur_idx = []; cur_ang = []; cur_dst = [];  % buffer for current incoming scan ring
buf = '';                                   % raw serial character buffer for line parsing

% track how many points each ring added so we can undo exactly
pointsPerRing = [];

% track plot handles for each ring so we can delete the last one on RESET
ringHandles = [];

% ---- MAIN LOOP ----
% runs continuously while the figure window is open
while ishandle(fig)
    try
        nb = s.NumBytesAvailable;   % check how many bytes are waiting in serial buffer
        if nb == 0
            pause(0.02);            % nothing available, wait 20ms and check again
            drawnow;                % keep the plot window responsive
            continue;
        end

        % read all available bytes and append to character buffer
        raw = read(s, nb, 'uint8');
        buf = [buf, char(raw)];

        % process all complete lines in the buffer
        while true
            pos = strfind(buf, newline);    % look for newline character
            if isempty(pos), break; end     % no complete line yet, wait for more data

            % extract one complete line
            line = buf(1:pos(1)-1);
            buf  = buf(pos(1)+1:end);       % remove processed line from buffer
            line = strrep(line, char(13), '');  % strip carriage return \r
            line = strtrim(line);               % strip leading/trailing whitespace
            if isempty(line), continue; end     % skip blank lines

            disp("RX: " + line);    % print every received line to Command Window for debugging

            % ---- RESET: undo last ring ----
            % sent by MCU when PJ1 is pressed
            if strcmp(line, 'RESET')
                if scanPlaneCount > 0
                    % delete the last ring's line from the 3D plot
                    if ishandle(ringHandles(end))
                        delete(ringHandles(end));
                    end
                    ringHandles(end) = [];          % remove handle from tracking array

                    % remove the last ring's points from the accumulated 3D buffers
                    nRemove = pointsPerRing(end);   % how many points that ring had
                    pointsPerRing(end) = [];        % remove its count from tracker
                    all_x(end-nRemove+1:end) = [];  % remove last nRemove x points
                    all_y(end-nRemove+1:end) = [];  % remove last nRemove y points
                    all_z(end-nRemove+1:end) = [];  % remove last nRemove z points

                    scanPlaneCount = scanPlaneCount - 1;    % decrement ring counter

                    % update title to reflect new state
                    if scanPlaneCount == 0
                        title(ax, 'All rings removed - waiting for scan...');
                    else
                        title(ax, sprintf('%d scan plane(s)  |  %d points total', ...
                              scanPlaneCount, length(all_x)));
                    end

                    drawnow;
                    disp(sprintf('Last ring removed. %d ring(s) remaining.', scanPlaneCount));
                else
                    disp('Nothing to undo.');   % no rings to remove
                end

            % ---- DISCARD ----
            % sent by MCU when stop button pressed mid-scan
            % throw away all buffered data for this incomplete scan
            elseif strcmp(line, 'DISCARD')
                cur_idx=[]; cur_ang=[]; cur_dst=[];
                disp('Scan discarded.');

            % ---- END: plot the completed scan ring ----
            % sent by MCU after all 32 angle,distance pairs transmitted
            elseif strcmp(line, 'END')
                disp(sprintf('END received. Points in buffer: %d', length(cur_ang)));

                if ~isempty(cur_ang)
                    % sort points by angle so the ring draws correctly
                    [cur_ang, sortIdx] = sort(cur_ang);
                    cur_dst = cur_dst(sortIdx);
                    cur_idx = cur_idx(sortIdx);

                    % convert polar (angle, distance) to cartesian (y, z)
                    rad   = deg2rad(cur_ang);
                    x_pts = double(cur_idx) .* DISPLACEMENT_MM;    % x = scan index * 100mm
                    y_pts = double(cur_dst) .* sin(rad);            % y = horizontal
                    z_pts = -double(cur_dst) .* cos(rad);           % z = vertical (negated so up is up)

                    % close the ring by repeating the first point at the end
                    x_ring = [x_pts, x_pts(1)];
                    y_ring = [y_pts, y_pts(1)];
                    z_ring = [z_pts, z_pts(1)];

                    % add this ring's points to the master accumulated buffers
                    all_x = [all_x, x_pts];
                    all_y = [all_y, y_pts];
                    all_z = [all_z, z_pts];

                    % record how many points this ring contributed for undo tracking
                    pointsPerRing(end+1) = length(x_pts);

                    % pick a unique colour for this ring
                    scanPlaneCount = scanPlaneCount + 1;
                    c = colours(mod(scanPlaneCount-1, size(colours,1))+1, :);

                    % draw the ring as a 3D line with markers and save its handle
                    h = plot3(ax, x_ring, y_ring, z_ring, '-o', ...
                              'Color', c, ...
                              'MarkerFaceColor', c, ...
                              'MarkerSize', 5, ...
                              'LineWidth', 1.2);
                    ringHandles(end+1) = h;     % save handle so we can delete it on RESET

                    % update axis labels and title
                    xlabel(ax, 'X - Along hallway (mm)');
                    ylabel(ax, 'Y - Left/Right (mm)');
                    zlabel(ax, 'Z - Up/Down (mm)');
                    title(ax, sprintf('%d scan plane(s)  |  %d points total', ...
                          scanPlaneCount, length(all_x)));
                    grid(ax, 'on');
                    view(ax, 45, 30);   % maintain 3D viewing angle

                    % auto scale axes to fit all data with padding
                    xpad = max(DISPLACEMENT_MM, 50);
                    rmax = max(abs([all_y, all_z])) * 1.15 + 50;   % 15% padding on radius
                    if length(unique(all_x)) > 1
                        xlim(ax, [min(all_x)-xpad, max(all_x)+xpad]);
                    else
                        xlim(ax, [-xpad, xpad]);    % only one ring so far
                    end
                    ylim(ax, [-rmax, rmax]);
                    zlim(ax, [-rmax, rmax]);

                    drawnow;    % update the plot immediately
                    disp(sprintf('Ring %d plotted at X=%dmm', scanPlaneCount, x_pts(1)));
                end

                % clear current ring buffer ready for next scan
                cur_idx=[]; cur_ang=[]; cur_dst=[];

            % ---- DATA LINE: scan_index,angle,distance ----
            % regular measurement point sent by MCU during transmission
            else
                parts = strsplit(line, ',');    % split on comma
                if length(parts) == 3
                    i = str2double(parts{1});   % scan index (which ring)
                    a = str2double(parts{2});   % angle in degrees (0 to 348.75)
                    d = str2double(parts{3});   % distance in mm from ToF sensor
                    if ~isnan(i) && ~isnan(a) && ~isnan(d) && d > 0
                        % only store valid non-zero readings
                        cur_idx(end+1) = i;     % add to current ring index buffer
                        cur_ang(end+1) = a;     % add to current ring angle buffer
                        cur_dst(end+1) = d;     % add to current ring distance buffer
                    end
                end
            end

            drawnow;    % keep plot responsive after processing each line
        end

    catch e
        disp('Error: ' + string(e.message));
        break;  % exit loop on any error
    end
end

% ---- CLEANUP ----
clear s;        % close and release the serial port
disp('Done.');