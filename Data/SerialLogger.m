% Serial Data Logger
% Yu Hin Hau
% 7/9/2013
% **CLOSE PLOT TO END SESSION
 
clear
clc
 
%User Defined Properties 
serialPort = 'COM3';            % define COM port #
plotTempTitle = 'L3DG20 Real-Time Temperature';  % plot title
xTempLabel = 'Elapsed Time (s)';    % x-axis label
yTempLabel = 'Temperature [C]';                % y-axis label
plotVelocityTitle = 'L3DG20 Real-Time Velocities';  % plot title
xVelocityLabel = 'Elapsed Time (s)';    % x-axis label
yVelocityLabel = 'Velocity [deg/s]';                % y-axis label
plotAngleTitle = 'L3DG20 Real-Time Angles';  % plot title
xAngleLabel = 'Elapsed Time (s)';    % x-axis label
yAngleLabel = 'Angle [deg]';                % y-axis label
plotGrid = 'on';                % 'off' to turn off grid
minAngle = -360;                     % set y-min
maxAngle = 360;                      % set y-max
minVelocity = -250;                     % set y-min
maxVelocity = 250;                      % set y-max
minTemp = 20;                     % set y-min
maxTemp = 60;                      % set y-max
scrollWidth = 20;               % display period in plot, plot entire data log if <= 0
delay = .001;                    % make sure sample faster than resolution
 
%Define Function Variables
time = 0;
angleXData = 0;
angleYData = 0;
angleZData = 0;
velocityXData = 0;
velocityYData = 0;
velocityZData = 0;
tempData = 0;
% velocityData = 0;
count = 0;
 
%Set up Plot
ax(1) = subplot(2,2,[1,3]); % This is for angle
hold on;
plotXGraph = plot(ax(1), time, angleXData, '-',...
                'LineWidth',1.5,...
                'Color', 'b');
            
plotYGraph = plot(ax(1), time, angleYData, '-',...
                'LineWidth',1.5,...
                'Color', 'g');
            
plotZGraph = plot(ax(1), time, angleZData, '-',...
                'LineWidth',1.5,...
                'Color', 'r');
            
hold off;
             
title(plotAngleTitle,'FontSize',25);
xlabel(xAngleLabel,'FontSize',15);
ylabel(yAngleLabel,'FontSize',15);
axis([0 10 minAngle maxAngle]);
grid(plotGrid);

ax(2) = subplot(2,2,2); % This is for temperature
plotTempGraph = plot(ax(2), time, tempData, '--',...
                'LineWidth',1.5,...
                'Color', 'k');
            
title(plotTempTitle,'FontSize',25);
xlabel(xTempLabel,'FontSize',15);
ylabel(yTempLabel,'FontSize',15);
axis([0 10 minTemp maxTemp]);
grid(plotGrid);

ax(3) = subplot(2,2,4); % This is for velocity
hold on;
plotXGraphVel = plot(ax(3), time, velocityXData, '-',...
                'LineWidth',1.5,...
                'Color', 'b');
            
plotYGraphVel = plot(ax(3), time, velocityYData, '-',...
                'LineWidth',1.5,...
                'Color', 'g');
            
plotZGraphVel = plot(ax(3), time, velocityZData, '-',...
                'LineWidth',1.5,...
                'Color', 'r');
            
hold off;

title(plotVelocityTitle,'FontSize',25);
xlabel(xVelocityLabel,'FontSize',15);
ylabel(yVelocityLabel,'FontSize',15);
axis([0 10 minVelocity maxVelocity]);
grid(plotGrid);
 
%Open Serial COM Port
s = serial(serialPort);
s.BaudRate = 115200;
disp('Close Plot to End Session');
fopen(s);
s

tic
 
while ishandle(plotXGraph) %Loop when Plot is Active
     
    dat = fscanf(s); % Read angle data from Serial as Signed Integer
%     velocityDat = fscanf(s, '%d'); % Read velocity data from Serial as a Signed Integer
    count = count + 1;    
    time(count) = toc;    %Extract Elapsed Time
  
    if(~isempty(dat)) %Make sure Data Type is Correct        
        C = textscan(dat, '%d%d%d%d%d%d%d', 'delimiter', ';');
            [angleXData(count),...
            angleYData(count),...
            angleZData(count),...
            velocityXData(count),...
            velocityYData(count),...
            velocityZData(count),...
            tempData(count)] = deal(C{:});
        
        tempData(count) = tempData(count)/1000;
    else
        angleXData(count) = 0; % If there is no data, there is no data :)
        angleYData(count) = 0; % If there is no data, there is no data :)
        angleZData(count) = 0; % If there is no data, there is no data :)
        velocityXData(count) = 0; % If there is no data, there is no data :)
        velocityYData(count) = 0; % If there is no data, there is no data :)
        velocityZData(count) = 0; % If there is no data, there is no data :)
        tempData(count) = 0; % If there is no data, there is no data :)
    end
    
    %Set Axis according to Scroll Width
    if(scrollWidth > 0)
    set(plotXGraph,'XData',time(time > time(count)-scrollWidth),'YData', angleXData(time > time(count)-scrollWidth));
    set(plotYGraph,'XData',time(time > time(count)-scrollWidth),'YData', angleYData(time > time(count)-scrollWidth));
    set(plotZGraph,'XData',time(time > time(count)-scrollWidth),'YData', angleZData(time > time(count)-scrollWidth));                                                         
    set(plotXGraphVel,'XData',time(time > time(count)-scrollWidth),'YData', velocityXData(time > time(count)-scrollWidth));
    set(plotYGraphVel,'XData',time(time > time(count)-scrollWidth),'YData', velocityYData(time > time(count)-scrollWidth));
    set(plotZGraphVel,'XData',time(time > time(count)-scrollWidth),'YData', velocityZData(time > time(count)-scrollWidth));                                                         
    set(plotTempGraph,'XData',time(time > time(count)-scrollWidth),'YData', tempData(time > time(count)-scrollWidth));                                                         
    subplot(ax(1));
    axis([time(count)-scrollWidth time(count) minAngle maxAngle]);
    subplot(ax(2));
    axis([time(count)-scrollWidth time(count) minTemp maxTemp]);
    subplot(ax(3));
    axis([time(count)-scrollWidth time(count) minVelocity maxVelocity]);
    else
    set(plotXGraph,'XData',time,'YData',angleXData);
    set(plotYGraph,'XData',time,'YData',angleYData);
    set(plotZGraph,'XData',time,'YData',angleZData);
    set(plotXGraphVel,'XData',time,'YData',angleXData);
    set(plotYGraphVel,'XData',time,'YData',angleYData);
    set(plotZGraphVel,'XData',time,'YData',angleZData);
    set(plotTempGraph,'XData',time,'YData',tempData);
    subplot(ax(1));
    axis([0 time(count) minAngle maxAngle]);
    subplot(ax(2));
    axis([0 time(count) minTemp maxTemp]);
    subplot(ax(3));
    axis([0 time(count) minVelocity maxVelocity]);
    end

    %Allow MATLAB to Update Plot
    pause(delay);

end
 
%Close Serial COM Port and Delete useless Variables
fclose(s);
clear count dat delay max min plotGraph plotGrid plotTitle s ...
        scrollWidth serialPort xLabel yLabel;
 
 
disp('Session Terminated...');