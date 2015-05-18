% Serial Data Logger
% Yu Hin Hau
% 7/9/2013
% **CLOSE PLOT TO END SESSION
 
clear
clc
 
%User Defined Properties 
serialPort = 'COM3';            % define COM port #
plotTitle = 'LED SENSOR DISCHARGE';  % plot title
xLabel = 'Elapsed Time (s)';    % x-axis label
yLabel = 'Discharge TIme';                % y-axis label
plotGrid = 'on';                % 'off' to turn off grid
min = 0;                     % set y-min
max = 50000;                      % set y-max
scrollWidth = 20;               % display period in plot, plot entire data log if <= 0
delay = .001;                    % make sure sample faster than resolution
 
%Define Function Variables
time = 0;
Data = 0;
count = 0;
 
%Set up Plot
plotXGraph = plot(time, Data, '-',...
                'LineWidth',1.5,...
                'Color', 'b');
             
title(plotTitle,'FontSize',25);
xlabel(xLabel,'FontSize',15);
ylabel(yLabel,'FontSize',15);
axis([0 10 min max]);
grid(plotGrid);

 
%Open Serial COM Port
s = serial(serialPort);
s.BaudRate = 115200;
disp('Close Plot to End Session');
fopen(s);
s

tic
 
while ishandle(plotXGraph) %Loop when Plot is Active
     
    dat = fscanf(s, '%d'); % Read angle data from Serial as Signed Integer
%     velocityDat = fscanf(s, '%d'); % Read velocity data from Serial as a Signed Integer
    count = count + 1;    
    time(count) = toc;    %Extract Elapsed Time
  
    if(~isempty(dat)) %Make sure Data Type is Correct        
            Data(count) = dat;
        
    else
        Data(count) = 0; % If there is no data, there is no data :)
    end
    
    %Set Axis according to Scroll Width
    if(scrollWidth > 0)
    set(plotXGraph,'XData',time(time > time(count)-scrollWidth),'YData', Data(time > time(count)-scrollWidth));
    axis([time(count)-scrollWidth time(count) min max]);
    else
    set(plotXGraph,'XData',time,'YData',Data);
    axis([0 time(count) min max]);
    end

    %Allow MATLAB to Update Plot
    pause(delay);

end
 
%Close Serial COM Port and Delete useless Variables
fclose(s);
clear count dat delay max min plotGraph plotGrid plotTitle s ...
        scrollWidth serialPort xLabel yLabel;
 
 
disp('Session Terminated...');