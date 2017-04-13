% Real time data collection example
%
% This script is implemented as a function so that it can
%   include sub-functions
%
% This script can be modified to be used on any platform by changing the
% serialPort variable.
% Example:-
% On Linux:     serialPort = '/dev/ttyS0';
% On MacOS:     serialPort = '/dev/tty.KeySerial1';
% On Windows:   serialPort = 'COM1';
%
%To run: 
%plot_cams()
%To reset ports: (if MATLAB still thinks they're busy)
%delete(instrfindall)


function plot_cameras_original 

%Send over bluetooth or serial
serialPort = '/dev/cu.usbmodem1412';
serialObject = serial(serialPort);
%configure serial connection
serialObject.BaudRate = 9600; %(Default)
%serialObject.BaudRate = 115200;
%serialObject.FlowControl = 'software';

%Initiate serial connection
fopen(serialObject);

% This gets called on cleanup to ensure the stream gets closed
finishup = onCleanup(@() myCleanupFun(serialObject));

% Instantiate variables
count = 1;
trace = zeros(1, 128); %Stored Values for Raw Input
bintrace = zeros(1,128); %Stored Values for Edge Detection
smoothtrace  = zeros(1,128); %Stored Values for 5-Point Averager

while (1)
    % Check for data in the stream
    if serialObject.BytesAvailable
        val = fscanf(serialObject,'%i');
        %val
        if ((val == -1) | (val == -3)) % -1 and -3 are start keywords
            count = 1;
            val;
        elseif (val == -2) % End camera1 tx
            if (count == 128)
                plotdata(trace, 1);
            end %otherwise there was an error and don't plot
            count = 1;
            %plotdata(trace);
        elseif (val == -4) % End camera2 tx
            count = 1;
            plotdata(trace, 2);
        else
            trace(count) = val;
            count = count + 1;
        end % if 
    end %bytes available    
end % while(1)

% Clean up the serial object
fclose(serialObject);
delete(serialObject);
clear serialObject;

end %plot_cams

%*****************************************************************************************************************
%*****************************************************************************************************************

function plotdata(trace, cam)
threshold = 0;
drawnow;
subplot(4,2,cam);
%figure(figureHandle);
plot(trace);
%set(figureHandle,'Visible','on');

%SMOOTH AND PLOT
smoothtrace = trace;
for i = 3:126
    %5-point Averager
    %INSERT CODE
    smoothtrace(i-1) = (trace(i-2) + trace(i-1) + trace(i) + trace(i+1) + trace(i+2))/5;
end;
subplot(4,2,cam+2);
%figure(smoothhand);
plot(smoothtrace);

%THRESHOLD
%calculate 1's and 0's via thresholding
maxval = max(smoothtrace);
%
threshold = maxval/2;
for i = 1:128
    %Edge detection (binary 0 or 1)
    %INSERT CODE
    if(smoothtrace(i) < threshold)
        bintrace(i) = 0;
    else
        bintrace(i) = 1;
    end;
    
    
end
drawnow;
subplot(4,2,cam+4);
%figure(binfighand);
plot(bintrace);

end %function

function myCleanupFun(serialObject)
% Clean up the serial object
fclose(serialObject);
delete(serialObject);
clear serialObject;
delete(instrfindall);
end