%% Graphical interface for signal visualization and interaction with ELEMYO sensors
% 2020-06-15 by ELEMYO (https://github.com/ELEMYO/Matlab_GUI)
% 
% Changelog:
%     2020-06-15 - initial release

%% Code is placed under t he MIT license
% Copyright (c) 2020 ELEMYO
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
% THE SOFTWARE.
% ===============================================

%% Delete old variables
clearvars;
delete(instrfindall);

%% Main graphics window
Graphics % File with visual data
Handles = guihandles(Fig); % Find graphics elements

%% Variables for communication
COM = 'COM6'; % COM port of Arduino
BaudRate = 115200; % Baudrate of communication

%% Variables
l = 2; % Index of the data point
Data = zeros(1, 2); % Array of data points
v = 0; % Value from COM port
N = 1;
SUMM=zeros(1);
TEMP3=1;

%% Start the serial communication
s = serial(COM,'BaudRate',BaudRate);
fopen(s);

%% Sample time
dt = 0;

%% Gain
gain = 1;

%% Initial state of filter
% Sample frequency
Sample_freq = 1000;
% Bandpass
W1_pass = 10;
W2_pass = 80;
order_pass = 4;
[b_pass,a_pass] = butter(order_pass, [W1_pass / (Sample_freq / 2), W2_pass / (Sample_freq / 2)], 'bandpass');
% Bandstop
W1_stop = 45;
W2_stop = 55;
order_stop = 4;
[b_stop,a_stop] = butter(order_stop, [W1_stop / (Sample_freq / 2), W2_stop / (Sample_freq / 2)], 'stop');

data_end = ' ';
v_end=0;
%% Main loop
while(ishandle(1))  % While exists figure
    if (s.BytesAvailable >= 12) % At least 3 points exist in the buffer
        %% Read data from serial buffer
        data = char(fread(s, s.BytesAvailable)); % Read raw data from serial buffer
        if (contains(data', 'T'))
            data=strrep(data', 'T', '-');
            v = sscanf(data,'%d'); % Convert raw data to array 
            
            if(length(v) > 3)
            for i=1:1:length(v) - 1
                if(v(i) < 0)
                    dt = - v(i) / 1000000;
                    v(i)=0;
                end
            end
            end
        else
            v = sscanf(data,'%d'); % Convert raw data to array
            if (length(v) > 3)
            if (~strcmp(data(1), newline) && ~strcmp(data(2), newline) && ~strcmp(data_end, newline))
                n_newline = strfind(data', newline);
                v(1) = str2double([int2str(v_end) data(1:n_newline)']);
            else
                 v = [v_end v'];
            end
            
            data_end = data(end);
            v_end = v(end);
            v(end) = [];
            end
            
        end
        
        if (length(v) < 3)% If some troubles
        	v = [0; 0; 0]; 
        end

        v=v/1024*5;
        
        %% Append new data
        t = zeros(length(v),2);
        t(:, 1) = v;
        t(1, 2) = Data(l-1,2) + dt;
        for i = 2:length(v)
            t(i, 2) = t(i-1, 2) + dt;
        end
        Data(l:l+length(v)-1,:) = t;
        
        %% Shift the boundaries of the graph
        T = Data(l+length(v)-1,2); % Curent time
        if Handles.full.Value == 1 % If "Full time" option
            set(Handles.main_axes, 'XLim', [0 T+0.1]);
        end
        if Handles.section.Value == 1 % If "10s section" option
            right = Handles.main_axes.XLim(2);
            if T > right
                set(Handles.main_axes, 'XLim', [right right+10]);
                set(Handles.main_axes, 'YLim', [Data_filtered(length(Data_filtered))-0.5, Data_filtered(length(Data_filtered))+0.5]);
                TEMP3 = l;
            end
        end
        
        %% Update gain
        old_gain = gain; % Remember old value of the gain
        gain = Handles.gainx1.Value * 1 + ...
            Handles.gainx2.Value * 2 + ...
            Handles.gainx4.Value * 4 + ...
            Handles.gainx5.Value * 5 + ...
            Handles.gainx8.Value * 8 + ...
            Handles.gainx10.Value * 10 + ...
            Handles.gainx16.Value * 16 + ...
            Handles.gainx32.Value * 32; % Read gain
        if (old_gain ~= gain) % If gain updated
            fwrite(s, gain); % Write gain to Arduino
        end
        
        %% Filtering signal
        Data_filtered(1:l,1) = Data(1:l,1);
        Sample_freq = 1 / dt;
        W1_pass = str2double( Handles.W1_pass.String );
        W2_pass = str2double( Handles.W2_pass.String );
      
        % Bandpass filter
        if (W1_pass > 0) && (W1_pass < W2_pass)
            if (Handles.Bandpass.Value == 1) && (W1_pass > 0)
                [b_pass,a_pass] = butter(order_pass, [W1_pass / (Sample_freq / 2), W2_pass / (Sample_freq / 2)], 'bandpass');
                Data_filtered(1:l,1) = filtfilt(b_pass, a_pass, Data_filtered(1:l,1));
            end
        end
        
        % Bandpass filter
        if (Handles.Bandstop50.Value == 1)
            [b_stop,a_stop] = butter(order_stop, [48 / (Sample_freq / 2), 52 / (Sample_freq / 2)], 'stop');
            Data_filtered(1:l,1) = filtfilt(b_stop, a_stop, Data_filtered(1:l,1));
            [b_stop,a_stop] = butter(order_stop, [98 / (Sample_freq / 2), 102 / (Sample_freq / 2)], 'stop');
            Data_filtered(1:l,1) = filtfilt(b_stop, a_stop, Data_filtered(1:l,1));
        end
        
        if (Handles.Bandstop60.Value == 1)
            [b_stop,a_stop] = butter(order_stop, [58 / (Sample_freq / 2), 62 / (Sample_freq / 2)], 'stop');
            Data_filtered(1:l,1) = filtfilt(b_stop, a_stop, Data_filtered(1:l,1));
            [b_stop,a_stop] = butter(order_stop, [118 / (Sample_freq / 2), 122 / (Sample_freq / 2)], 'stop');
            Data_filtered(1:l,1) = filtfilt(b_stop, a_stop, Data_filtered(1:l,1));
        end

        %% Update plot data
        
        % Update signal data
        set(SensorData, 'XData', Data(1:l,2), 'YData',  Data_filtered(1:l,1));
        if (l-TEMP3>400)
            set(Handles.main_axes, 'YLim', [min(Data_filtered(TEMP3:l-400,1))-0.1*abs(min(Data_filtered(TEMP3:l-400,1))), min([5 1.1*max(Data_filtered(TEMP3:l-400,1))])]);
        end
        
        % Update FFT data
        if (l>1500)
            L=1000;
            Y = fft(Data_filtered(l-L-500:l-500,1));
            
            P2 = abs(Y/(L));
            P1 = P2(1:L/2+1);
            P1(2:end-1) = 2*P1(2:end-1);

            f = 1/dt*(0:(L/2))/L;
            SUMM=(1-0.85)*P1 + 0.85*SUMM;
            set(SensorData2, 'XData', f, 'YData',  SUMM);
            set(Handles.main_axes2, 'YLim', [0, 1.1*abs(max(SUMM(2:length(SUMM),1)))]);    
        end       
     
        % Update spectrum data
        if ( (Data(l,2)- Data(N,2))>2 && dt>0.001 )
            L=l-N;
            N=l;
            mtlb= Data_filtered(l-L:1:l-500,1);
            [Z,Y] = cwt(mtlb,1/dt/1);
            X = Data(l-L:1:l-500,2);
            Z=abs(Z);
            Y=abs(Y);
            
            set(Handles.main_axes3, 'XLim', [X(1)  X(length(X))]);
            set(Handles.main_axes3, 'YLim', [min(Y)  120]);
            set(SensorData3 , 'XData', X, 'YData', Y, 'ZData', Z);
            shading flat;
        end
        
        pause(0.05);

        l = l + length(v); % Update data index     
    end
end

%% Stop the serial communication
fclose(s);
