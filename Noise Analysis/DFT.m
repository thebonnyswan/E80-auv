
clear;
clf;

% map from datatype to length in bytes
dataSizes.('float') = 4;
dataSizes.('ulong') = 4;
dataSizes.('int') = 4;
dataSizes.('int32') = 4;
dataSizes.('uint8') = 1;
dataSizes.('uint16') = 2;
dataSizes.('char') = 1;




infofile = 'Pool2.txt';
datafile = 'Pool2.bin';

%% read from info file to get log file structure
fileID = fopen(infofile);
items = textscan(fileID,'%s','Delimiter',',','EndOfLine','\r\n');
fclose(fileID);
[ncols,~] = size(items{1});
ncols = ncols/2;
varNames = items{1}(1:ncols)';
varTypes = items{1}(ncols+1:end)';
varLengths = zeros(size(varTypes));
for i = 1:numel(varTypes)
    varLengths(i) = dataSizes.(varTypes{i});
end


R = cell(1,numel(varNames));

%% read column-by-column from datafile
fid = fopen(datafile,'rb');
for i=1:numel(varTypes)
    %# seek to the first field of the first record
    fseek(fid, sum(varLengths(1:i-1)), 'bof');

    %# % read column with specified format, skipping required number of bytes
    R{i} = fread(fid, Inf, ['*' varTypes{i}], sum(varLengths)-varLengths(i));
    eval(strcat(varNames{i},'=','R{',num2str(i),'};'));
end
fclose(fid);

% Convert to voltages
A00 = double(A00).*3.3./1023;
A02 = double(A02).*3.3./1023;
A01 = double(A01).*3.3./1023; % Temperature

Fs = 10; % Sampling frequency
T = 1/Fs;
L = length(A02); % Data length
t = (0:L-1)*T; % Time vector
f = Fs*(0:(L/2))/L;
cutoff = (L/2-2750);
cutoff = 100.5;

subplot(2,1,1)
temp_fft = fft(A01);
plot(abs(temp_fft(1:(L/2+1))))


subplot(2,1,2)
press_fft = fft(A00);
plot(abs(press_fft(1:(L/2+1))))

new_temp_fft = temp_fft;
new_temp_fft(cutoff:end-cutoff+1) = temp_fft(cutoff:end-cutoff+1)*0.01;
new_press_fft = press_fft;
new_press_fft(cutoff:end-cutoff+1) = press_fft(cutoff:end-cutoff+1)*0.01;
new_temp = ifft(new_temp_fft);
new_press = ifft(new_press_fft);
close all
figure(1)
plot(t,A01)
hold on
plot(t,new_temp,'r')

figure(2)

plot(t,A00)
hold on
plot(t,new_press,'r')