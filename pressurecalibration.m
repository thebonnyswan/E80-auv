file = 'pressurecalibration.csv';
data = csvread(file);

filetwo = 'Indexes.csv';
indexes = csvread(filetwo);

%startindex = indexes(:,3);

starttimes = [12 29.3 38.4 51.2 60.8 73.1 87.5 100.1 113.9 125 138 ...
    150.6 164.4 179 190.7 203.7 218.4 230.9 243.5 256.5 269.2 286.2 ...
    297.7 311.3 323.5 336.6 350.7];
startindex = starttimes.*10;
endindex = startindex+80;

pressuredata = data(2:end,2);
pressuredata = pressuredata';
timedata = data(2:end,1);
timedata = timedata';
figure(29)
plot(timedata,pressuredata)

len = length(startindex);
averagedata = zeros(len);
timedata = startindex./10;

for i=1:27 %last point was the 141cm, didn't matter
    start = startindex(i);
    endthing = endindex(i);
    avgvector = pressuredata(start:endthing-1);
    averagedata(i) = mean(avgvector);
    %figure(i)
    %plot(avgvector)
end

figure(28)
plot(timedata,averagedata,'*')
