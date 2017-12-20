clear;

% format, respective columns, 
% [clean_t',clean_temp',average_temp',clean_pressure']
load('1pmdata.mat')
load('145pmdata.mat')
load('210pmdata.mat')
load('245pmdata.mat')
load('1145amdata.mat')

clean_t1145 = data1145(:,1);
average_temp1145 = data1145(:,3);
clean_pressure1145 = data1145(:,4);
clean_t1 = data1(:,1);
average_temp1 = data1(:,3);
clean_pressure1 = data1(:,4);
clean_t145 = data145(:,1);
average_temp145 = data145(:,3);
clean_pressure145 = data145(:,4);
clean_t210 = data210(:,1);
average_temp210 = data210(:,3);
clean_pressure210 = data210(:,4);
clean_t245 = data245(:,1);
average_temp245 = data245(:,3);
clean_pressure245 = data245(:,4);

% figure(1)
% subplot(2,1,1)
% plot(clean_pressure1145,average_temp1145,'r')
% hold on
% plot(clean_pressure1,average_temp1,'y')
% plot(clean_pressure145,average_temp145,'g')
% plot(clean_pressure210,average_temp210,'c')
% plot(clean_pressure245,average_temp245,'b')
% title('alltemp')

% figure(2)
% subplot(5,1,1)
% plot(clean_t1145, clean_pressure1145, 'r')
% hold on
% plot(clean_t1145, average_temp1145, 'b')
% 
% subplot(5,1,2)
% plot(clean_t1, clean_pressure1, 'r')
% hold on
% plot(clean_t1, average_temp1, 'b')
% 
% subplot(5,1,3)
% plot(clean_t145, clean_pressure145, 'r')
% hold on
% plot(clean_t145, average_temp145, 'b')
% 
% subplot(5,1,4)
% plot(clean_t210, clean_pressure210, 'r')
% hold on
% plot(clean_t210, average_temp210, 'b')
% 
% subplot(5,1,5)
% plot(clean_t245, clean_pressure245, 'r')
% hold on
% plot(clean_t245, average_temp245, 'b')
% 
% figure(3)
% plot(clean_t1145, clean_pressure1145, 'r')
% hold on
% plot(clean_t1, clean_pressure1, 'y')
% plot(clean_t145, clean_pressure145, 'g')
% plot(clean_t210, clean_pressure210, 'c')
% plot(clean_t245, clean_pressure245, 'b')
% title('Measured Pressure over Time of Deployment')
% xlabel('Time (s)')
% ylabel('Pressure Voltage (V)')
% ylim = ([0 3.5]);

% first range- robot is in the water, but before motors move
% for all, largest possible is [76.7 116.3]
% start at 77 sec
% 11:45- 756
% 1- 369
% 145- 475
% 210- 446
% 245- 647

% end at 115.1 or closest
% 11:45- 1136 //380
% 1- 572 //203
% 145- 792 //317
% 210- 809 //363
% 245- 1011 //364


% second range- first depth
% largest possible is [160.3 201.6] 
% will exclude 11:45 (never dove then)

% 11:45- 1257 1457 //200
% 1- 651 734 //83
% 145- 935 1053 //118
% 210- 918 1038 //120
% 245- 1159 1296 //137

% third- second depth
% largest possible [225.2 289.3] 
% excludes 11:45 and 2:45 (Evan kicked it)

% 11:45- 1544 1706 //162
% 1- 783 893 //110
% 145- 1125 1302 //177
% 210- 1105 1296 //191
% 245- 1363 1553 //190


% fourth- third depth
% largest [316 382.5]

% 11:45- 1767 2006 //239
% 1- 950 1098 //148
% 145- 1377 1589 //212
% 210- 1377 1618 //241
% 245- 1622 1830 //208

tempstime1 = zeros(381,5);
tempstime1(:,1) = average_temp1145(756:1136)';
tempstime1(1:204,2) = average_temp1(369:572);
tempstime1(1:318,3) = average_temp145(475:792);
tempstime1(1:364,4) = average_temp210(446:809);
tempstime1(1:365,5) = average_temp245(647:1011);