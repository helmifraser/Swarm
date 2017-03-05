clc
close all

x_divisions = 5;

source_dir = pwd(); %uigetdir([]);
d = dir([source_dir '/*.csv']);
total = length(d);

filename = cell(1,total);
data = cell(1,total);
myPlot = figure('Position', [100, 100, 1350, 800]);
set (0, "defaultaxesfontsize", 14);
set (0, "defaulttextfontsize", 14);
annotation('textbox', [0.45 0.97 14 30], ...
    'String', 'Detected robots vs time', ...
    'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center')
    
av = 0;

for i = 1:total
  s1 = '[e';
  s2 = int2str(i);
  s3 = '] _data.csv';
  filename(1,i) = strcat(s1,s2,s3);
  data(1,i) = csvread(filename{1,i}, 2, 0);
  
  rows = ceil(total/(total/2));
  collumns = ceil(total/2);
  
  subplot(rows, collumns, i);
  plot(1:length(data{1,i}(:,3)), data{1,i}(:,3))
  
  if i == 1
    min_length = length(data{1,i}(:,3));
  elseif i > 1 & min_length > length(data{1,i}(:,3))
    min_length = length(data{1,i}(:,3));
  end
  
  axis([0 length(data{1,i}(:,3))]);
  set(gca,'XTick',0:ceil(length(data{1,i}(:,3))/x_divisions):length(data{1,i}(:,3)));
  robot_name = 'e-puck';
  plot_title = strcat(robot_name, s2);
  title(plot_title);
  xlabel('Simulation time steps (32ms)')
  ylabel('Detected robots')
  grid on
end

av = 0;

for i = 1:total
  av = av + data{1,i}(1:min_length,3);
end

average = round(av/total);

figure 
plot(1:min_length, average(:,1))
axis([0 min_length]);
set(gca,'XTick',0:ceil(min_length/x_divisions):min_length);
title('Average detected robots vs simulation time step');
xlabel('Simulation time steps (32ms)')
ylabel('Average detected robots')
grid on