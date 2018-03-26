% Levi Burner 2-10-18
% IARC Thrust Testing
% Graphs the current vs thrust for all files in the directory

clear
close all

files = dir('csv');
files = files(3:end);

data = {};
legends = {};
for i = 1:length(files)
    legends(end+1) = cellstr(files(i).name);
    fullname = ['csv/' files(i).name];
    data{end+1} = csvread(fullname);
    if contains(fullname, 'backward')
        data{end}(:, 1) = -data{end}(:, 1);
    end
end

% Plot current to thrust raw
subplot(2,2,1)
hold on
for i = 1:length(files)
   array = cell2mat(data(i));
   plot(array(:,2), array(:, 1));
end
xlabel('Current (A)')
ylabel('Thrust (kg)')
lgd = legend(legends);
lgd.Interpreter = 'none';
grid on
hold off

% Plot current to thrust fitted
subplot(2,2,2)
hold on
for i = 1:length(files)
   array = cell2mat(data(i));
   plot(array(:,2), smooth(array(:,2), array(:, 1), length(array(:, 1)), 'loess'));
end
xlabel('Current (A)')
ylabel('Thrust (kg)')
lgd = legend(legends);
lgd.Interpreter = 'none';
grid on
hold off

% Plot watts to thrust raw
subplot(2,2,3)
hold on
for i = 1:length(files)
   array = cell2mat(data(i));
   plot(array(:,4), array(:, 1));
end
xlabel('Watts (W)')
ylabel('Thrust (kg)')
lgd = legend(legends);
lgd.Interpreter = 'none';
grid on
hold off

% Plot watts to thrust fitted
subplot(2,2,4)
hold on
for i = 1:length(files)
   array = cell2mat(data(i));
   plot(array(:,4), smooth(array(:,4), array(:, 1), length(array(:, 1)), 'loess'));
end
xlabel('Watts (W)')
ylabel('Thrust (kg)')
lgd = legend(legends);
lgd.Interpreter = 'none';
grid on
hold off