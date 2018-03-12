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

% Plot Everything on one plot
figure
hold on
for i = 1:length(files)
   array = cell2mat(data(i));
   plot(array(:,4), smooth(array(:,4), array(:, 1), length(array(:, 1)), 'loess'));
end
xlabel('Power (W)')
ylabel('Thrust (kg)')
lgd = legend(legends);
lgd.Interpreter = 'none';
grid on
hold off