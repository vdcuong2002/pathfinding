tic
clear all
close all

InputMap = zeros(10, 10);
InputMap(2:3, 2:4) = 1;
InputMap(4, 6:10) = 1;
start = [1, 1];
goal = [10, 10];

% InputMap = 1 - imread("MAP1.bmp");
% start = [1, 1];
% goal = [48, 98];

emptyMark = 1;
obstacleMark = 2;
goalMark = 3;
startMark = 4;
doneMark = 5;
loading = 6;
pathMark = 7;
EnvMark = 8;

row = size(InputMap, 1);
col = size(InputMap, 2);
mapArr = zeros(row, col);

mapArr(InputMap==0) = emptyMark;
mapArr(InputMap==1) = obstacleMark;
mapArr(start(1), start(2)) = startMark;
mapArr(goal(1), goal(2)) = goalMark;

positionArr = Inf(row, col, 2);
positionArr(start(1), start(2), :) = [start(1), start(2)];

parent = Inf(row, col, 2);
step = 0;
NeighborRow = start(1); NeighborCol = start(2);

figure('Position', [999,266,534,420])

gridOn = false; treeDraw = false;
distanceArr = 0;
DrawMap(mapArr, step, gridOn, treeDraw, parent);
NumberDirect = 8;

while true
    [RandomizedRow, RandomizedCol] = CreateRandomPoint(row, col);
    [NearestPointRow, NearestPointCol, distanceToNearest] = FindNearest(RandomizedRow, RandomizedCol, positionArr);
    NeighborRow = NearestPointRow + round((RandomizedRow - NearestPointRow)/distanceToNearest);
    NeighborCol = NearestPointCol + round((RandomizedCol - NearestPointCol)/distanceToNearest);

    if NeighborRow > row || NeighborRow < 1 || ...
            NeighborCol > col || NeighborCol < 1 || ...
            (NeighborRow == start(1) && NeighborCol == start(2))
        continue;
    elseif mapArr(NeighborRow, NeighborCol) == obstacleMark || ...
            mapArr(NeighborRow, NeighborCol) == doneMark
        continue;
    elseif (NeighborRow == goal(1) && NeighborCol == goal(2))
        parent(goal(1), goal(2), :) = [NearestPointRow, NearestPointCol];
        break;
    end
            
    parent(NeighborRow, NeighborCol, :) = [NearestPointRow, NearestPointCol];
    mapArr(NeighborRow, NeighborCol) = loading;

    step = step + 1;
    
%     if mod(step, 1) == 0
%         DrawMap(mapArr, step, gridOn, treeDraw, parent);
%         pause(0.001);
%     end

    mapArr(NeighborRow, NeighborCol) = doneMark;
    positionArr(NeighborRow, NeighborCol, :) = [NeighborRow, NeighborCol];
end

path = [goal(1), goal(2)];
while parent(path(1, 1), path(1, 2), 1) ~= start(1) || ...
        parent(path(1, 1), path(1, 2), 2) ~= start(2)
    path = [[parent(path(1, 1), path(1, 2), 1), parent(path(1, 1), path(1, 2), 2)]; ...
        path];
    mapArr(path(1, 1), path(1, 2)) = pathMark;
end

DrawMap(mapArr, step, gridOn, treeDraw, parent);

toc 

function [NearestPointRow, NearestPointCol, distanceToNearest] = FindNearest(CurrentRow, CurrentCol, positionArr)
DistanceToExistPoint = (positionArr(:, :, 1) - CurrentRow).^2 + (positionArr(:, :, 2) - CurrentCol).^2;
minimum = min(min(DistanceToExistPoint));
[NearestPointRow, NearestPointCol]=find(DistanceToExistPoint==minimum);
distanceToNearest = sqrt((CurrentRow - NearestPointRow)^2 + (CurrentCol - NearestPointCol)^2);
end

function [RandomizedRow, RandomizedCol] = CreateRandomPoint(row, col)
RandomizedRow = rand()*row;
RandomizedCol = rand()*col;
end

function DrawMap(mapArr, step, gridOn, treeDraw, parent)
cmap = [1 1 1; ...
    0 0 0; ...
    0 0 0.9; ...
    0 1 0; ...
    1 1 0.8; ...
    1 0.5 1;...
    1 0 0];
colormap(cmap);
row = size(mapArr, 1);
col = size(mapArr, 2);

image(1.5, 1.5, mapArr);
if treeDraw
    for i = 1:row
        for j = 1:col
            if parent(i, j, 1) ~= Inf
                line([j+0.5, parent(i, j, 2)+0.5], [i+0.5, parent(i, j, 1)+0.5], "LineStyle", '--', "Marker", "o", 'LineWidth', 1.5)
            end
        end
    end
end
legend("Tree branch")
PlotSetUp(gca, gridOn, col, row, step)
end
function PlotSetUp(gca, gridOn, col, row, step)
if gridOn
    grid on
    set(gca,'xtick', 0:1:col);
    set(gca,'ytick', 0:1:row);
    set(gca,'Yticklabel',[])
    set(gca,'Xticklabel',[])
else
    set(gca,'xtick', []);
    set(gca,'ytick', []);
end
axis equal
xlim([1, col+1])
ylim([1, row+1])
title(['Bước: ', num2str(step)]);
colorbar('Ticks',[1, 2, 3, 4, 5, 6, 7]+0.5, 'TickLabels',{'Chưa xem xét','Vật cản','Đích','Bắt đầu','Đã xem xét', 'Đang xem xét', 'Đường đi'})
end