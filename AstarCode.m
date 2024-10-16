clear all
close all

% InputMap = zeros(10, 10);
% InputMap(2:3, 2:4) = 1;
% InputMap(4, 6:10) = 1;
% start = [1, 1];
% goal = [10, 10];
% 
InputMap = 1 - imread("MAP1.bmp");
start = [1, 1];
goal = [48, 98];

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

CostValue = Inf(row, col);
CostValue(start(1), start(2)) = 0;
CostValueArr = CostValue;
CostValueArr(start(1), start(2)) = 0;
distanceArr = CostValue;
distanceArr(start(1), start(2)) = 0;

cost = ones(row, col);
parent = zeros(row, col, 2);

step = 0;
CurrentRow = start(1); CurrentCol = start(2);

figure('Position', [999,266,534,420])

gridOn = false; NodeInfor = false;
DrawMap(mapArr, step, gridOn, NodeInfor, distanceArr);
NumberDirect = 8;

while CurrentRow ~= goal(1) || CurrentCol ~= goal(2)
    [CurrentDistance, CurrentBlock] = min(CostValue);
    [CurrentDistance, CurrentCol] = min(CurrentDistance);
    CurrentRow = CurrentBlock(CurrentCol);


    for i = 1:1:NumberDirect
        neighborRow = CurrentRow; neighborCol = CurrentCol;
        switch i
            case 1
                neighborCol = CurrentCol + 1;
                k_cost = 1;
            case 2
                neighborCol = CurrentCol - 1;
                k_cost = 1;
            case 3
                neighborRow = CurrentRow + 1;
                k_cost = 1;
            case 4
                neighborRow = CurrentRow - 1;
                k_cost = 1;
            case 5
                neighborRow = CurrentRow - 1;
                neighborCol = CurrentCol + 1;
                k_cost = 1.4;
            case 6
                neighborRow = CurrentRow - 1;
                neighborCol = CurrentCol - 1;
                k_cost = 1.4;
            case 7
                neighborRow = CurrentRow + 1;
                neighborCol = CurrentCol + 1;
                k_cost = 1.4;
            otherwise
                neighborRow = CurrentRow + 1;
                neighborCol = CurrentCol - 1;
                k_cost = 1.4;
        end

        if neighborRow > row || neighborRow < 1 || ...
                neighborCol > col || neighborCol < 1 || ...
                (neighborRow == start(1) && neighborCol == start(2))
            continue;
        elseif mapArr(neighborRow, neighborCol) == obstacleMark
            continue;
        end

        weight = 0.1;
        distance_to_Goal = sqrt((goal(1) - neighborRow)^2 + (goal(2) - neighborCol)^2);
        costTerm = CurrentDistance + k_cost*cost(neighborRow, neighborCol) + weight*distance_to_Goal;
        if  mapArr(neighborRow, neighborCol) == doneMark || costTerm > CostValueArr(neighborRow, neighborCol)
            continue;
        end

        CostValue(neighborRow, neighborCol) = costTerm;
        CostValueArr(neighborRow, neighborCol) = CostValue(neighborRow, neighborCol);
        distanceArr(neighborRow, neighborCol) = CurrentDistance + k_cost*cost(neighborRow, neighborCol);

        mapArr(neighborRow, neighborCol) = loading;
        parent(neighborRow, neighborCol, :) = [CurrentRow, CurrentCol];
    end

    CostValue(CurrentRow, CurrentCol) = Inf;
    mapArr(CurrentRow, CurrentCol) = doneMark;
    mapArr(start(1), start(2)) = startMark;
    mapArr(goal(1), goal(2)) = goalMark;

    step = step + 1;
    if mod(step, 1000) == 0
        DrawMap(mapArr, step, gridOn, NodeInfor, distanceArr);
        pause(0.001);
    end

end

path = [goal(1), goal(2)];
while parent(path(1, 1), path(1, 2), 1) ~= start(1) || ...
        parent(path(1, 1), path(1, 2), 2) ~= start(2)
    path = [[parent(path(1, 1), path(1, 2), 1), parent(path(1, 1), path(1, 2), 2)]; ...
        path];
    mapArr(path(1, 1), path(1, 2)) = pathMark;
end

% mapArr(cost~=1) = 8;
DrawMap(mapArr, step, gridOn, NodeInfor, distanceArr);

function DrawMap(mapArr, step, gridOn, NodeInfor, distanceArr)
cmap = [1 1 1; ... % white
    0 0 0; ... % black
    0 0 0.9; ... % blue
    0 1 0; ... % green
    1 1 0.8; ... % yellow
    1 0.5 1;...  % gray
    1 0 0]; % red];
colormap(cmap);

row = size(mapArr, 1);
col = size(mapArr, 2);

image(1.5, 1.5, mapArr);
PlotSetUp(gca, gridOn, col, row, step)

if NodeInfor
    for i = 1:1:row
        for j = 1:1:col
            if distanceArr(i, j) ~= inf
                text(j + 0.5, i + 0.5, num2str(distanceArr(i, j)))
            end
        end
    end
end
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