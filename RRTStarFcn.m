function [pathWPX, pathWPY] = RRTStarFcn(startX, startY, goalX, goalY)
% 2243716 MEng Project 
% Fault Tolerant Coordination of Multiple Rovers for Planetary Exploration
% Adapted from Sai Vemprala: RRT* algorithm in 2D with collision avoidance
%---------------------------------------------------------------------%

% Path Planner RRTStar Algorithm
% Code Summary: 
% 1) Initialise start point, target point, map boundaries, max nodes, max
%    step size, obstacles, threshold distance from target to goal, node list
% 2) Loop until maxNodes is reached: 
    % 2.1) Create a random node
    % 2.2) Check if goalPoint has been reached
    % 2.3) Find the closest explored node to random node, and branch
    % 2.4) Ensure there are no obstacle collisions
    % 2.5) Calculate the cost to the new point
    % 2.6) Evaluate the cost to new point against nearest neighbours 
    % 2.7) Update parent list with lowest cost points 
    % 2.8) Find the distance between nodes and target 
    % 2.9) Find optimal path searching backwards from optimal point 

 
% Modifications required: 
% 1) New obstacle definitions: Canyon Test Trapezoid -- Done! 
%   1.1) Handle multiple obstacles
% 2) Output final path as an array of waypoints
% 3) Redefine cost function 
%---------------------------------------------------------------------%
%%%%% Initial Segment 
%---------------------------------------------------------------------%
%clearvars
close all

% Define start and goal points
startPoint.coord = [startX startY];      
startPoint.cost = 0;
startPoint.parent = 0; 
goalPoint.coord = [goalX goalY];
goalPoint.cost = 0;

% Define map variables 
xBoundary = 25;
yBoundary = 25; 
maxNodes = 1000; 
maxStepSize = 1;      % max step size in m 

% Define obstacles and terrain objects 
obsXOne = [0 0 5.2 5.2];
obsYOne = [2.8 25 25 6.8];
obsXTwo = [7.8 9.8 15.2 15.2];
obsYTwo = [0 18.2 20.2 0];
obsXThree = [5 5 25 25];
obsYThree = [23.8 25 25 23.8];
obsXFour =[23.8 23.8 25 25];   
obsYFour = [0 24 24 0];     
rockFieldX = [15 15 24 24]; 
rockFieldY = [10 15 18 13];
steepSlopeXOne = [0 0 6 6]; 
steepSlopeYOne = [2 24 24 6];
steepSlopeXTwo = [7 9 15.5 16 ]; 
steepSlopeYTwo = [0 19 21 0];
steepSlopeXThree = [6 6 25 25]; 
steepSlopeYThree = [23.5 25 25 23.5];
steepSlopeXFour = [23 23 25 25]; 
steepSlopeYFour = [0 23.5 23.5 0];
obstacleOneP = polyshape(obsXOne, obsYOne);
obstacleTwoP = polyshape(obsXTwo, obsYTwo);
obstacleThreeP = polyshape(obsXThree, obsYThree);
obstacleFourP = polyshape(obsXFour, obsYFour);
rockField = polyshape(rockFieldX,rockFieldY);
steepSlopeOne = polyshape(steepSlopeXOne, steepSlopeYOne);
steepSlopeTwo = polyshape(steepSlopeXTwo, steepSlopeYTwo);
steepSlopeThree = polyshape(steepSlopeXThree, steepSlopeYThree);
steepSlopeFour = polyshape(steepSlopeXFour, steepSlopeYFour);

% Plot environment objects
plot(steepSlopeOne, 'FaceColor', 'red', 'FaceAlpha', 0.2)
hold on
plot(steepSlopeTwo, 'FaceColor', 'red', 'FaceAlpha', 0.2)
plot(steepSlopeThree, 'FaceColor', 'red', 'FaceAlpha', 0.2)
plot(steepSlopeFour, 'FaceColor', 'red', 'FaceAlpha', 0.2)
plot(obstacleOneP, 'FaceColor', 'black', 'FaceAlpha', 0.8) 
plot(obstacleTwoP, 'FaceColor', 'black', 'FaceAlpha', 0.8)
plot(obstacleThreeP, 'FaceColor', 'black', 'FaceAlpha', 0.8)
plot(obstacleFourP, 'FaceColor', 'black', 'FaceAlpha', 0.8)
plot(rockField, 'FaceColor', 'green', 'FaceAlpha', 0.4)
plot(goalPoint.coord)
plot(goalPoint.coord(1), goalPoint.coord(2), '--o','MarkerSize',15, 'MarkerFaceColor',[0.75, 0, 0.75]);
hold on

% Plot map 
nodes(1) = startPoint;
figure(1)
axis([0 xBoundary 0 yBoundary])
title('Rover Environment')
xlabel('X Position (m)')
ylabel('Y Position (m)')
hold on

%---------------------------------------------------------------------%
%%%%% Map Creation Segment 
%---------------------------------------------------------------------%
for i = 1:1:maxNodes
    randomPoint = [floor(rand(1)*xBoundary) floor(rand(1)*yBoundary)];   % Find random coords
    plot(randomPoint(1), randomPoint(2), 'x', 'Color',  [0 0.4470 0.7410])    % Plot new random point 

    % Nearest Neighbour Search 
    nodeDistanceArray = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        distanceToNode = PathPlannerDistance(n.coord, randomPoint); % + PathPlannerCostClassification(n.coord,maxStepSize,rockField,steepSlopeOne,steepSlopeTwo,steepSlopeThree,steepSlopeFour); Adding this allows path to go within obstacles
        nodeDistanceArray = [nodeDistanceArray distanceToNode];
    end
    
    % Select nearest explored node  to randomPoint 
    [val, idx] = min(nodeDistanceArray);
    nearestNode = nodes(idx);      
    
    % Set new node by moving towards random point for maxStepSize
    newPoint.coord = PathPlannerSteer(randomPoint, nearestNode.coord, val, maxStepSize);
    
    % Check if new point is within 1m of target
    if sqrt((newPoint.coord(1)-goalPoint.coord(1))^2 + (newPoint.coord(2)-goalPoint.coord(2))^2) <= 1
        break   % Break if condition is true 
    end
    
   % Check for obstacle collisions between random point and nearest node??
   % (changed it to new point and nearest node, because that makes more
   % sense?
    if PathPlannerCollisionCheck(newPoint.coord, nearestNode.coord, obsXOne, obsYOne, obsXTwo, obsYTwo, obsXThree, obsYThree, obsXFour, obsYFour)
        % Draw line between new point and the nearest explored node 
        line([nearestNode.coord(1), newPoint.coord(1)], [nearestNode.coord(2), newPoint.coord(2)], 'Color', 'k', 'LineWidth', 2);
        drawnow
        hold on
        
        % calculate the cost of going to the new point from nearest node 
        newPoint.cost = PathPlannerDistance(newPoint.coord, nearestNode.coord) + nearestNode.cost + PathPlannerCostClassification(newPoint.coord,maxStepSize,rockField,steepSlopeOne,steepSlopeTwo,steepSlopeThree,steepSlopeFour);
        
        % Within a radius of r, find all existing nodes
        nearestNeighbours = [];
        r = 3; % 3X max step as per original code 
        
        
        neighbor_count = 1;
        for j = 1:1:length(nodes)
            if  PathPlannerCollisionCheck(nodes(j).coord, newPoint.coord, obsXOne, obsYOne, obsXTwo, obsYTwo, obsXThree, obsYThree, obsXFour, obsYFour) && PathPlannerDistance(nodes(j).coord, newPoint.coord) <= r; % &&  PathPlannerCostClassification(nodes(j).coord,maxStepSize,rockField,steepSlopeOne,steepSlopeTwo,steepSlopeThree,steepSlopeFour) ~= 9;
                nearestNeighbours(neighbor_count).coord = nodes(j).coord;
                nearestNeighbours(neighbor_count).cost = nodes(j).cost + PathPlannerCostClassification(nearestNeighbours(neighbor_count).coord,maxStepSize,rockField,steepSlopeOne,steepSlopeTwo,steepSlopeThree,steepSlopeFour);
                neighbor_count = neighbor_count+1;
            end
        end
        
        % Initialize cost to currently known value
        q_min = nearestNode;
        costMin = newPoint.cost;
        
        % Iterate through all nearest neighbors to find alternate lower
        % cost paths
        
        for k = 1:1:length(nearestNeighbours)
            if PathPlannerCollisionCheck(nearestNeighbours(k).coord, newPoint.coord, obsXOne, obsYOne, obsXTwo, obsYTwo, obsXThree, obsYThree, obsXFour, obsYFour) && nearestNeighbours(k).cost + PathPlannerDistance(nearestNeighbours(k).coord, newPoint.coord) < costMin
                q_min = nearestNeighbours(k);
                costMin = nearestNeighbours(k).cost + PathPlannerDistance(nearestNeighbours(k).coord, newPoint.coord); % add cost here 
%                  line([q_min.coord(1), newPoint.coord(1)], [q_min.coord(2), newPoint.coord(2)], 'Color', 'g');                
%                  hold on
            end
        end
        
        % Update parent to least cost-from node
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                newPoint.parent = j;
            end
        end
        
        % Append to nodes
        nodes = [nodes newPoint];
    end
    
    
end
%---------------------------------------------------------------------%
%%%%% Path Creation Segment 
%---------------------------------------------------------------------%

% create an empty array and fill with distance from nodes to goal
D = [];
for j = 1:1:length(nodes)
    currentDistance = PathPlannerDistance(nodes(j).coord, goalPoint.coord) + PathPlannerCostClassification(nodes(j).coord,maxStepSize,rockField,steepSlopeOne,steepSlopeTwo,steepSlopeThree,steepSlopeFour);
    D = [D currentDistance];
end

% Find optimal path (lowest cost) searching backwards from goal point
[~, idx] = min(D);
goalPoint.parent = idx;
endPoint = goalPoint;
nodes = [nodes goalPoint];
pathWPX = endPoint.coord(1);
pathWPY = endPoint.coord(2);
while endPoint.parent ~= 0
    start = endPoint.parent;
    line([endPoint.coord(1), nodes(start).coord(1)], [endPoint.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
    hold on
    endPoint = nodes(start);
    pathWPX = [pathWPX endPoint.coord(1)];
    pathWPY = [pathWPY endPoint.coord(2)];
end
%---------------------------------------------------------------------%
%%%%% END
%---------------------------------------------------------------------%
end

