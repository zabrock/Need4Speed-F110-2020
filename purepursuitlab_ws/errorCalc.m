% ECE599 F1/10 Racing
% Hw 06
% Error calculation
clc, clear

% get the data of goal path
goal = csvread('new_waypoints.csv');
goalX  = goal(:,1);
goalY  = goal(:,2);
goalTh = goal(:,3) * 180/pi;

% get the data of pure pursuit path
pursuit = csvread('pp_path.csv');
pursuitX  = pursuit(:,1);
pursuitY  = pursuit(:,2);
pursuitTh = pursuit(:,3) *180/pi;

% initialize error
errData = zeros(length(pursuitTh),1);

for n1=1:length(pursuitTh)
    % goalNeigh (X1,Y1), (X2,Y2)
    % : two neighboring points on the goal path
    goalNeighX1 = goalX(2);
    goalNeighY1 = goalY(2);
    goalNeighX2 = goalX(1);
    goalNeighY2 = goalY(1);
    minDist = sqrt( (pursuitX(n1)-goalX(2))^2   ...
                   +(pursuitY(n1)-goalY(2))^2 ) ...
            + sqrt( (pursuitX(n1)-goalX(1))^2   ...
                   +(pursuitY(n1)-goalY(1))^2 );   % case n2=2 
    for n2 = 3:length(goalTh)   % start at n2=3
        dist = sqrt( (pursuitX(n1)-goalX(n2  ))^2   ...
                    +(pursuitY(n1)-goalY(n2  ))^2 ) ...
             + sqrt( (pursuitX(n1)-goalX(n2-1))^2   ...
                    +(pursuitY(n1)-goalY(n2-1))^2 );
        if (minDist > dist)
            minDist = dist;
            goalNeighX1 = goalX(n2  );
            goalNeighY1 = goalY(n2  );
            goalNeighX2 = goalX(n2-1);
            goalNeighY2 = goalY(n2-1);
        end
    end
    % Get the error by calculating the perpendicular distance 
    % from the goal path using Heron's Formula
    % : a,b,c are the length of triangle
    %   b,h are the base and height of triangle
    %   A is the area of triangle
    %   Solve for h, which is equivalent to error value
    a = sqrt( (pursuitX(n1)-goalNeighX1)^2  ...
             +(pursuitY(n1)-goalNeighY1)^2 );
    b = sqrt( (goalNeighX1 -goalNeighX2)^2  ...
             +(goalNeighY1 -goalNeighY2)^2 );
    c = sqrt( (pursuitX(n1)-goalNeighX2)^2  ...
             +(pursuitY(n1)-goalNeighY2)^2 );
    s = (a + b + c) / 2;
    A = sqrt(s * (s-a) * (s-b) * (s-c));
    h = A / b;
    errData(n1) = h;
end

errData
csvwrite('ErrorData.csv',errData)
maxAbsError   = max(errData)
totalAbsError = sum(errData)
numOfData     = length(errData)

txtFileWrite = fopen('ErrorSummary.txt','w');
fprintf(txtFileWrite, 'maxAbsError: %f\n', maxAbsError);
fprintf(txtFileWrite, 'totalAbsError: %f\n', totalAbsError);
fprintf(txtFileWrite, 'numOfData: %d\n', numOfData);
fclose(txtFileWrite);