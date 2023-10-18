% Read Image
image = imread('desk2.jpg');
gray_image = double(im2gray(image));
imshow(image);
hold on 
[image_rows, image_cols, channels] = size(image);

% Get Start and End Points via GUI
[start_col,start_row] = ginput(1);
[end_col,end_row] = ginput(1);

% Round coordnates of start and end points
start_col = round(start_col);
start_row = round(start_row);
end_col = round(end_col);
end_row = round(end_row);

% Display the clicked points
start_end_cols = [start_col end_col];
start_end_rows = [start_row end_row];
scatter(start_end_cols, start_end_rows)

hold on 

% Compute cost response of each link from each point
leftOrthogonalGradientFilter= [1  1  0;
                               0  0  0;
                              -1 -1  0];

upOrthogonalGradientFilter= [-1  0  1;
                             -1  0  1;
                              0  0  0;];

upLeftOrthogonalGradientFilter = [ 0 1 0;
                                  -1 0 0;
                                   0 0 0;];

upRightOrthogonalGradientFilter = [ 0  1  0;
                                    0  0 -1;
                                    0  0  0;];

leftOrthogonalGradient = imfilter(gray_image, leftOrthogonalGradientFilter, ...
                    "replicate", "same", "corr");
upOrthogonalGradient = imfilter(gray_image, upOrthogonalGradientFilter, ...
                    "replicate", "same", "corr");

upLeftOrthogonalGradient = imfilter(gray_image, upLeftOrthogonalGradientFilter, ...
                    "replicate", "same", "corr");

upRightOrthogonalGradient = imfilter(gray_image, upRightOrthogonalGradientFilter, ...
                    "replicate", "same", "corr");

leftOrthogonalGradient = (-abs(leftOrthogonalGradient) + 255)/ 4;
upOrthogonalGradient = (-abs(upOrthogonalGradient) + 255) / 4;
upLeftOrthogonalGradient = (-abs(upLeftOrthogonalGradient) + 255) / sqrt(2);
upRightOrthogonalGradient = (-abs(upRightOrthogonalGradient) + 255) / sqrt(2);

% Dijkstra's Shortest Path Algorithm
costToPoint = Inf(size(image));
costToPoint(start_row, start_col) = 0; %set cost to initial point to 0
costToP = 0;

p = [start_row start_col];  % Starting point

%2D Array Containing node objects that keep track of the path to a point
path(image_rows, image_cols) = Node;

% mark explored nodes when after a visit with integer 1.
explored = uint8(zeros(size(image)));
explored(start_row, start_col) = 1;

% Run Dikstra's Shortest Path Algorithm until p == end_point
while ~isequal(p, [end_row end_col])
    % links costs index corressponds clockwise to link starting from up direction.
    linkCosts = zeros(1, 8);

    % Up
    if p(1) - 1 > 0
    linkCosts(1) = upOrthogonalGradient(p(1), p(2));
        [costToPoint(p(1) - 1, p(2)), I] = min([linkCosts(1) + costToP, costToPoint(p(1) - 1, p(2))]);
        if I == 1 && explored(p(1) - 1, p(2)) == 0
            % If new min cost is found for node update its path and add to the
            % active list.
            
            path(p(1) - 1, p(2)).prevPathRow = [path(p(1), p(2)).prevPathRow (p(1) - 1)];
            path(p(1) - 1, p(2)).prevPathCol = [path(p(1), p(2)).prevPathCol p(2)];
        end
    end

    % Up Right [p(1) - 1, p(2) + 1]
    if p(1) - 1 > 0 && p(2) + 1 < image_cols
        linkCosts(2) = upRightOrthogonalGradient(p(1), p(2));
        [costToPoint(p(1) - 1, p(2) + 1), I] = min([linkCosts(2) + costToP, costToPoint(p(1) - 1, p(2) + 1)]);
        if I == 1 && explored(p(1) - 1, p(2) + 1) == 0
            path(p(1) - 1, p(2) + 1).prevPathRow = [path(p(1), p(2)).prevPathRow (p(1) - 1)];
            path(p(1) - 1, p(2) + 1).prevPathCol = [path(p(1), p(2)).prevPathCol (p(2) + 1)];
        end
    end
    
    % Right
    if p(2) + 1 < image_cols
        linkCosts(3)  = leftOrthogonalGradient(p(1), p(2)+1);
        [costToPoint(p(1), p(2) + 1), I] = min([linkCosts(3) + costToP, costToPoint(p(1), p(2) + 1)]);
        if I == 1 && explored(p(1), p(2) + 1) == 0
            path(p(1), p(2) + 1).prevPathRow = [path(p(1), p(2)).prevPathRow (p(1))];
            path(p(1), p(2) + 1).prevPathCol = [path(p(1), p(2)).prevPathCol (p(2) + 1)];
        end
    end
    
    % Right Down
    if p(1) + 1 < image_rows && p(2) + 1 < image_cols
        linkCosts(4)  = upLeftOrthogonalGradient(p(1) + 1, p(2) + 1);
        [costToPoint(p(1) + 1, p(2) + 1), I] = min([linkCosts(4) + costToP, costToPoint(p(1) + 1, p(2) + 1)]);
        if I == 1 && explored(p(1) + 1, p(2) + 1) == 0
            path(p(1) + 1, p(2) + 1).prevPathRow = [path(p(1), p(2)).prevPathRow (p(1) + 1)];
            path(p(1) + 1, p(2) + 1).prevPathCol = [path(p(1), p(2)).prevPathCol (p(2) + 1)];
        end
    end
    
    % Down
    if p(1) + 1 < image_rows
        linkCosts(5)  = upOrthogonalGradient(p(1) + 1, p(2));
        [costToPoint(p(1) + 1, p(2)), I] = min([linkCosts(5) + costToP, costToPoint(p(1) + 1, p(2))]);
        if I == 1 && explored(p(1) + 1, p(2)) == 0
            path(p(1) + 1, p(2)).prevPathRow = [path(p(1), p(2)).prevPathRow (p(1) + 1)];    
            path(p(1) + 1, p(2)).prevPathCol = [path(p(1), p(2)).prevPathCol (p(2))];   
        end
    end

    % Down Left
    if p(1) + 1 < image_rows && p(2) - 1 > 0
        linkCosts(6)  = upRightOrthogonalGradient(p(1) + 1, p(2) - 1);
        [costToPoint(p(1) + 1, p(2) - 1), I] = min([linkCosts(6) + costToP, costToPoint(p(1) + 1, p(2) - 1)]);
        if I == 1 && explored(p(1) + 1, p(2) - 1) == 0
            path(p(1) + 1, p(2) - 1).prevPathRow = [path(p(1), p(2)).prevPathRow (p(1) + 1)];
            path(p(1) + 1, p(2) - 1).prevPathCol = [path(p(1), p(2)).prevPathCol (p(2) - 1)];
        end
    end
    
    % Left
    if p(2) - 1 > 0
        linkCosts(7)  = leftOrthogonalGradient(p(1), p(2));
        [costToPoint(p(1), p(2) - 1), I] = min([linkCosts(7) + costToP, costToPoint(p(1), p(2) - 1)]);
        if I == 1 && explored(p(1), p(2) - 1) == 0
            path(p(1), p(2) - 1).prevPathRow = [path(p(1), p(2)).prevPathRow (p(1))];
            path(p(1), p(2) - 1).prevPathCol = [path(p(1), p(2)).prevPathCol (p(2) - 1)];
        end
    end

    % Left Up
    if p(1) - 1 > 0 && p(2) - 1 > 0
        linkCosts(8)  = upLeftOrthogonalGradient(p(1), p(2));
        [costToPoint(p(1) - 1, p(2) - 1), I] = min([linkCosts(8) + costToP, costToPoint(p(1) - 1, p(2) - 1)]);
        if I == 1 && explored(p(1) - 1, p(2) - 1) == 0
            path(p(1) - 1, p(2) - 1).prevPathRow = [path(p(1), p(2)).prevPathRow (p(1) - 1)];
            path(p(1) - 1, p(2) - 1).prevPathCol = [path(p(1), p(2)).prevPathCol (p(2) - 1)];
        end
    end

    %Determine point for next interation
    minCost = Inf;
    minI = 0;
    minJ = 0;
    for i = 1 : image_rows
        for j = 1 : image_cols
            if explored(i, j) == 0 && costToPoint(i, j) < minCost
                minCost = costToPoint(i, j);
                minI = i;
                minJ = j;
            end
        end
    end
    
    % new P
    p = [minI minJ];
    costToP = minCost;
    
    % Mark P as explored
    explored(p(1), p(2)) = 1;
     
end

% Display Shortest Path
shortestPathPointsRow = path(end_row, end_col).prevPathRow;
shortestPathPointsCol = path(end_row, end_col).prevPathCol;
for i = 1: length(shortestPathPointsCol)
    plot(shortestPathPointsCol, shortestPathPointsRow);
    hold on;
end











