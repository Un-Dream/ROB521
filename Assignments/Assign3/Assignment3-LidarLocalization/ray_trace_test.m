% Define the map size
map_size = [10, 10];

% Call the ray tracing function to generate rr and cc
[rr, cc] = ray_trace(60, 140, 60, 190)

% Initialize the indices that are within the map boundary
valid_indices = (rr >= 1) & (rr <= map_size(1)) & (cc >= 1) & (cc <= map_size(2));

% Filter the rr and cc arrays based on the valid indices
rr = rr(valid_indices);
cc = cc(valid_indices);

% Ensure that rr and cc have the same number of elements
min_length = min(length(rr), length(cc));
rr = rr(1:min_length);
cc = cc(1:min_length);

% given two points in a matrix, returns the diagonal indices 
function [rr, cc] = ray_trace(x1, y1, x2, y2)
    % Initialize the return arrays
    rr = [];
    cc = [];
    
    % Calculate the x and y distances between the two points
    dx = abs(x2 - x1);
    dy = abs(y2 - y1);
    
    % Calculate the step size for the x and y axes
    if x1 < x2
        sx = 1;
    else
        sx = -1;
    end
    
    if y1 < y2
        sy = 1;
    else
        sy = -1;
    end
    
    % Initialize the error term
    err = dx - dy;
    
    % Traverse the line using Bresenham's algorithm
    while true
        % Add the current point to the return arrays
        rr = [rr; y1];
        cc = [cc; x1];
        
        % Check if we've reached the end point
        if x1 == x2 && y1 == y2
            break;
        end
        
        % Update the error term
        e2 = 2*err;
        if e2 > -dy
            err = err - dy;
            x1 = x1 + sx;
        end
        if e2 < dx
            err = err + dx;
            y1 = y1 + sy;
        end
    end
end



