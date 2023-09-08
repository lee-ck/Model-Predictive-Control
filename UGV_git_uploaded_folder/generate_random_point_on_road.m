function random_point = generate_random_point_on_road(point1, point2, width)
    % Generate a random weight to interpolate between point1 and point2
    t = rand;
    
    % Interpolate along the line between point1 and point2
    interp_point = (1-t)*point1 + t*point2;
    
    % Compute the vector from point1 to point2
    vec = point2 - point1;
    
    % Normalize the vector
    unit_vec = vec / norm(vec);
    
    % Compute the perpendicular vector
    perp_vec = [-unit_vec(2), unit_vec(1)];
    
    % Generate a random weight for the perpendicular displacement
    s = (rand - 0.5) * width; % random number between -width/2 and +width/2
    
    % Calculate the random point on the road
    random_point = interp_point + s * perp_vec;
end