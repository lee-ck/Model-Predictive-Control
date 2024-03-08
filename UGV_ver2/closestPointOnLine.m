function closestPoint = closestPointOnLine(point, lineOrigin, lineEnd)
x1 = point(1);
y1 = point(2);
x2 = lineOrigin(1);
y2 = lineOrigin(2);
x3 = lineEnd(1);
y3 = lineEnd(2);


% Calculate the direction vector of the line
dir_vector = [x3 - x2, y3 - y2];

% Calculate the vector from the second point to the given point
vec_to_point = [x1 - x2, y1 - y2];

% Calculate the scalar projection of vec_to_point onto dir_vector
scalar_projection = dot(vec_to_point, dir_vector) / norm(dir_vector)^2;

% Calculate the closest point on the line
x_closest = x2 + scalar_projection * dir_vector(1);
y_closest = y2 + scalar_projection * dir_vector(2);
closestPoint = [x_closest y_closest];
end

