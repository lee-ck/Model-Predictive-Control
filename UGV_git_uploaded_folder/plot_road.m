function plot_road(point1, point2, width, c, a, b)
    % Input: point1 and point2 are the [x, y] coordinates of the two points defining the road
    %        width is the width of the road
    %
    % This function plots a road between point1 and point2 with the given width.
    
    % Compute the vector from point1 to point2
    vec = point2 - point1;
    
    % Normalize the vector
    unit_vec = vec / norm(vec);
    
    % Compute the perpendicular vector
    perp_vec = [-unit_vec(2), unit_vec(1)];
    
    % Compute the four corners of the road
    corner1 = point1 + width/2 * perp_vec;
    corner2 = point1 - width/2 * perp_vec;
    corner3 = point2 - width/2 * perp_vec;
    corner4 = point2 + width/2 * perp_vec;
    
    % Plot the road as a filled polygon
    x_corners = [corner1(1), corner2(1), corner3(1), corner4(1), corner1(1)];
    y_corners = [corner1(2), corner2(2), corner3(2), corner4(2), corner1(2)];
    
    h = fill(x_corners, y_corners, c);
    h.FaceAlpha = a;
    h.EdgeAlpha = b;
%     hold on;
    
    % Plot the points for reference
%     plot([point1(1), point2(1)], [point1(2), point2(2)], 'ro');
    
    % Set the axis for better visualization
%     axis equal;
end