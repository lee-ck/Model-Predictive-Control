function input = yaw_discontinuity(input)


for i = 1 : size(input.x,1)-1
    a(1,1) = abs(input.x(i,3) - input.y(i,3));
    a(1,2) = abs(input.x(i,3) - (input.y(i,3) - 2*pi)); 
    a(1,3) = abs(input.x(i,3) - (input.y(i,3) + 2*pi));
    [~, index] = min(a);
    
    if index == 1
        input.y(i,3) = input.y(i,3);
    elseif index == 2
        input.y(i,3) = input.y(i,3) - 2*pi;
    elseif index == 3
        input.y(i,3) = input.y(i,3) + 2*pi;
    end
end


a(1,1) = abs(input.x(end,3) - input.yN(1,3));
a(1,2) = abs(input.x(end,3) - (input.yN(1,3) - 2*pi)); 
a(1,3) = abs(input.x(end,3) - (input.yN(1,3) + 2*pi));
[~, index] = min(a);

if index == 1
    input.yN(1,3) = input.yN(1,3);
elseif index == 2
    input.yN(1,3) = input.yN(1,3) - 2*pi;
elseif index == 3
    input.yN(1,3) = input.yN(1,3) + 2*pi; 
end

end
