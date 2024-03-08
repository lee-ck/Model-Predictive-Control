function result = isLeft(a, b, c)
if (b(1) - a(1))*(c(2) - a(2)) - (b(2) - a(2))*(c(1) - a(1)) > 0
    result = 1;
else
    result = -1;
end
end

