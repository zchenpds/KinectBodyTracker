function [x2, y2] = my_transform(x, y, T)
    x = x + T.x;
    y = y + T.y;
    x2 = x * cos(T.th) + y * sin(T.th);
    y2 = -x * sin(T.th) + y * cos(T.th);
end