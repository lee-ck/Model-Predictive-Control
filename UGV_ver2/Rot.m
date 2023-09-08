function R = Rot(c,t)


    switch c
        case 'x'
            R = [1 0 0;
                0 cos(t) -sin(t);
                0 sin(t) cos(t)];
        case 'y'
            R = [cos(t) 0 sin(t);
                0 1 0;
                -sin(t) 0 cos(t)];
        case 'z'
            R = [cos(t) -sin(t) 0;
                sin(t) cos(t) 0;
                0 0 1];
        otherwise
                error('Choose a valid axis')
    end



end