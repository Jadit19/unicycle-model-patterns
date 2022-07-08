function ret = generate_function(v, r, r_min, r_max)
    n = rand();
    if (n < 0.1)
        % Linear function generation - working
        m = v * (r_max+r_min) / (r_max-r_min);
        c = (v-m) * r_max;
        ret = m*r + c;
    elseif (n < 0.4)
        % Quadratic function generation - working
        r1 = 0;                                     % Point of extrema
        if (n < 0.25)
            % Upward facing parabola
            r1 = rand() * r_min;
        else
            % Downward facing parabola
            r1_min = ((r_max+r_min)^2) / (4*r_min);
            r1 = r1_min * (rand() + 1);
        end
        a = v * (r_max+r_min) / ((r_max-r_min) * ((r_max+r_min)/2 - r1));
        b = v*r_max + a*r1*r_max - a*r_max*r_max/2;
        ret = a*r*r/2 - a*r1*r + b;
    else
        % Cubic function generation - working
        r1 = 0;                                     % Points of extrema,
        r2 = 0;                                     % with r1 < r2
        if (n < 0.6)
            % Both extremas lie to the left of r_min
            r1 = -1 * rand() * r_max;
            r2 = rand() * r_min;
        elseif (n < 0.8)
            % r_min and r_max lie within the two extremas
            r1 = rand() * r_min;
            r2_min = ((r_max+r_min)^2) / (4*r_min);
            r2 = r2_min * (rand() + 1);
        else
            % Both extremas lie to the right of r_max
            min_r = ((r_max+r_min)^2) / (4*r_min);
            r1 = min_r * (rand() + 1);
            r2 = min_r * (rand() + 2);
        end
        a = v * (r_max+r_min) / ((r_max-r_min) * ((r_max^2+r_min^2+r_max*r_min)/3 + r1*r2 - (r1+r2)*(r_max+r_min)/2));
        b = v*r_max - (a/3)*(r_max^3) + (a*(r1+r2)/2)*(r_max^2) - a*r1*r2*r_max;
        ret = a*r*r*r/3 - a*(r1+r2)*r*r/2 + a*r1*r2*r + b;
    end
end