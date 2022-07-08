%% Basics
clc;
clear all;
close all;

%% Changeable parameters
global v r_min r_max;
v = 1;
r_min = 20;
r_max = 40;

%% Global variables
global count flag1 flag2 sp;
count = 0;                      % No. of times switching point has been crossed from last switching
flag1 = 0;                      % No. of times trajectory has been switched
flag2 = 1;                      % Bot is available to switch or not (boolean)

%% Symbolic functions and variables
global r g1 g2 f1 f2;
syms r g1(r) g2(r) f1(r) f2(r);
g1(r) = -1 * generate_function(v, r, r_min, r_max);
g2(r) = generate_function(v, r, r_min, r_max);
f1(r) = diff(g1, r) / r;
f2(r) = diff(g2, r) / r;

%% Solving for initial switching point
eqn = g1(r) == g2(r);
sp = double(vpasolve(eqn, r, [r_min r_max]));   % Bot can only switch between r_min and r_max

%% Initial conditions
r0 = r_min;                                         % Radius where the bot starts from
theta0 = 0;                                         % Angle where the bot starts from
phi0 = 3 * pi / 2;
alpha0 = 3 * pi / 2;                                % = (phi-theta)
options = odeset('RelTol', 1e-8, 'AbsTol', 1e-8);   % ODE Solver sensitivity

%% Trajectory Generation
[t1, Y1] = ode23(@(t1, Y1) odefunc(t1, Y1), [0, 1000], [r0 phi0 alpha0 theta0], options);
[x1, y1] = pol2cart(Y1(:, 4), Y1(:, 1));
plot_graph(x1, y1, r_min, r_max);

%% ODE Function
function ret = odefunc(t, Y)
    global count flag1 flag2;
    global v sp;
    global f1 f2;

    f_ = 0;
    r_dash = -1 * v * cos(Y(2));
    
    if (flag2 == 1)
        if (mod(count, 2) == 0)
            if (Y(1) > sp)
                % Switching point crossed with r_dash > 0
                count = count + 1
                if (count == 6)
                    flag1 = flag1 + 1;
                    flag2 = 0;
                    count = 0;
                end
            end
        else
            if (Y(1) < sp)
                % Switching point crossed with r_dash < 0
                count = count + 1
                if (count == 6)
                    flag1 = flag1 + 1;
                    flag2 = 0;
                    count = 0;
                end
            end
        end
    end

    if (flag2 == 0)
        if (Y(1)>=(sp+2) || Y(1)<=(sp-2))
            % Bot is clear of the switching point
            flag2 = 1;
        end
    end

    if (mod(flag1, 2) == 0)
        f_ = double(f1(Y(1)));
    else
        f_ = double(f2(Y(1)));
    end

    ret(1, 1) = r_dash;                     % r' equation
    ret(2, 1) = f_ + (v/Y(1))*sin(Y(2));    % phi' equation
    ret(3, 1) = f_;                         % alpha' equation
    ret(4, 1) = -1*(v/Y(1))*sin(Y(2));      % theta' equation
end