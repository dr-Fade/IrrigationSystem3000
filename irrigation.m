#
# The model needs to know what you want from it. There are parameters that you have to change to extend the model
# or alter its behavior:
# 
# y_ref - a vector that holds values the model will try to drive the state vector to. The first value is irrelevant since it
# describes the water levels outside the pools.
#
# tau - a vector that holds delay values. Change it to make the distance between i and i+1 gates larger.
#
# Tff, Tf - functions that describe the "feed forward control" and the "local feedback", see (https://www.sciencedirect.com/science/article/pii/S1474667016381046) for details.
#
# d - a function that describes the rate of water consumption from a given pool.
#
# Since octave's ODE solver is dumb enough to not allow initializing the state arrays in loops,
# we have to do this stuff by hand :D 
#
# Just copy-paste the latest component and set the appropriate indeces.
#

function irrigation()

    function ydot = irrigation_system(y, t, tau, y_ref, u, d) 
        ydot(1) = y_ref(1);
        ydot(2) = u{1}(t - tau(2), y) - u{2}(t, y) - d(t);
        ydot(3) = u{2}(t - tau(3), y) - u{3}(t, y) - d(t);
    end

    function d = disturbance(r, x)
        dt = 0.5;
        d = dt;
        for i = 1 : x 
            d = r * dt * (1 - dt);
            dt = d;
        end 

        d = d * 5;
    end

    #
    # time constraints
    #
    t0 = 0;
    dt = 0.01;
    T = 100;
    t = t0 : dt : T;

    #
    # initial conditions
    #
    y0 = [0 0 0];

    #
    # values that the controllers will try to achieve 
    #
    y_ref = [0 0 40];

    #
    # feedback functions
    #
    Tff = @(x, t) x + disturbance(3.9, mod(t, 100));
    Tf = @(x, t) x + disturbance(3.8, mod(t, 100)); 
    d = @(t) 10 + disturbance(3.7, mod(t, 100));

    #
    # controllers
    #
    tau = [1 1 1];
    u{length(y_ref)} = @(t,y) 0;
    for i = length(y_ref)-1 : -1 : 1
        u{i} = @(t, y) Tff(i, t) * (d(i) + u{i+1}(t,y)) + Tf(i, t) * (y_ref(i+1) - y(i+1))
    end

    f = @(y, t) irrigation_system(y, t, tau, y_ref, u, d);
    trajectory = lsode(f, y0, t);

    hold on;
    plot(t, trajectory(:,2));
    plot(t, trajectory(:,3));

end
