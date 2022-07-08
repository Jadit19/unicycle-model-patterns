function plot_graph(x, y, r_min, r_max)
    angle = 0:0.01:2*pi;
    figure;
    set(gcf, 'Position', [1000 100 800 800]);
    hold on;
    grid on;
    h = animatedline;
    plot(r_min*cos(angle), r_min*sin(angle), '--r');
    plot(r_max*cos(angle), r_max*sin(angle), '--r');
    for i = 1:length(x)
        addpoints(h, x(i), y(i));
        drawnow;
    end
    hold off;
end