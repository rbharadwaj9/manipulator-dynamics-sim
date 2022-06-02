function animate(Qres, Qdes, robot_config)
    % Animate resulting configuration against desired configuration for a
    % given robot.
    Qres = Qres';
    % Convert theta to cartesian
    x_res_pos = robot_config.L1.*cos(Qres(:,1));
    y_res_pos = robot_config.L1.*sin(Qres(:,1));
    x_des_pos = robot_config.L1.*cos(Qdes(:,1));
    y_des_pos = robot_config.L1.*sin(Qdes(:,1));
    figure;
    plot(x_res_pos,y_res_pos, 'b');
    hold on;
    plot(x_des_pos,y_des_pos, 'k');
    title("Aggregate Trajectory in X-Y Space");
    legend("Result", "Desired");

    % Animate
    
    f = figure;
    for k = 1 : length(x_res_pos)
      figure(f.Number);
      plot(x_res_pos(k), y_res_pos(k), 'r*-', 'LineWidth', 2);
      xlim([-2, 2]);
      ylim([-2, 2]);
      grid on;
      hold on;
      plot(x_des_pos(k), y_des_pos(k), 'b*-', 'LineWidth', 2);
      
      line([0,x_res_pos(k)], [0, y_res_pos(k)], 'Color', 'red', 'LineWidth', 1);
      line([0,x_des_pos(k)], [0, y_des_pos(k)], 'Color', 'blue', 'LineStyle', '-.', 'LineWidth', 1);
      hold off;
      title("Live");
      legend("Result", "Desired");
    end
end
