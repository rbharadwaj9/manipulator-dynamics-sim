function M = spatial_rnea_mass(q, robot_params)
    use_gravity = 0;
    n = length(q);

    if ~robot_params.use_interval
        M = zeros(n,n);
    else
        M = interval(zeros(n,n), zeros(n,n));
    end

    qd = zeros(n,1);
    qd_a = zeros(n,1);
    qdd = eye(n);
    
    Ftip = zeros(6,1);
    
    % calculate mass matrix
    for i = 1:n
        M(:,i) = spatial_rnea(q, qd, qd_a, qdd(:,i), use_gravity, Ftip, robot_params);
    end
end

