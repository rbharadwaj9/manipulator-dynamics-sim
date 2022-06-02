function C = spatial_rnea_coriolis(q, qd, robot_params)
    use_gravity = 0;
    n = length(q);

    if ~robot_params.use_interval
        C = zeros(n,n);
    else
        C = interval(zeros(n,n), zeros(n,n));
    end

    qd_a = eye(n);
    qdd = zeros(n,1);
    
    Ftip = zeros(6,1);
    
    % calculate coriolis matrix
    for i = 1:n
        C(:,i) = spatial_rnea(q, qd, qd_a(:,i), qdd, use_gravity, Ftip, robot_params);
    end
end
