function g = spatial_rnea_gravity(q, robot_params)
    use_gravity = 1;
    n = length(q);
    g = spatial_rnea(q, zeros(n,1), zeros(n,1), zeros(n,1), use_gravity, zeros(6,1), robot_params);
end
