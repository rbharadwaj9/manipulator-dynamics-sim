function u = spatial_rnea(q, qd, qd_a, qdd, use_gravity, Ftip, robot_params)
% Takes q: n-vector of joint variables,
%       qd: n-vector of joint rates,
%       qdd: n-vector of joint accelerations,
%       g: Gravity vector g,
%       Ftip: Spatial force applied by the end-effector expressed in frame 
%             {n+1},
%       M: List of link frames {i} relative to {i-1} at the home 
%              position,
%       robot_params: Dictionary of robot parameters
%       G: Spatial inertia matrices Gi of the links,
%       S: Screw axes Si of the joints in a space frame, in the format
%              of a matrix with the screw axes as the columns.
%
% Returns u: The n-vector of required joint forces/torques.
% This function uses forward-backward Newton-Euler iterations to solve the 
% equation:
% u = H(q)*qdd + C(q, qd)*qd_a + G(q) + J(q)*Ftip

% use interval arithmetic
use_interval = robot_params.use_interval;

g = use_gravity * robot_params.gravity;
M = robot_params.M;
S = robot_params.S;
G = robot_params.G;

n = size(q, 1);
Mi = eye(4);
Ai = zeros(6, n);
AdTi = zeros(6, 6, n + 1);
Vi = zeros(6, n + 1);
Vai = zeros(6, n + 1); % auxillary spatial velocity

Vdi = zeros(6, n + 1);
Vdi(4: 6, 1) = -g;

AdTi(:, :, n + 1) = Adjoint(TransInv(M(:, :, n + 1)));

u = zeros(n, 1);

if use_interval
    u = interval(u);
end

%% forward recursion
for i=1: n    
    % frame-to-frame transform
    Mi = Mi * M(:, :, i);
    
    % screw axis in body frame
    Ai(:, i) = Adjoint(TransInv(Mi)) * S(:, i);    
    
    % adjoint of transformation of frame {i-1} to {i}
    AdTi(:, :, i) = Adjoint(MatrixExp6(VecTose3(Ai(:, i) ...
                    * -q(i))) * TransInv(M(:, :, i)));  
    
    % spatial velocty
    Vi(:, i + 1) = AdTi(:, :, i) * Vi(:, i) + Ai(:, i) * qd(i);
    
    % auxillary spatial velocity
    Vai(:, i + 1) = AdTi(:, :, i) * Vai(:, i) + Ai(:, i) * qd_a(i);
      
    % spatial acceleration
    Vdi(:, i + 1) = AdTi(:, :, i) * Vdi(:, i) ...
                + Ai(:, i) * qdd(i) ...
                + ad(Vi(:, i + 1)) * Ai(:, i) * qd_a(i);
                     
end

%% backward recursion
Fi = Ftip;
for i = n: -1: 1
    % enforce skew-symmetry
    w = crossop(Vai(1:3, i+1));
    I = G{i}(1:3,1:3);
    m = G{i}(4, 4);
    
    Ci = [w * I + I *w 0*w; 0*w m*w];
    
    % spatial force
    Fi = AdTi(:, :, i + 1)' * Fi ...
       + G{i} * Vdi(:, i + 1) ...
       + Ci * Vi(:, i + 1);
     
    % torque
    u(i) = Fi' * Ai(:, i);
end
end


function W = crossop(w)

    W = [0   -w(3)   w(2);
        w(3)   0    -w(1);
       -w(2)  w(1)     0];

end