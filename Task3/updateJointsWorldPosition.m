function [X, T] = updateJointsWorldPosition(robot_struct, theta)
% Compute forward kinematics using the Product of Exponentials (PoE) formula

% INPUT:
%   robot_struct : MATLAB rigidBodyTree
%   theta        : [nJoints x 1] vector of joint angles

% OUTPUT:
%   X : [nJoints x 4] positions of each joint (in homogeneous form)
%   T : {1 x nJoints} cell array of homogeneous transforms of each joint

nJoints = length(theta);
T = cell(1, nJoints);
X = zeros(nJoints, 4);

% Persistent data M_all Slist so that they are only computed once
persistent M_all Slist
if isempty(Slist)
    Slist = zeros(6, nJoints);
    M_all = cell(1, nJoints);
    T_accum = eye(4);

    for i = 1:nJoints
        T_joint = robot_struct.Bodies{i}.Joint.JointToParentTransform;
        T_child = robot_struct.Bodies{i}.Joint.ChildToJointTransform;
        T_accum = T_accum * T_joint * T_child;
        M_all{i} = T_accum;  % home configuration of joint i

        % Screw axis in space frame
        w_local = robot_struct.Bodies{i}.Joint.JointAxis(:);
        R = T_accum(1:3, 1:3);
        q = T_accum(1:3, 4);
        w = R * w_local;
        v = -cross(w, q);
        Slist(:, i) = [w; v];
    end
end


for k = 1:nJoints
    T_k = eye(4);
    for j = 1:k
        T_k = T_k * MatrixExp6(VecToSE3(Slist(:, j) * theta(j)));
    end
    T{k} = T_k * M_all{k};
    X(k, :) = (T{k} * [0; 0; 0; 1])';
end

end


% --------- Helper Functions --------- 

function se3mat = VecToSE3(V)
% Takes a 6-vector (representing a spatial velocity).
% Returns the corresponding 4x4 se(3) matrix

omega = V(1:3);
v = V(4:6);

% VecToso3 function incorporated here:
% Takes a 3x3 skew-symmetric matrix (an element of so(3)).
% Returns the corresponding 3-vector (angular velocity).
omega_hat = [  0       -omega(3)  omega(2);
            omega(3)     0       -omega(1);
           -omega(2)  omega(1)     0];

se3mat = [omega_hat, v; 0, 0, 0, 0];
end


function omg = so3ToVec(so3mat)
% Takes a 3x3 skew-symmetric matrix (an element of so(3)).
% Returns the corresponding 3-vector (angular velocity).

omg = [so3mat(3,2); so3mat(1,3); so3mat(2,1)];
end


function [omghat, theta] = AxisAng3(omgtheta)
% Takes A 3-vector of exponential coordinates for rotation.
% Returns the unit rotation axis omghat and the corresponding rotation 

theta = norm(omgtheta);
if theta < 1e-6
    omghat = zeros(3,1);
else
    omghat = omgtheta / theta;
end
end


function res = NearZero(z)
% Checks if the scalar is small enough to be neglected.

res = abs(z) < 1e-6;
end


function R = MatrixExp3(so3mat)
% Takes a 3x3 so(3) representation of exponential coordinates.
% Returns R in SO(3) that is achieved by rotating about omghat by theta 

omgtheta = so3ToVec(so3mat);
if NearZero(norm(omgtheta))
    R = eye(3);
else
    [omghat, theta] = AxisAng3(omgtheta);
    omgmat = so3mat / theta;
    R = eye(3) + sin(theta)*omgmat + (1 - cos(theta))*(omgmat^2);
end
end


function T = MatrixExp6(se3mat)
% Takes a se(3) representation of exponential coordinates.
% Returns a T matrix in SE(3) that is achieved by traveling along/about the 
% screw axis S for a distance theta from an initial configuration T = I.

omgtheta = so3ToVec(se3mat(1:3, 1:3));
if NearZero(norm(omgtheta))
    T = [eye(3), se3mat(1:3, 4); 0 0 0 1];
else
    [omghat, theta] = AxisAng3(omgtheta);
    omgmat = se3mat(1:3, 1:3) / theta;
    R = MatrixExp3(se3mat(1:3, 1:3));
    p = (eye(3)*theta + (1 - cos(theta))*omgmat + (theta - sin(theta))*omgmat^2) * se3mat(1:3,4) / theta;
    T = [R, p; 0 0 0 1];
end
end
