% dtheta: estimated gradient
% em: 1 by nJoints cell, each cell is nSamples by nDiscretize matrix
function dtheta = stompDTheta(trajProb, em)

nJoints = length(em);
nDiscretize = size(trajProb, 2);
% variable declaration
dtheta = zeros(nJoints, nDiscretize);

%% TODO: iterate over all joints to compute dtheta: (complete your code according to the STOMP algorithm) 
for j = 1:nJoints
    %For each join, compute the probability-weighted sum of deviation
    % This is the gradient estimator: dtheta = sum(P * epsilon)
    for t = 1:nDiscretize
        dtheta(j,t) = sum(trajProb(:,t) .* em{j}(:,t));
    end
end

end