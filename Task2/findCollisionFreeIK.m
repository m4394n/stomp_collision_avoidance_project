function qSol = findCollisionFreeIK(robot, world, endEffector, targetTform)
    ik = inverseKinematics('RigidBodyTree', robot);
    ik.SolverParameters.AllowRandomRestart = true;
    ik.SolverParameters.MaxIterations = 200;

    weights = [1 1 1 1 1 1];
    numSeeds = 200;
    solutionFound = false;

    % Create multiple random seeds
    seeds = repmat(homeConfiguration(robot), 1, numSeeds);
    for i = 2:numSeeds
        seeds(:, i) = randomConfiguration(robot);
    end

    for i = 1:numSeeds
        qCandidate = ik(endEffector, targetTform, weights, seeds(:, i));
        qCandidate = wrapToPi(qCandidate);

        [inCollision, ~] = checkCollision(robot, qCandidate, world, ...
            "IgnoreSelfCollision", "on", "Exhaustive", "on");

        if ~inCollision
            qSol = qCandidate;
            %fprintf('✅ Collision-free solution found (seed %d)\n', i);
            solutionFound = true;
            return;
        end
    end

    if ~solutionFound
        error('❌ No collision-free IK found for this exact end-effector pose.');
    end
end
