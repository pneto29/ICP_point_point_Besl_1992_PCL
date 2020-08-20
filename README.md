# IterativeClosestPoint provides a base implementation of the Iterative Closest Point algorithm.

The transformation is estimated based on Singular Value Decomposition (SVD).

The algorithm has several termination criteria:

    Number of iterations has reached the maximum user imposed number of iterations (via setMaximumIterations)
    The epsilon (difference) between the previous transformation and the current estimated transformation is smaller than an user imposed value (via setTransformationEpsilon)
    The sum of Euclidean squared errors is smaller than a user defined threshold (via setEuclideanFitnessEpsilon)
