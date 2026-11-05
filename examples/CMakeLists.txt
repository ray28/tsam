set (excluded_examples
    elaboratePoint2KalmanFilter.cpp
)

gtsamAddExamplesGlob("*.cpp" "${excluded_examples}" "gtsam")
