/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GncOptimizer.h
 * @brief   The GncOptimizer class
 * @author  Jingnan Shi
 * @author  Luca Carlone
 * @author  Frank Dellaert
 *
 * Implementation of the paper: Yang, Antonante, Tzoumas, Carlone, "Graduated Non-Convexity for Robust Spatial Perception:
 * From Non-Minimal Solvers to Global Outlier Rejection", ICRA/RAL, 2020. (arxiv version: https://arxiv.org/pdf/1909.08605.pdf)
 *
 * See also:
 * Antonante, Tzoumas, Yang, Carlone, "Outlier-Robust Estimation: Hardness, Minimally-Tuned Algorithms, and Applications",
 * arxiv: https://arxiv.org/pdf/2007.15109.pdf, 2020.
 */

#pragma once
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <cmath>
#include <vector>
#include <limits>
#include <gtsam/nonlinear/GncParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
// #include <boost/math/distributions/chi_squared.hpp>

namespace gtsam {

    // 计算伽马函数的对数
    static double LogGamma(double x) {
        // 这里使用了 Lanczos 近似公式
        double coef[] = {76.18009172947146, -86.50532032941677,
                         24.01409824083091, -1.231739572450155,
                         0.1208650973866179e-2, -0.5395239384953e-5};
        double ser = 1.000000000190015;
        double xx, y, tmp;
        y = xx = x;
        tmp = xx + 5.5;
        tmp -= (xx + 0.5) * log(tmp);
        for (int j = 0; j < 6; j++) {
            ser += coef[j] / (++y);
        }
        return -tmp + log(2.5066282746310005 * ser / xx);
    }
    // 不完全伽马函数的上部分（正则化）
    static double GammaP(double a, double x);
    static double GammaQ(double a, double x) {
        if (x <= 0 || a <= 0) {
            return 1.0;
        }

        if (x < a + 1) {
            return 1.0 - GammaP(a, x);
        }

        double loggamma_a = LogGamma(a);
        double y = 1.0 - a;
        double z = x + y + 1.0;
        double c = 0.0;
        double pkm2 = 1.0;
        double qkm2 = x;
        double pkm1 = x + 1.0;
        double qkm1 = z * x;
        double ans = pkm1 / qkm1;

        double epsilon = std::numeric_limits<double>::epsilon();
        for (int i = 0; i < 1000; i++) {
            c++;
            y++;
            z += 2.0;
            double yc = y * c;
            double pk = pkm1 * z - pkm2 * yc;
            double qk = qkm1 * z - qkm2 * yc;
            if (qk != 0) {
                double r = pk / qk;
                double t = fabs((ans - r) / r);
                ans = r;
                if (t < epsilon) {
                    return exp(-x + a * log(x) - loggamma_a) * ans;
                }
            }
            pkm2 = pkm1;
            pkm1 = pk;
            qkm2 = qkm1;
            qkm1 = qk;
            if (fabs(pk) > std::numeric_limits<double>::max()) {
                pkm2 *= std::numeric_limits<double>::min();
                pkm1 *= std::numeric_limits<double>::min();
                qkm2 *= std::numeric_limits<double>::min();
                qkm1 *= std::numeric_limits<double>::min();
            }
        }

        return exp(-x + a * log(x) - loggamma_a) * ans;
    }
    // 不完全伽马函数的下部分（正则化）
    static double GammaP(double a, double x) {
        if (x <= 0 || a <= 0) {
            return 0.0;
        }

        if (x > a + 1) {
            return 1.0 - GammaQ(a, x);
        }

        double loggamma_a = LogGamma(a);
        double ap = a;
        double sum = 1.0 / a;
        double del = sum;
        double b = x + 1.0 - a;
        double c = 1.0 / std::numeric_limits<double>::min();
        double d = 1.0 / b;
        double h = d;

        int i;
        for (i = 1; i <= 100; i++) {
            double an = -i * (i - a);
            b += 2.0;
            d = an * d + b;
            if (fabs(d) < std::numeric_limits<double>::min()) {
                d = std::numeric_limits<double>::min();
            }
            c = b + an / c;
            if (fabs(c) < std::numeric_limits<double>::min()) {
                c = std::numeric_limits<double>::min();
            }
            d = 1.0 / d;
            del = d * c;
            h *= del;
            if (fabs(del - 1.0) <= std::numeric_limits<double>::epsilon()) {
                break;
            }
        }

        return exp(-x + a * log(x) - loggamma_a) * h;
    }
    // 卡方分布的累积分布函数
    double chi2cdf(double x, size_t dofs) {
        return GammaP(dofs / 2.0, x / 2.0);
    }

    /*
 * Quantile of chi-squared distribution with given degrees of freedom at probability alpha.
 * Equivalent to chi2inv in Matlab.
 */
static double Chi2inv(const double alpha, const size_t dofs) {
    // Create a chi-squared distribution with the given degrees of freedom
    std::chi_squared_distribution<double> chi2(dofs);

    // Set up the parameters for the binary search
    double lower_bound = 0;
    double upper_bound = std::numeric_limits<double>::max(); // A reasonable starting point for the upper bound
    double quantile = dofs; // Start with an initial guess
    double precision = 1e-6; // Set the precision to determine when to stop the search

    // Perform the binary search
    while (upper_bound - lower_bound > precision) {
        double mid = (lower_bound + upper_bound) / 2;
        double p = chi2cdf(mid, dofs);

        if (p < alpha) {
            lower_bound = mid;
        } else {
            upper_bound = mid;
        }
    }

    quantile = (lower_bound + upper_bound) / 2;
    return quantile;
}

/* ************************************************************************* */
template<class GncParameters>
class GncOptimizer {
 public:
  /// For each parameter, specify the corresponding optimizer: e.g., GaussNewtonParams -> GaussNewtonOptimizer.
  typedef typename GncParameters::OptimizerType BaseOptimizer;

 private:
  NonlinearFactorGraph nfg_; ///< Original factor graph to be solved by GNC.
  Values state_; ///< Initial values to be used at each iteration by GNC.
  GncParameters params_; ///< GNC parameters.
  Vector weights_;  ///< Weights associated to each factor in GNC (this could be a local variable in optimize, but it is useful to make it accessible from outside).
  Vector barcSq_;  ///< Inlier thresholds. A factor is considered an inlier if factor.error() < barcSq_[i] (where i is the position of the factor in the factor graph. Note that factor.error() whitens by the covariance.

 public:
  /// Constructor.
  GncOptimizer(const NonlinearFactorGraph& graph, const Values& initialValues,
               const GncParameters& params = GncParameters())
      : state_(initialValues),
        params_(params) {

    // make sure all noiseModels are Gaussian or convert to Gaussian
    nfg_.resize(graph.size());
    for (size_t i = 0; i < graph.size(); i++) {
      if (graph[i]) {
        NoiseModelFactor::shared_ptr factor = std::dynamic_pointer_cast<
            NoiseModelFactor>(graph[i]);
        auto robust = std::dynamic_pointer_cast<
            noiseModel::Robust>(factor->noiseModel());
        // if the factor has a robust loss, we remove the robust loss
        nfg_[i] = robust ? factor-> cloneWithNewNoiseModel(robust->noise()) : factor;
      }
    }

    // check that known inliers and outliers make sense:
    std::vector<size_t> inconsistentlySpecifiedWeights; // measurements the user has incorrectly specified
    // to be BOTH known inliers and known outliers
    std::set_intersection(params.knownInliers.begin(),params.knownInliers.end(),
                        params.knownOutliers.begin(),params.knownOutliers.end(),
                        std::inserter(inconsistentlySpecifiedWeights, inconsistentlySpecifiedWeights.begin()));
    if(inconsistentlySpecifiedWeights.size() > 0){ // if we have inconsistently specified weights, we throw an exception
      params.print("params\n");
      throw std::runtime_error("GncOptimizer::constructor: the user has selected one or more measurements"
          " to be BOTH a known inlier and a known outlier.");
    }
    // check that known inliers are in the graph
    for (size_t i = 0; i < params.knownInliers.size(); i++){
      if( params.knownInliers[i] > nfg_.size()-1 ){ // outside graph
        throw std::runtime_error("GncOptimizer::constructor: the user has selected one or more measurements"
                  "that are not in the factor graph to be known inliers.");
      }
    }
    // check that known outliers are in the graph
    for (size_t i = 0; i < params.knownOutliers.size(); i++){
      if( params.knownOutliers[i] > nfg_.size()-1 ){ // outside graph
        throw std::runtime_error("GncOptimizer::constructor: the user has selected one or more measurements"
                  "that are not in the factor graph to be known outliers.");
      }
    }
    // initialize weights (if we don't have prior knowledge of inliers/outliers
    // the weights are all initialized to 1.
    weights_ = initializeWeightsFromKnownInliersAndOutliers();

    // set default barcSq_ (inlier threshold)
    double alpha = 0.99; // with this (default) probability, inlier residuals are smaller than barcSq_
    setInlierCostThresholdsAtProbability(alpha);
  }

  /** Set the maximum weighted residual error for an inlier (same for all factors). For a factor in the form f(x) = 0.5 * || r(x) ||^2_Omega,
   * the inlier threshold is the largest value of f(x) for the corresponding measurement to be considered an inlier.
   * In other words, an inlier at x is such that 0.5 * || r(x) ||^2_Omega <= barcSq.
   * Assuming an isotropic measurement covariance sigma^2 * Identity, the cost becomes: 0.5 * 1/sigma^2 || r(x) ||^2 <= barcSq.
   * Hence || r(x) ||^2 <= 2 * barcSq * sigma^2.
   * */
  void setInlierCostThresholds(const double inth) {
    barcSq_ = inth * Vector::Ones(nfg_.size());
  }

  /** Set the maximum weighted residual error for an inlier (one for each factor). For a factor in the form f(x) = 0.5 * || r(x) ||^2_Omega,
   * the inlier threshold is the largest value of f(x) for the corresponding measurement to be considered an inlier.
   * In other words, an inlier at x is such that 0.5 * || r(x) ||^2_Omega <= barcSq.
   * */
  void setInlierCostThresholds(const Vector& inthVec) {
    barcSq_ = inthVec;
  }

  /** Set the maximum weighted residual error threshold by specifying the probability
   * alpha that the inlier residuals are smaller than that threshold
   * */
  void setInlierCostThresholdsAtProbability(const double alpha) {
    barcSq_  = Vector::Ones(nfg_.size()); // initialize
    for (size_t k = 0; k < nfg_.size(); k++) {
      if (nfg_[k]) {
        barcSq_[k] = 0.5 * Chi2inv(alpha, nfg_[k]->dim()); // 0.5 derives from the error definition in gtsam
      }
    }
  }

  /** Set weights for each factor. This is typically not needed, but
   * provides an extra interface for the user to initialize the weightst
   * */
  void setWeights(const Vector w) {
    if (size_t(w.size()) != nfg_.size()) {
      throw std::runtime_error(
          "GncOptimizer::setWeights: the number of specified weights"
          " does not match the size of the factor graph.");
    }
    weights_ = w;
  }

  /// Access a copy of the internal factor graph.
  const NonlinearFactorGraph& getFactors() const { return nfg_; }

  /// Access a copy of the internal values.
  const Values& getState() const { return state_; }

  /// Access a copy of the parameters.
  const GncParameters& getParams() const { return params_;}

  /// Access a copy of the GNC weights.
  const Vector& getWeights() const { return weights_;}

  /// Get the inlier threshold.
  const Vector& getInlierCostThresholds() const {return barcSq_;}

  /// Equals.
  bool equals(const GncOptimizer& other, double tol = 1e-9) const {
    return nfg_.equals(other.getFactors())
        && equal(weights_, other.getWeights())
        && params_.equals(other.getParams())
        && equal(barcSq_, other.getInlierCostThresholds());
  }

  Vector initializeWeightsFromKnownInliersAndOutliers() const{
    Vector weights = Vector::Ones(nfg_.size());
    for (size_t i = 0; i < params_.knownOutliers.size(); i++){
      weights[ params_.knownOutliers[i] ] = 0.0; // known to be outliers
    }
    return weights;
  }

  /// Compute optimal solution using graduated non-convexity.
  Values optimize() {
    NonlinearFactorGraph graph_initial = this->makeWeightedGraph(weights_);
    BaseOptimizer baseOptimizer(
        graph_initial, state_, params_.baseOptimizerParams);
    Values result = baseOptimizer.optimize();
    double mu = initializeMu();
    double prev_cost = graph_initial.error(result);
    double cost = 0.0;  // this will be updated in the main loop

    // handle the degenerate case that corresponds to small
    // maximum residual errors at initialization
    // For GM: if residual error is small, mu -> 0
    // For TLS: if residual error is small, mu -> -1
    int nrUnknownInOrOut = nfg_.size() - ( params_.knownInliers.size() + params_.knownOutliers.size() );
    // ^^ number of measurements that are not known to be inliers or outliers (GNC will need to figure them out)
    if (mu <= 0 || nrUnknownInOrOut == 0) { // no need to even call GNC in this case
      if (mu <= 0 && params_.verbosity >= GncParameters::Verbosity::SUMMARY) {
        std::cout << "GNC Optimizer stopped because maximum residual at "
                  "initialization is small."
                  << std::endl;
      }
      if (nrUnknownInOrOut==0 && params_.verbosity >= GncParameters::Verbosity::SUMMARY) {
        std::cout << "GNC Optimizer stopped because all measurements are already known to be inliers or outliers"
                  << std::endl;
      }
      if (params_.verbosity >= GncParameters::Verbosity::MU) {
        std::cout << "mu: " << mu << std::endl;
      }
      if (params_.verbosity >= GncParameters::Verbosity::VALUES) {
        result.print("result\n");
      }
      return result;
    }

    size_t iter;
    for (iter = 0; iter < params_.maxIterations; iter++) {

      // display info
      if (params_.verbosity >= GncParameters::Verbosity::MU) {
        std::cout << "iter: " << iter << std::endl;
        std::cout << "mu: " << mu << std::endl;
      }
      if (params_.verbosity >= GncParameters::Verbosity::WEIGHTS) {
        std::cout << "weights: " << weights_ << std::endl;
      }
      if (params_.verbosity >= GncParameters::Verbosity::VALUES) {
        result.print("result\n");
      }
      // weights update
      weights_ = calculateWeights(result, mu);

      // variable/values update
      NonlinearFactorGraph graph_iter = this->makeWeightedGraph(weights_);
      BaseOptimizer baseOptimizer_iter(
          graph_iter, state_, params_.baseOptimizerParams);
      result = baseOptimizer_iter.optimize();

      // stopping condition
      cost = graph_iter.error(result);
      if (checkConvergence(mu, weights_, cost, prev_cost)) {
        break;
      }

      // update mu
      mu = updateMu(mu);

      // get ready for next iteration
      prev_cost = cost;

      // display info
      if (params_.verbosity >= GncParameters::Verbosity::VALUES) {
        std::cout << "previous cost: " << prev_cost << std::endl;
        std::cout << "current cost: " << cost << std::endl;
      }
    }
    // display info
    if (params_.verbosity >= GncParameters::Verbosity::SUMMARY) {
      std::cout << "final iterations: " << iter << std::endl;
      std::cout << "final mu: " << mu << std::endl;
      std::cout << "previous cost: " << prev_cost << std::endl;
      std::cout << "current cost: " << cost << std::endl;
    }
    if (params_.verbosity >= GncParameters::Verbosity::WEIGHTS) {
      std::cout << "final weights: " << weights_ << std::endl;
    }
    return result;
  }

  /// Initialize the gnc parameter mu such that loss is approximately convex (remark 5 in GNC paper).
  double initializeMu() const {

    double mu_init = 0.0;
    // initialize mu to the value specified in Remark 5 in GNC paper.
    switch (params_.lossType) {
      case GncLossType::GM:
        /* surrogate cost is convex for large mu. initialize as in remark 5 in GNC paper.
         Since barcSq_ can be different for each factor, we compute the max of the quantity in remark 5 in GNC paper
         */
        for (size_t k = 0; k < nfg_.size(); k++) {
          if (nfg_[k]) {
            mu_init = std::max(mu_init, 2 * nfg_[k]->error(state_) / barcSq_[k]);
          }
        }
        return mu_init;  // initial mu
      case GncLossType::TLS:
        /* surrogate cost is convex for mu close to zero. initialize as in remark 5 in GNC paper.
         degenerate case: 2 * rmax_sq - params_.barcSq < 0 (handled in the main loop)
         according to remark mu = params_.barcSq / (2 * rmax_sq - params_.barcSq) = params_.barcSq/ excessResidual
         however, if the denominator is 0 or negative, we return mu = -1 which leads to termination of the main GNC loop.
         Since barcSq_ can be different for each factor, we look for the minimimum (positive) quantity in remark 5 in GNC paper
         */
        mu_init = std::numeric_limits<double>::infinity();
        for (size_t k = 0; k < nfg_.size(); k++) {
          if (nfg_[k]) {
            double rk = nfg_[k]->error(state_);
            mu_init = (2 * rk - barcSq_[k]) > 0 ? // if positive, update mu, otherwise keep same
                std::min(mu_init, barcSq_[k] / (2 * rk - barcSq_[k]) ) : mu_init;
          }
        }
        if (mu_init >= 0 && mu_init < 1e-6){
          mu_init = 1e-6; // if mu ~ 0 (but positive), that means we have measurements with large errors,
          // i.e., rk > barcSq_[k] and rk very large, hence we threshold to 1e-6 to avoid mu = 0
        }
  
        return mu_init > 0 && !std::isinf(mu_init) ? mu_init : -1; // if mu <= 0 or mu = inf, return -1,
        // which leads to termination of the main gnc loop. In this case, all residuals are already below the threshold
        // and there is no need to robustify (TLS = least squares)
      default:
        throw std::runtime_error(
            "GncOptimizer::initializeMu: called with unknown loss type.");
    }
  }

  /// Update the gnc parameter mu to gradually increase nonconvexity.
  double updateMu(const double mu) const {
    switch (params_.lossType) {
      case GncLossType::GM:
        // reduce mu, but saturate at 1 (original cost is recovered for mu -> 1)
        return std::max(1.0, mu / params_.muStep);
      case GncLossType::TLS:
        // increases mu at each iteration (original cost is recovered for mu -> inf)
        return mu * params_.muStep;
      default:
        throw std::runtime_error(
            "GncOptimizer::updateMu: called with unknown loss type.");
    }
  }

  /// Check if we have reached the value of mu for which the surrogate loss matches the original loss.
  bool checkMuConvergence(const double mu) const {
    bool muConverged = false;
    switch (params_.lossType) {
      case GncLossType::GM:
        muConverged = std::fabs(mu - 1.0) < 1e-9;  // mu=1 recovers the original GM function
        break;
      case GncLossType::TLS:
        muConverged = false;  // for TLS there is no stopping condition on mu (it must tend to infinity)
        break;
      default:
        throw std::runtime_error(
            "GncOptimizer::checkMuConvergence: called with unknown loss type.");
    }
    if (muConverged && params_.verbosity >= GncParameters::Verbosity::SUMMARY)
      std::cout << "muConverged = true " << std::endl;
    return muConverged;
  }

  /// Check convergence of relative cost differences.
  bool checkCostConvergence(const double cost, const double prev_cost) const {
    bool costConverged = std::fabs(cost - prev_cost) / std::max(prev_cost, 1e-7)
        < params_.relativeCostTol;
    if (costConverged && params_.verbosity >= GncParameters::Verbosity::SUMMARY){
      std::cout << "checkCostConvergence = true (prev. cost = " << prev_cost
                << ", curr. cost = " << cost << ")" << std::endl;
    }
    return costConverged;
  }

  /// Check convergence of weights to binary values.
  bool checkWeightsConvergence(const Vector& weights) const {
    bool weightsConverged = false;
    switch (params_.lossType) {
      case GncLossType::GM:
        weightsConverged = false;  // for GM, there is no clear binary convergence for the weights
        break;
      case GncLossType::TLS:
        weightsConverged = true;
        for (int i = 0; i < weights.size(); i++) {
          if (std::fabs(weights[i] - std::round(weights[i]))
              > params_.weightsTol) {
            weightsConverged = false;
            break;
          }
        }
        break;
      default:
        throw std::runtime_error(
            "GncOptimizer::checkWeightsConvergence: called with unknown loss type.");
    }
    if (weightsConverged
        && params_.verbosity >= GncParameters::Verbosity::SUMMARY)
      std::cout << "weightsConverged = true " << std::endl;
    return weightsConverged;
  }

  /// Check for convergence between consecutive GNC iterations.
  bool checkConvergence(const double mu, const Vector& weights,
                        const double cost, const double prev_cost) const {
    return checkCostConvergence(cost, prev_cost)
        || checkWeightsConvergence(weights) || checkMuConvergence(mu);
  }

  /// Create a graph where each factor is weighted by the gnc weights.
  NonlinearFactorGraph makeWeightedGraph(const Vector& weights) const {
    // make sure all noiseModels are Gaussian or convert to Gaussian
    NonlinearFactorGraph newGraph;
    newGraph.resize(nfg_.size());
    for (size_t i = 0; i < nfg_.size(); i++) {
      if (nfg_[i]) {
        auto factor = std::dynamic_pointer_cast<
            NoiseModelFactor>(nfg_[i]);
        auto noiseModel =
            std::dynamic_pointer_cast<noiseModel::Gaussian>(
                factor->noiseModel());
        if (noiseModel) {
          Matrix newInfo = weights[i] * noiseModel->information();
          auto newNoiseModel = noiseModel::Gaussian::Information(newInfo);
          newGraph[i] = factor->cloneWithNewNoiseModel(newNoiseModel);
        } else {
          throw std::runtime_error(
              "GncOptimizer::makeWeightedGraph: unexpected non-Gaussian noise model.");
        }
      }
    }
    return newGraph;
  }

  /// Calculate gnc weights.
  Vector calculateWeights(const Values& currentEstimate, const double mu) {
    Vector weights = initializeWeightsFromKnownInliersAndOutliers();

    // do not update the weights that the user has decided are known inliers
    std::vector<size_t> allWeights;
    for (size_t k = 0; k < nfg_.size(); k++) {
      allWeights.push_back(k);
    }
    std::vector<size_t> knownWeights;
    std::set_union(params_.knownInliers.begin(), params_.knownInliers.end(),
                   params_.knownOutliers.begin(), params_.knownOutliers.end(),
                   std::inserter(knownWeights, knownWeights.begin()));

    std::vector<size_t> unknownWeights;
    std::set_difference(allWeights.begin(), allWeights.end(),
                        knownWeights.begin(), knownWeights.end(),
                        std::inserter(unknownWeights, unknownWeights.begin()));

    // update weights of known inlier/outlier measurements
    switch (params_.lossType) {
      case GncLossType::GM: {  // use eq (12) in GNC paper
        for (size_t k : unknownWeights) {
          if (nfg_[k]) {
            double u2_k = nfg_[k]->error(currentEstimate);  // squared (and whitened) residual
            weights[k] = std::pow(
                (mu * barcSq_[k]) / (u2_k + mu * barcSq_[k]), 2);
          }
        }
        return weights;
      }
      case GncLossType::TLS: {  // use eq (14) in GNC paper
        for (size_t k : unknownWeights) {
          if (nfg_[k]) {
            double u2_k = nfg_[k]->error(currentEstimate);  // squared (and whitened) residual
            double upperbound = (mu + 1) / mu * barcSq_[k];
            double lowerbound = mu / (mu + 1) * barcSq_[k];
            weights[k] = std::sqrt(barcSq_[k] * mu * (mu + 1) / u2_k) - mu;
            if (u2_k >= upperbound || weights[k] < 0) {
              weights[k] = 0;
            } else if (u2_k <= lowerbound || weights[k] > 1) {
              weights[k] = 1;
            }
          }
        }
        return weights;
      }
      default:
        throw std::runtime_error(
            "GncOptimizer::calculateWeights: called with unknown loss type.");
    }
  }
};

}
