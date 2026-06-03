package com.graphhopper.trailmap.matching;

import static java.lang.Math.PI;
import static java.lang.Math.exp;
import static java.lang.Math.log;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

/**
 * Trailmap fork of {@link com.graphhopper.matching.HmmProbabilities}.
 *
 * <p>This is a faithful copy of GraphHopper's HMM probability model (Newson &amp; Krumm
 * 2009). In this revision (Phases 0/1/5/6) the formulas are <b>identical to upstream</b> —
 * the class exists as the home for the future profile-aware emission/transition work
 * (M2a/b/c) so those experiments never have to touch GH core.
 *
 * <p>The log-distribution helpers are inlined (rather than calling {@code Distributions}),
 * because {@code Distributions.logExponentialDistribution} is package-private to
 * {@code com.graphhopper.matching}. The formulas match upstream exactly.
 */
public class TrailmapHmmProbabilities {

    private final double sigma;
    private final double beta;

    /**
     * @param sigma standard deviation of the normal distribution [m] used for modeling the
     *              GPS error
     * @param beta  beta parameter of the exponential distribution used for modeling
     *              transition probabilities
     */
    public TrailmapHmmProbabilities(double sigma, double beta) {
        this.sigma = sigma;
        this.beta = beta;
    }

    /**
     * Returns the logarithmic emission probability density.
     *
     * @param distance Absolute distance [m] between GPS measurement and map matching
     *                 candidate.
     */
    public double emissionLogProbability(double distance) {
        return logNormalDistribution(sigma, distance);
    }

    /**
     * Emission log-probability density using an EXPLICIT per-observation sigma (adaptive-σ path),
     * instead of the instance's global sigma. Additive: the global {@link #emissionLogProbability(double)}
     * is unchanged. Note for Viterbi: the constant term {@code ln(1/(√2π·σ))} is identical across all
     * candidates AT a given observation, so it cancels in the argmin; only the {@code -0.5·(d/σ)²}
     * term discriminates — a per-obs σ is therefore well-defined.
     */
    public double emissionLogProbability(double distance, double sigmaOverride) {
        return logNormalDistribution(sigmaOverride, distance);
    }

    /**
     * Returns the logarithmic transition probability density for the given transition
     * parameters.
     *
     * @param routeLength    Length of the shortest route [m] between two consecutive map
     *                       matching candidates.
     * @param linearDistance Linear distance [m] between two consecutive GPS measurements.
     */
    public double transitionLogProbability(double routeLength, double linearDistance) {
        // Transition metric taken from Newson & Krumm.
        double transitionMetric = Math.abs(linearDistance - routeLength);
        return logExponentialDistribution(beta, transitionMetric);
    }

    /** Inlined copy of {@code Distributions.logNormalDistribution} (upstream formula). */
    private static double logNormalDistribution(double sigma, double x) {
        return log(1.0 / (sqrt(2.0 * PI) * sigma)) + (-0.5 * pow(x / sigma, 2));
    }

    /**
     * Inlined copy of {@code Distributions.logExponentialDistribution} (upstream formula).
     *
     * @param beta =1/lambda with lambda being the standard exponential distribution rate parameter
     */
    private static double logExponentialDistribution(double beta, double x) {
        return log(1.0 / beta) - (x / beta);
    }
}
