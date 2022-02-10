#include "IKEvaluator.h"

#include <Kore/System.h>
#include "RotationUtility.h"

IKEvaluator::IKEvaluator(size_t initialRunReserve) {
	data.reserve(initialRunReserve);
}


IKEvaluator::RunStatsBuilder IKEvaluator::beginRun() {
	return RunStatsBuilder(*this);
}

IKEvaluator::RunStatsBuilder::RunStatsBuilder(IKEvaluator& evaluator):
	evaluator(evaluator),
	timePointRunStart(Kore::System::time()),
	sumDurationIterations(0),
	numIterations(0) {
}

void IKEvaluator::RunStatsBuilder::beginIteration() {
	timePointIterationStart = Kore::System::time();
}

void IKEvaluator::RunStatsBuilder::endIteration() {
	sumDurationIterations += Kore::System::time() - timePointIterationStart;
	numIterations++;
}

void IKEvaluator::RunStatsBuilder::endRun(float errorPosition, float errorOrientation, bool hasReachedTarget, bool isStuck) {
	double timePointEndRun = Kore::System::time();

	evaluator.data.push_back({
		numIterations,
		(float)(timePointEndRun - timePointRunStart),
		(float)(sumDurationIterations / numIterations),
		errorPosition * 1000, // m -> mm
		errorOrientation,
		hasReachedTarget,
		isStuck
	});
}

void IKEvaluator::clear() {
	data.clear();
}

IKEvaluator::RunStatsAverage IKEvaluator::getAverage() const {
	RunStatsAverage result;

	// Calculate averages by first summing all respective data points and then dividing them by the amount of data points.
	for (RunStats stats : data) {
		result.numIterations.average += stats.numIterations;
		result.durationsMs.average += stats.durationMs;
		result.durationAveragePerIterationMs.average += stats.durationAveragePerIterationMs;
		result.errorPositionMm.average += stats.errorPositionMm;
		result.errorRotation.average += stats.errorRotation;
		result.targetReachedPercent += 1;
		result.stuckPercent += 1;
	}

	const size_t numDataPoints = data.size();

	result.numIterations.average /= numDataPoints;
	result.durationsMs.average /= numDataPoints;
	result.durationAveragePerIterationMs.average /= numDataPoints;
	result.errorPositionMm.average /= numDataPoints;
	result.errorRotation.average /= numDataPoints;
	result.targetReachedPercent /= numDataPoints * 100.f;
	result.stuckPercent /= numDataPoints * 100.f;

	// calculate deviations

	for (RunStats stats : data) {
		result.numIterations.deviation += Kore::pow(stats.numIterations - result.numIterations.average, 2);
		result.durationsMs.deviation += Kore::pow(stats.durationMs - result.durationsMs.average, 2);
		result.durationAveragePerIterationMs.deviation += Kore::pow(stats.durationAveragePerIterationMs - result.durationAveragePerIterationMs.average, 2);
		result.errorPositionMm.deviation += Kore::pow(stats.errorPositionMm - result.errorPositionMm.average, 2);
		result.errorRotation.deviation += Kore::pow(stats.errorRotation - result.errorRotation.average, 2);
	}

	result.numIterations.deviation = Kore::sqrt(result.numIterations.deviation / numDataPoints);
	result.durationsMs.deviation = Kore::sqrt(result.durationsMs.deviation/ numDataPoints);
	result.durationAveragePerIterationMs.deviation = Kore::sqrt(result.durationAveragePerIterationMs.deviation / numDataPoints);
	result.errorPositionMm.deviation = Kore::sqrt(result.errorPositionMm.deviation / numDataPoints);
	result.errorRotation.deviation = Kore::sqrt(result.errorRotation.deviation / numDataPoints);

	return result;
}