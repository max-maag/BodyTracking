#pragma once

#include <vector>
#include <functional>
#include <Kore/Math/Vector.h>
#include <Kore/Math/Quaternion.h>

class IKEvaluator
{
private:
	struct RunStats {
		unsigned int numIterations;
		float durationMs;
		float durationAveragePerIterationMs;
		float errorPositionMm;
		float errorRotation;
		bool hasReachedTarget;
		bool isStuck;
	};

	std::vector<RunStats> data;

public:
	struct AverageAndDeviation {
		float average = 0;
		float deviation = 0;
	};

	struct RunStatsAverage {
		AverageAndDeviation numIterations;
		AverageAndDeviation durationsMs;
		AverageAndDeviation durationAveragePerIterationMs;
		AverageAndDeviation errorPositionMm;
		AverageAndDeviation errorRotation;
		float targetReachedPercent = 0;
		float stuckPercent = 0;
	};

	class RunStatsBuilder {
		friend class IKEvaluator;

	private:
		IKEvaluator& evaluator;
		double timePointRunStart;
		double timePointIterationStart;
		double sumDurationIterations;
		unsigned int numIterations;
		
		RunStatsBuilder(IKEvaluator& evaluator);

	public:
		void beginIteration();
		void endIteration();
		void endRun(float errorPosition, float errorOrientation, bool hasReachedTarget, bool isStuck);
	};

	IKEvaluator(size_t initialReserve = 20000);

	RunStatsBuilder beginRun();
	void clear();

	RunStatsAverage getAverage() const;
};

