#include "pch.h"
#include "Logger.h"

#include <Kore/Log.h>

#include <iostream>
#include <string>
#include <ctime>

extern float lambda[];
extern float errorMaxPos[];
extern float errorMaxRot[];
extern float maxIterations[];

namespace {
	bool initHmmAnalysisData = false;
}

Logger::Logger() {
}

Logger::~Logger() {
	logDataReader.close();
	logDataWriter.close();
	hmmWriter.close();
	hmmAnalysisWriter.close();
}

void Logger::startLogger(const char* filename) {
	time_t t = time(0);   // Get time now
	
	char logFileName[50];
	sprintf(logFileName, "%s_%li.csv", filename, t);
	
	logDataWriter.open(logFileName, std::ios::app); // Append to the end
	
	// Append header
	logDataWriter << "tag rawPosX rawPosY rawPosZ rawRotX rawRotY rawRotZ rawRotW scale\n";
	logDataWriter.flush();
	
	log(Kore::Info, "Start logging");
}

void Logger::endLogger() {
	logDataWriter.close();
	
	log(Kore::Info, "Stop logging");
}

void Logger::saveData(const char* tag, Kore::vec3 rawPos, Kore::Quaternion rawRot, float scale) {
	// Save position and rotation
	logDataWriter << tag << " " << rawPos.x() << " " << rawPos.y() << " " << rawPos.z() << " " << rawRot.x << " " << rawRot.y << " " << rawRot.z << " " << rawRot.w << " " << scale << "\n";
	logDataWriter.flush();
}

void Logger::startHMMLogger(const char* filename, int num) {
	char logFileName[50];
	sprintf(logFileName, "%s_%i.csv", filename, num);
	
	hmmWriter.open(logFileName, std::ios::out);
	
	// Append header
	char hmmHeader[] = "tag time posX posY posZ rotX rotY rotZ rotW\n";
	hmmWriter << hmmHeader;
	hmmWriter.flush();
	
	log(Kore::Info, "Start logging data for HMM");
}

void Logger::endHMMLogger() {
	hmmWriter.flush();
	hmmWriter.close();
	
	log(Kore::Info, "Stop logging data for HMM");
}

void Logger::saveHMMData(const char* tag, float lastTime, Kore::vec3 pos, Kore::Quaternion rot) {
	// Save position
	hmmWriter << tag << " " << lastTime << " "  << pos.x() << " " << pos.y() << " " << pos.z() << " " << rot.x << " " << rot.y << " " << rot.z << " " << rot.w << "\n";
	hmmWriter.flush();
}

void Logger::analyseHMM(const char* hmmName, double probability, bool newLine) {
	if (!initHmmAnalysisData) {
		char hmmAnalysisPath[100];
		strcat(hmmAnalysisPath, hmmName);
		strcat(hmmAnalysisPath, "_analysis.txt");
		hmmAnalysisWriter.open(hmmAnalysisPath, std::ios::out | std::ios::app);
		initHmmAnalysisData = true;
	}
	
	if (newLine) hmmAnalysisWriter << "\n";
	else hmmAnalysisWriter << probability << " ";
	
	hmmAnalysisWriter.flush();
}

void Logger::saveEvaluationData(std::string filename, const IKSolver& solver, IKEvaluator::RunStatsAverage ikStatsTotal, std::array<IKEvaluator::RunStatsAverage, numEndEffectors> ikStatsPerEndEffector) {
	
	// Save settings
	char evaluationDataPath[100];
	sprintf(evaluationDataPath, "eval/evaluationData_IK_%s_%s", solver.getName().data(), filename.c_str());
	
	if (!evaluationDataOutputFile.is_open()) {
		evaluationDataOutputFile.open(evaluationDataPath, std::ios::app);
		
		// Append to the end
		evaluationDataOutputFile << "IKMode;File;Lambda;ErrorMaxPos;ErrorMaxRot;IterationsMax;";
		evaluationDataOutputFile << "MeanIterations;StdIterations;";
		evaluationDataOutputFile << "MeanPosError[mm];StdPosError[mm];";
		evaluationDataOutputFile << "MeanRotError[deg];StdRotError[deg];";
		evaluationDataOutputFile << "MeanTimePerIteration[ms];StdTimePerIteration[ms];";
		evaluationDataOutputFile << "MeanTime[ms];StdTime[ms];";
		evaluationDataOutputFile << "Reached[%];Stucked[%];";
		evaluationDataOutputFile << "MeanHeadPosError;StdHeadPosError;MeanHeadRotError;StdHeadRotError;";
		evaluationDataOutputFile << "MeanHipPosError;StdHipPosError;MeanHipRotError;StdHipRotError;";
		evaluationDataOutputFile << "MeanLeftHandPosError;StdLeftHandPosError;MeanLeftHandRotError;StdLeftHandRotError;";
		evaluationDataOutputFile << "MeanLeftForeArmPosError;StdLeftForeArmPosError;MeanLeftForeArmRotError;StdLeftForeArmRotError;";
		evaluationDataOutputFile << "MeanRightHandPosError;StdRightHandPosError;MeanRightHandRotError;StdRightHandRotError;";
		evaluationDataOutputFile << "MeanRightForeArmPosError;StdRightForeArmPosError;MeanRightForeArmRotError;StdRightForeArmRotError;";
		evaluationDataOutputFile << "MeanLeftFootPosError;StdLeftFootPosError;MeanLeftFootRotError;StdLeftFootRotError;";
		evaluationDataOutputFile << "MeanRightFootPosError;StdRightFootPosError;MeanRightFootRotError;StdRightFootRotError;";
		evaluationDataOutputFile << "MeanLeftKneePosError;StdLeftKneePosError;MeanLeftKneeRotError;StdLeftKneeRotError;";
		evaluationDataOutputFile << "MeanRightKneePosError;StdRightKneePosError;MeanRightKneeRotError;StdRightKneeRotError\n";
	}
	
	// Save settings
	log(
		Kore::Info,
		"%s \t IK: %s \t lambda: ? \t errorMaxPos: %f \t errorMaxRot: %f \t maxIterations: %f",
		filename,
		solver.getName(),
		solver.getThresholdTargetReachedPosition(),
		solver.getThresholdTargetReachedRotation(),
		solver.getNumIterationsMax()
	);

	evaluationDataOutputFile << solver.getName()  << ";" << filename << ";" << "nan" << ";" << solver.getThresholdTargetReachedPosition() << ";" << solver.getThresholdTargetReachedRotation() << ";" << solver.getNumIterationsMax() << ";";

	// Save mean and std for iterations
	evaluationDataOutputFile << ikStatsTotal.numIterations.average << ";" << ikStatsTotal.numIterations.deviation << ";";

	// Save mean and std for pos and rot error
	evaluationDataOutputFile << ikStatsTotal.errorPositionMm.average << ";" << ikStatsTotal.errorPositionMm.deviation << ";";
	evaluationDataOutputFile << ikStatsTotal.errorRotation.average << ";" << ikStatsTotal.errorRotation.deviation << ";";
	
	// Save time
	evaluationDataOutputFile << ikStatsTotal.durationAveragePerIterationMs.average << ";" << ikStatsTotal.durationAveragePerIterationMs.deviation << ";";
	evaluationDataOutputFile << ikStatsTotal.durationsMs.average << ";" << ikStatsTotal.durationsMs.average << ";";

	// Reached & stucked
	evaluationDataOutputFile << ikStatsTotal.targetReachedPercent << ";" << ikStatsTotal.stuckPercent << ";";
	
	// Save results for each individual bone

	
	for (size_t idxEndEffector = 0; idxEndEffector < numEndEffectors; idxEndEffector++) {
		IKEvaluator::RunStatsAverage& ikEE = ikStatsPerEndEffector[idxEndEffector];

		evaluationDataOutputFile << ikEE.errorPositionMm.average << ";" << ikEE.errorPositionMm.deviation << ";"
			<< ikEE.errorRotation.average << ";" << ikEE.errorRotation.deviation;

		if (idxEndEffector < numEndEffectors - 1) {
			evaluationDataOutputFile << ";";
		}
		else {
			evaluationDataOutputFile << "\n";
		}
	}
	
	evaluationDataOutputFile.flush();
}

void Logger::endEvaluationLogger() {
	evaluationDataOutputFile.flush();
	evaluationDataOutputFile.close();
	
	log(Kore::Info, "Stop eval-logging!");
}

/* Reads scale as well as one positionand rotation for each end effector from file.

   Returns true iff file contains more unread data.
 */ 
bool Logger::readData(const int numOfEndEffectors, const char* filename, Kore::vec3* rawPos, Kore::Quaternion* rawRot, EndEffectorIndices indices[], float& scale) {
	std::string tag;
	float posX, posY, posZ;
	float rotX, rotY, rotZ, rotW;
	
	if(!logDataReader.is_open()) {
		
		if (std::ifstream(filename)) {
			logDataReader.open(filename);
			log(Kore::Info, "Read data from %s", filename);
			
			// Skip header
			logDataReader >> tag >> tag >> tag >> tag >> tag >> tag >> tag >> tag >> tag;
		} else {
			log(Kore::Info, "Could not find file %s", filename);
		}
	}
	
	// Read lines
	for (int i = 0; i < numOfEndEffectors; ++i) {
		logDataReader >> tag >> posX >> posY >> posZ >> rotX >> rotY >> rotZ >> rotW >> scale;
		
		EndEffectorIndices endEffectorIndex = unknown;
		if(std::strcmp(tag.c_str(), headTag) == 0)			endEffectorIndex = head;
		else if(std::strcmp(tag.c_str(), hipTag) == 0)		endEffectorIndex = hip;
		else if(std::strcmp(tag.c_str(), lHandTag) == 0)	endEffectorIndex = leftHand;
		else if(std::strcmp(tag.c_str(), rHandTag) == 0)	endEffectorIndex = rightHand;
		else if (std::strcmp(tag.c_str(), lForeArm) == 0)	endEffectorIndex = leftForeArm;
		else if (std::strcmp(tag.c_str(), rForeArm) == 0)	endEffectorIndex = rightForeArm;
		else if (std::strcmp(tag.c_str(), lFootTag) == 0)	endEffectorIndex = leftFoot;
		else if (std::strcmp(tag.c_str(), rFootTag) == 0)	endEffectorIndex = rightFoot;
		else if (std::strcmp(tag.c_str(), lKneeTag) == 0)	endEffectorIndex = leftKnee;
		else if (std::strcmp(tag.c_str(), rKneeTag) == 0)	endEffectorIndex = rightKnee;
		
		rawPos[i] = Kore::vec3(posX, posY, posZ);
		rawRot[i] = Kore::Quaternion(rotX, rotY, rotZ, rotW);
		indices[i] = endEffectorIndex;
		
		if (logDataReader.eof()) {
			logDataReader.close();
			return false;
		}
	}
	
	return true;
}
