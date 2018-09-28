#include "pch.h"
#include "Logger.h"

#include <Kore/Log.h>

#include <ctime>

namespace {
	int currLineNumber = 0;
	char* source;
	
	bool initHmmAnalysisData = false;
	int hmmHeaderLength;
	
	bool getLine(const char* tag, Kore::vec3* rawPos, Kore::Quaternion* rawRot, float* scale) {
		int i = 0;
		while(i < 9) {
			
			if (currLineNumber > 0) {
				if (i == 0) {
					tag = source;
					bool checkTag = false;
					if (tag == nullptr) checkTag = false;
					else if (strcmp(tag, headTag) == 0) checkTag = true;
					else if (strcmp(tag, hipTag) == 0) checkTag = true;
					else if (strcmp(tag, lHandTag) == 0) checkTag = true;
					else if (strcmp(tag, rHandTag) == 0) checkTag = true;
					else if (strcmp(tag, lFootTag) == 0) checkTag = true;
					else if (strcmp(tag, rFootTag) == 0) checkTag = true;
					if (!checkTag) return false;
				} else {
					try {
						float value = stof(source);
						if (i == 1) rawPos->x() = value;
						else if (i == 2) rawPos->y() = value;
						else if (i == 3) rawPos->z() = value;
						else if (i == 4) rawRot->x = value;
						else if (i == 5) rawRot->y = value;
						else if (i == 6) rawRot->z = value;
						else if (i == 7) rawRot->w = value;
						else if (i == 8) *scale = value;
					} catch (const std::exception& e) {
						Kore::log(Kore::Info, "Invalid input: %s", e.what());
						return false;
					}
				}
				i++;
			}
			// Get token
			source = strtok(NULL, ";\n");
		}
		//Kore::log(Kore::Info, "%s %f %f %f %f %f %f %f %f", tag, rawPos->x(), rawPos->y(), rawPos->z(), rawRot->x, rawRot->y, rawRot->z, rawRot->w, *scale);
		return true;
	}
}

Logger::Logger() {
}

Logger::~Logger() {
	logDataOutputFile.close();
	logDataReader.close();
	hmmDataOutputFile.close();
	hmmAnalysisOutputFile.close();
}

void Logger::startLogger(const char* filename) {
	time_t t = time(0);   // Get time now
	
	std::stringstream logFileName;
	logFileName << filename << "_" << t << ".csv";
	
	logDataOutputFile.open(logFileName.str(), std::ios::app); // Append to the end
	logDataOutputFile << "tag;rawPosX;rawPosY;rawPosZ;rawRotX;rawRotY;rawRotZ;rawRotW;scale\n";
	logDataOutputFile.flush();
	
	log(Kore::Info, "Start logging");
}

void Logger::endLogger() {
	logDataOutputFile.close();
	
	log(Kore::Info, "Stop logging");
}

void Logger::saveData(const char* tag, Kore::vec3 rawPos, Kore::Quaternion rawRot, float scale) {
	// Save positional and rotation data
	logDataOutputFile << tag << ";" << rawPos.x() << ";" << rawPos.y() << ";" << rawPos.z() << ";" << rawRot.x << ";" << rawRot.y << ";" << rawRot.z << ";" << rawRot.w << ";" << scale << "\n";
	logDataOutputFile.flush();
}

void Logger::startHMMLogger(const char* filename, int num) {
	std::stringstream logFileName;
	logFileName << filename << "_" << num << ".csv";
	
	hmmDataOutputFile.open(logFileName.str(), std::ios::out);
	const char* hmmHeader = "tag;time;posX;posY;posZ\n";
	hmmDataOutputFile << hmmHeader;
	
	// Placeholder for line number that will be overwritten when the file is closed
	hmmDataOutputFile << "N=        ;;;;\n";
	hmmHeaderLength = (int)strlen(hmmHeader);
	
	hmmDataOutputFile.flush();
	
	log(Kore::Info, "Start logging data for HMM");
}

void Logger::endHMMLogger(int lineCount) {
	hmmDataOutputFile.seekp(hmmHeaderLength);
	// Store number of lines / datapoints
	hmmDataOutputFile << "N=" << lineCount;
	hmmDataOutputFile.flush();
	hmmDataOutputFile.close();
	
	log(Kore::Info, "Stop logging data for HMM");
}

void Logger::saveHMMData(const char* tag, float lastTime, Kore::vec3 pos) {
	// Save positional and rotation data
	hmmDataOutputFile << tag << ";" << lastTime << ";"  << pos.x() << ";" << pos.y() << ";" << pos.z() << "\n";
	hmmDataOutputFile.flush();
}

void Logger::analyseHMM(const char* hmmName, double probability, bool newLine) {
	if (!initHmmAnalysisData) {
		std::stringstream hmmAnalysisPath;
		hmmAnalysisPath << hmmName << "_analysis.txt";
		hmmAnalysisOutputFile.open(hmmAnalysisPath.str(), std::ios::out | std::ios::app);
		initHmmAnalysisData = true;
	}
	
	if (newLine) hmmAnalysisOutputFile << "\n";
	else hmmAnalysisOutputFile << probability << ";";
	
	hmmAnalysisOutputFile.flush();
}

void Logger::startEvaluationLogger(const char* filename, int ikMode, float lambda, float errorMaxPos, float errorMaxRot, int maxSteps) {
	time_t t = time(0);   // Get time now
	
	evaluationDataPath.str(std::string());
	evaluationConfigPath.str(std::string());
	evaluationDataPath << "eval/" << evaluationDataFilename << "_" << t << ".csv";
	evaluationConfigPath << "eval/" << evaluationConfigFilename << "_" << t << ".csv";
	
	evaluationConfigOutputFile.open(evaluationConfigPath.str(), std::ios::app);
	evaluationConfigOutputFile << "IK Mode;File;Lambda;Error Pos Max;Error Rot Max;Steps Max\n";
	evaluationConfigOutputFile << ikMode << ";" << filename << ";" << lambda << ";" << errorMaxPos << ";" << errorMaxRot << ";" << maxSteps << "\n";
	evaluationConfigOutputFile.flush();
	evaluationConfigOutputFile.close();
	
	evaluationDataOutputFile.open(evaluationDataPath.str(), std::ios::app);
	evaluationDataOutputFile << "Iterations;Error Pos;Error Rot;Error;Time [us];Time/Iteration [us];";
	evaluationDataOutputFile << "Iterations Min;Error Pos Min;Error Rot Min;Error Min;Time [us] Min;Time/Iteration [us] Min;";
	evaluationDataOutputFile << "Iterations Max;Error Pos Max;Error Rot Max;Error Max;Time [us] Max;Time/Iteration [us] Max;";
	evaluationDataOutputFile << "Reached [%];Stucked [%]\n";
	evaluationDataOutputFile.flush();
	
	log(Kore::Info, "Start eval-logging!");
}

void Logger::endEvaluationLogger() {
	evaluationDataOutputFile.close();
	
	log(Kore::Info, "Stop eval-logging!");
}

void Logger::saveEvaluationData(Avatar *avatar) {
	float* iterations = avatar->getIterations();
	float* errorPos = avatar->getErrorPos();
	float* errorRot = avatar->getErrorRot();
	float* time = avatar->getTime();
	float* timeIteration = avatar->getTimeIteration();
	
	// Save datas
	for (int i = 0; i < 3; ++i) {
		float error = sqrtf(Square(*(errorPos + i)) + Square(*(errorRot + i)));
		
		evaluationDataOutputFile << *(iterations + i) << ";";
		evaluationDataOutputFile << *(errorPos + i) << ";";
		evaluationDataOutputFile << *(errorRot + i) << ";";
		evaluationDataOutputFile << error << ";";
		evaluationDataOutputFile << *(time + i) << ";";
		evaluationDataOutputFile << *(timeIteration + i) << ";";
	}
	evaluationDataOutputFile << avatar->getReached() << ";" << avatar->getStucked() << "\n";
	evaluationDataOutputFile.flush();
}

bool Logger::readData(const int numOfEndEffectors, const char* filename, Kore::vec3* rawPos, Kore::Quaternion* rawRot, float& scale) {
	if(currLineNumber == 0) {
		log(Kore::Info, "Read data from %s", filename);
		logDataReader.open(filename);
		
		// Real all
		void* data = logDataReader.readAll();
		int length = logDataReader.size();
		source = new char[length + 1];
		for (int i = 0; i < length; ++i) {
			source[i] = ((char*)data)[i];
		}
		
		// Skip header
		source = std::strtok(source, "\n");
		source = strtok(NULL, ";\n");
		++currLineNumber;
	}
	
	// Read lines
	for (int i = 0; i < numOfEndEffectors; ++i) {
		const char* tag;
		Kore::vec3 pos = Kore::vec3(0, 0, 0);
		Kore::Quaternion rot = Kore::Quaternion(0, 0, 0, 1);
		
		bool success = getLine(tag, &pos, &rot, &scale);
		
		if (success) {
			++currLineNumber;
			rawPos[i] = pos;
			rawRot[i] = rot;
		} else {
			//delete source;
			currLineNumber = 0;
			logDataReader.close();
			return false;
		}
	}
	
	return true;
}
