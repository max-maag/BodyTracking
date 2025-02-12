#include "pch.h"

#include <optional>
#include <array>
#include <Kore/IO/FileReader.h>
#include <Kore/Graphics4/PipelineState.h>
#include <Kore/Graphics1/Color.h>
#include <Kore/Input/Keyboard.h>
#include <Kore/Input/Mouse.h>
#include <Kore/Audio2/Audio.h>
#include <Kore/Audio1/Audio.h>
#include <Kore/Audio1/Sound.h>
#include <Kore/Audio1/SoundStream.h>
#include <Kore/System.h>
#include <Kore/Log.h>

#include "Settings.h"
#include "EndEffector.h"
#include "Avatar.h"
#include "LivingRoom.h"
#include "Logger.h"
#include "JacobianIK.h"

#include <algorithm> // std::sort, std::copy

#ifdef KORE_STEAMVR
#include <Kore/Vr/VrInterface.h>
#include <Kore/Vr/SensorState.h>
#include <Kore/Input/Gamepad.h>
#endif

#define EPSILON 0.0001   // Define your own tolerance
#define FLOAT_EQ(x,v) (((v - EPSILON) < x) && (x <( v + EPSILON)))

using namespace Kore;
using namespace Kore::Graphics4;


//							JT = 0			JPI = 1		DLS = 2		SVD = 3		SVD_DLS = 4		SDLS = 5
// Uncomment this to evaluate lambda
float lambda[6] 			= { 1.0f,		1.0f,		0.05f,		1.0f,		0.05f,			Kore::pi / 120.0f };
float evalInitValue[6]		= { 1.0f,		1.0f,		0.05f,		1.0f,		0.05f,			Kore::pi / 120.0f };
float* evalValue			= lambda;
const float evalStep[6]		= { 0.0f,		0.0f,		0.05f,		0.0f,		0.05f,			Kore::pi / 120.0f };
const float evalMaxValue[6] = { 1.0f,		1.0f,		1.5f,		1.0f,		1.5f,			Kore::pi / 4.0f };
float maxIterations[6]		= { 200.0f,		200.0f,		200.0f,		200.0f,		200.0f,			200.0f };

// Uncomment this to evaluate iterations
/*float lambda[6] 			= { 1.0f,		1.0f,		0.25f,		1.0f,		0.25f,			1.0f / 12.0f * Kore::pi };
float maxIterations[6] 		= { 1.0f,		1.0f,		1.0f,		1.0f,		1.0f,			1.0f };
float evalInitValue[6]		= { 1.0f,		1.0f,		1.0f,		1.0f,		1.0f,			1.0f };
float* evalValue			= maxIterations;
const float evalMaxValue[6] = { 100.0f,		100.0f,		100.0f,		100.0f,		100.0f,			100.0f 	};
const float evalStep[6] 	= { 1.0f,		1.0f,		1.0f,		1.0f,		1.0f,			1.0f	};*/

// Uncomment this to evaluate accuracy
/*float lambda[6]			= { 1.0f,		1.0f,		0.25f,		1.0f,		0.25f,			1.0f / 12.0f * Kore::pi };
float evalInitValue[6]		= { 1.0f,		1.0f,		0.25f,		1.0f,		0.25f,			1.0f / 12.0f * Kore::pi };
float* evalValue			= lambda;
const float evalStep[6]		= { 0.0f,		0.0f,		0.0f,		0.0f,		0.0f,			0.0f };
const float evalMaxValue[6] = { 1.0f,		1.0f,		0.25f,		1.0f,		0.25f,			1.0f / 12.0f * Kore::pi };
float maxIterations[6]		= { 200.0f,		200.0f,		200.0f,		200.0f,		200.0f,			200.0f };*/

float errorMaxPos[6] 		= { 0.0001f,	0.0001f,	0.0001f,	0.0001f,	0.0001f,		0.0001f	};
float errorMaxRot[6] 		= { 0.0001f,	0.0001f,	0.0001f,	0.0001f,	0.0001f,		0.0001f	};

namespace {
	const std::array<std::shared_ptr<IKSolver>, 6> IK_SOLVERS {
		std::make_shared<JacobianIK>(JacobianIKMode::JT,		30,		0.01,	0.01),
		std::make_shared<JacobianIK>(JacobianIKMode::JPI,		4000,	0.1,	0.1),
		std::make_shared<JacobianIK>(JacobianIKMode::DLS,		20,		0.001,	0.01),
		std::make_shared<JacobianIK>(JacobianIKMode::SVD,		10,		0.01,	0.01),
		std::make_shared<JacobianIK>(JacobianIKMode::SVD_DLS,	20,		0.001,	0.01),
		std::make_shared<JacobianIK>(JacobianIKMode::SDLS,		20,		0.01,	0.01)
	};


	size_t ikMode = 2;
	size_t currentFile = 0;

	const int width = 1024;
	const int height = 768;
	
	const bool renderRoom = true;
	const bool renderTrackerAndController = true;
	const bool renderAxisForEndEffector = false;
	
	EndEffector** endEffector;
	const int numOfEndEffectors = 10;
	
	Logger* logger;
	
	const double fpsLimit = 1.0f / 90.0f;
	double startTime;
	double lastTime;
	double lastFrameTime = 0.0f;
	
	// Audio cues
	Sound* startRecordingSound;
	Sound* stopRecordingSound;
	
	// Avatar shader
	VertexStructure structure;
	Shader* vertexShader;
	Shader* fragmentShader;
	PipelineState* pipeline;
	
	TextureUnit tex;
	ConstantLocation pLocation;
	ConstantLocation vLocation;
	ConstantLocation mLocation;
	
	// Living room shader
	VertexStructure structure_living_room;
	Shader* vertexShader_living_room;
	Shader* fragmentShader_living_room;
	PipelineState* pipeline_living_room;
	
	TextureUnit tex_living_room;
	ConstantLocation pLocation_living_room;
	ConstantLocation vLocation_living_room;
	ConstantLocation mLocation_living_room;
	ConstantLocation mLocation_living_room_inverse;
	ConstantLocation diffuse_living_room;
	ConstantLocation specular_living_room;
	ConstantLocation specular_power_living_room;
	ConstantLocation lightPosLocation_living_room;
	ConstantLocation lightCount_living_room;
	
	// Keyboard controls
	bool rotate = false;
	bool W, A, S, D = false;
	
	vec4 camUp(0.0f, 1.0f, 0.0f, 0.0f);
	vec4 camForward(0.0f, 0.0f, 1.0f, 0.0f);
	vec4 camRight(1.0f, 0.0f, 0.0f, 0.0f);
	
	vec3 cameraPos(0, 0, 0);
	
	// Null terminated array of MeshObject pointers (Vive Controller and Tracker)
	MeshObject* viveObjects[] = { nullptr, nullptr, nullptr };
	Avatar* avatar;
	LivingRoom* livingRoom;
	
	// Variables to mirror the room and the avatar
	vec3 mirrorOver(6.057f, 0.0f, 0.04f);
	
	mat4 initTrans;
	mat4 initTransInv;
	Kore::Quaternion initRot;
	Kore::Quaternion initRotInv;

	std::optional<std::array<IKEvaluator, numEndEffectors>> ikEvaluators = Settings::eval ? std::make_optional(std::array<IKEvaluator, numEndEffectors>()) : std::nullopt;
	
	bool calibratedAvatar = false;
	
#ifdef KORE_STEAMVR
	bool controllerButtonsInitialized = false;
	float currentUserHeight;
	bool firstPersonMonitor = false;
#endif
	
	void renderVRDevice(int index, Kore::mat4 M) {
		Graphics4::setMatrix(mLocation, M);
		viveObjects[index]->render(tex);
	}
	
	mat4 getMirrorMatrix() {
		Kore::Quaternion rot(0, 0, 0, 1);
		rot.rotate(Kore::Quaternion(vec3(0, 1, 0), Kore::pi));
		mat4 zMirror = mat4::Identity();
		zMirror.Set(2, 2 , -1);
		Kore::mat4 M = zMirror * mat4::Translation(mirrorOver.x(), mirrorOver.y(), mirrorOver.z()) * rot.matrix().Transpose();
		
		return M;
	}
	
	void renderControllerAndTracker(int tracker, Kore::vec3 desPosition, Kore::Quaternion desRotation) {
		// World Transformation Matrix
		Kore::mat4 W = mat4::Translation(desPosition.x(), desPosition.y(), desPosition.z()) * desRotation.matrix().Transpose();
		
		// Mirror Transformation Matrix
		Kore::mat4 M = getMirrorMatrix() * W;
		
		if (tracker) {
			// Render a tracker for both feet and back
			renderVRDevice(0, W);
			renderVRDevice(0, M);
		} else {
			// Render a controller for both hands
			renderVRDevice(1, W);
			renderVRDevice(1, M);
		}
		
		// Render a local coordinate system only if the avatar is not calibrated
		if (!calibratedAvatar) {
			renderVRDevice(2, W);
			renderVRDevice(2, M);
		}
	}
	
	void renderAllVRDevices() {
		Graphics4::setPipeline(pipeline);
	
#ifdef KORE_STEAMVR
		VrPoseState controller;
		for (int i = 0; i < 16; ++i) {
			controller = VrInterface::getController(i);
			
			vec3 desPosition = controller.vrPose.position;
			Kore::Quaternion desRotation = controller.vrPose.orientation;
			
			if (controller.trackedDevice == TrackedDevice::ViveTracker) {
				renderControllerAndTracker(true, desPosition, desRotation);
			} else if (controller.trackedDevice == TrackedDevice::Controller) {
				renderControllerAndTracker(false, desPosition, desRotation);
			}
			
		}
#else
		for(int i = 0; i < numOfEndEffectors; ++i) {
			Kore::vec3 desPosition = endEffector[i]->getDesPosition();
			Kore::Quaternion desRotation = endEffector[i]->getDesRotation();
			
			if (i == hip || (!Settings::simpleIK && i == leftForeArm) || (!Settings::simpleIK && i == rightForeArm) || i == leftFoot || i == rightFoot) {
				renderControllerAndTracker(true, desPosition, desRotation);
			} else if (i == rightHand || i == leftHand) {
				renderControllerAndTracker(false, desPosition, desRotation);
			}
		}
#endif
	}
	
	void renderCSForEndEffector() {
		Graphics4::setPipeline(pipeline);
		
		for(int i = 0; i < numOfEndEffectors; ++i) {
			BoneNode* bone = avatar->getBoneWithIndex(endEffector[i]->getBoneIndex());
			
			vec3 endEffectorPos = bone->getPosition();
			endEffectorPos = initTrans * vec4(endEffectorPos.x(), endEffectorPos.y(), endEffectorPos.z(), 1);
			Kore::Quaternion endEffectorRot = initRot.rotated(bone->getOrientation());
			
			Kore::mat4 M = mat4::Translation(endEffectorPos.x(), endEffectorPos.y(), endEffectorPos.z()) * endEffectorRot.matrix().Transpose();
			Graphics4::setMatrix(mLocation, M);
			viveObjects[2]->render(tex);
		}
	}
	
	void renderLivingRoom(mat4 V, mat4 P) {
		Graphics4::setPipeline(pipeline_living_room);
		
		livingRoom->setLights(lightCount_living_room, lightPosLocation_living_room);
		Graphics4::setMatrix(vLocation_living_room, V);
		Graphics4::setMatrix(pLocation_living_room, P);
		livingRoom->render(tex_living_room, mLocation_living_room, mLocation_living_room_inverse, diffuse_living_room, specular_living_room, specular_power_living_room, false);
		
		livingRoom->render(tex_living_room, mLocation_living_room, mLocation_living_room_inverse, diffuse_living_room, specular_living_room, specular_power_living_room, true);
	}
	
	void renderAvatar(mat4 V, mat4 P) {
		Graphics4::setPipeline(pipeline);
		
		Graphics4::setMatrix(vLocation, V);
		Graphics4::setMatrix(pLocation, P);
		Graphics4::setMatrix(mLocation, initTrans);
		avatar->animate(tex);
		
		// Mirror the avatar
		mat4 initTransMirror = getMirrorMatrix() * initTrans;
		
		Graphics4::setMatrix(mLocation, initTransMirror);
		avatar->animate(tex);
	}
	
	Kore::mat4 getProjectionMatrix() {
		mat4 P = mat4::Perspective(45, (float)width / (float)height, 0.01f, 1000);
		P.Set(0, 0, -P.get(0, 0));
		
		return P;
	}
	
	Kore::mat4 getViewMatrix() {
		mat4 V = mat4::lookAlong(camForward.xyz(), cameraPos, vec3(0.0f, 1.0f, 0.0f));
		return V;
	}

	IKEvaluator* getIkEvaluator(int endEffectorID) {
		return ikEvaluators ? &(ikEvaluators.value()[endEffectorID]) : nullptr;
	}
	
	void executeMovement(int endEffectorID) {
		Kore::vec3 desPosition = endEffector[endEffectorID]->getDesPosition();
		Kore::Quaternion desRotation = endEffector[endEffectorID]->getDesRotation();

		// Save raw data
		if (Settings::logRawData) {
			logger->saveData(endEffector[endEffectorID]->getName(), desPosition, desRotation, avatar->scale);
		}
		
		if (calibratedAvatar) {
			// Transform desired position/rotation to the character local coordinate system
			desPosition = initTransInv * vec4(desPosition.x(), desPosition.y(), desPosition.z(), 1);
			desRotation = initRotInv.rotated(desRotation);

			// Add offset
			Kore::Quaternion offsetRotation = endEffector[endEffectorID]->getOffsetRotation();
			vec3 offsetPosition = endEffector[endEffectorID]->getOffsetPosition();
			Kore::Quaternion finalRot = desRotation.rotated(offsetRotation);
			vec3 finalPos = mat4::Translation(desPosition.x(), desPosition.y(), desPosition.z()) * finalRot.matrix().Transpose() * mat4::Translation(offsetPosition.x(), offsetPosition.y(), offsetPosition.z()) * vec4(0, 0, 0, 1);

			endEffector[endEffectorID]->setFinalPosition(finalPos);
			endEffector[endEffectorID]->setFinalRotation(finalRot);
			
			if (endEffectorID == hip) {
				avatar->setFixedPositionAndOrientation(endEffector[endEffectorID]->getBoneIndex(), finalPos, finalRot);
			} else if (endEffectorID == head) {
				avatar->setDesiredPositionAndOrientation(endEffector[endEffectorID]->getBoneIndex(), finalPos, finalRot, getIkEvaluator(endEffectorID));
			} else if (endEffectorID == leftForeArm || endEffectorID == rightForeArm) {
				if (!Settings::simpleIK)
					avatar->setDesiredPositionAndOrientation(endEffector[endEffectorID]->getBoneIndex(), finalPos, finalRot, getIkEvaluator(endEffectorID));
			} else if (endEffectorID == leftFoot || endEffectorID == rightFoot) {
				avatar->setDesiredPositionAndOrientation(endEffector[endEffectorID]->getBoneIndex(), finalPos, finalRot, getIkEvaluator(endEffectorID));
			} else if (endEffectorID == leftHand || endEffectorID == rightHand) {
				if (Settings::simpleIK) {
					avatar->setDesiredPositionAndOrientation(endEffector[endEffectorID]->getBoneIndex(), finalPos, finalRot, getIkEvaluator(endEffectorID));
				} else {
					avatar->setFixedOrientation(endEffector[endEffectorID]->getBoneIndex(), finalRot);
				}
			}
		}
	}
	
	void initTransAndRot() {
		initRot = Kore::Quaternion(0, 0, 0, 1);
		initRot.rotate(Kore::Quaternion(vec3(1, 0, 0), -Kore::pi / 2.0));
		initRot.rotate(Kore::Quaternion(vec3(0, 0, 1), Kore::pi / 2.0));
		initRot.normalize();
		initRotInv = initRot.invert();
		
		// Move character in the middle of both feet
		Kore::vec3 initPos = Kore::vec3(0, 0, 0);
		Kore::vec3 posLeftFoot = endEffector[leftFoot]->getDesPosition();
		Kore::vec3 posRightFoot = endEffector[rightFoot]->getDesPosition();
		//initPos = (posRightFoot + posLeftFoot) / 2.0f;
		initPos.y() = 0.0f;

		initTrans = mat4::Translation(initPos.x(), initPos.y(), initPos.z()) * initRot.matrix().Transpose();
		initTransInv = initTrans.Invert();
	}
	
	void calibrate() {
		initTransAndRot();
		
		for (int i = 0; i < numOfEndEffectors; ++i) {
			Kore::vec3 desPosition = endEffector[i]->getDesPosition();
			Kore::Quaternion desRotation = endEffector[i]->getDesRotation();
			
			// Transform desired position/rotation to the character local coordinate system
			desPosition = initTransInv * vec4(desPosition.x(), desPosition.y(), desPosition.z(), 1);
			desRotation = initRotInv.rotated(desRotation);
			
			// Get actual position/rotation of the character skeleton
			BoneNode* bone = avatar->getBoneWithIndex(endEffector[i]->getBoneIndex());
			vec3 targetPos = bone->getPosition();
			Kore::Quaternion targetRot = bone->getOrientation();
			
			endEffector[i]->setOffsetPosition((mat4::Translation(desPosition.x(), desPosition.y(), desPosition.z()) * targetRot.matrix().Transpose()).Invert() * mat4::Translation(targetPos.x(), targetPos.y(), targetPos.z()) * vec4(0, 0, 0, 1));
			endEffector[i]->setOffsetRotation((desRotation.invert()).rotated(targetRot));
		}
	}
	
void record() {
	Settings::logRawData = !Settings::logRawData;
	
	if (Settings::logRawData) {
		Audio1::play(startRecordingSound);
		logger->startLogger("logData");
	} else {
		Audio1::play(stopRecordingSound);
		logger->endLogger();
	}
}

#ifdef KORE_STEAMVR
	void setSize() {
		float currentAvatarHeight = avatar->getHeight();
		
		SensorState state = VrInterface::getSensorState(0);
		vec3 hmdPos = state.pose.vrPose.position; // z -> face, y -> up down
		currentUserHeight = hmdPos.y();
		
		float scale = currentUserHeight / currentAvatarHeight;
		avatar->setScale(scale);
		
		Kore::log(Info, "current avatar height %f, current user height %f ==> scale %f", currentAvatarHeight, currentUserHeight, scale);
	}
	
	void initEndEffector(int efID, int deviceID, vec3 pos, Kore::Quaternion rot) {
		endEffector[efID]->setDeviceIndex(deviceID);
		endEffector[efID]->setDesPosition(pos);
		endEffector[efID]->setDesRotation(rot);
		
		Kore::log(Info, "%s, device id: %i", endEffector[efID]->getName(), deviceID);
	}
	
	void assignControllerAndTracker() {
		VrPoseState vrDevice;

		int trackerCount = 0;
		
		std::vector<EndEffector*> trackers;
		
		// Get indices for VR devices
		for (int i = 0; i < 16; ++i) {
			vrDevice = VrInterface::getController(i);
			
			vec3 devicePos = vrDevice.vrPose.position;
			Kore::Quaternion deviceRot = vrDevice.vrPose.orientation;

			if (vrDevice.trackedDevice == TrackedDevice::ViveTracker) {
				EndEffector* tracker = new EndEffector(-1, (IKMode)ikMode);
				tracker->setDeviceIndex(i);
				tracker->setDesPosition(devicePos);
				tracker->setDesRotation(deviceRot);
				trackers.push_back(tracker);
				
				++trackerCount;
				if (trackerCount == numTrackers) {
					// Sort trackers regarding the y-Axis (height)
					std::sort(trackers.begin(), trackers.end(), sortByYAxis());
					
					if (numTrackers == 7) {
						// Left or Right Foot
						std::sort(trackers.begin(), trackers.begin()+2, sortByZAxis());
						initEndEffector(leftFoot, trackers[0]->getDeviceIndex(), trackers[0]->getDesPosition(), trackers[0]->getDesRotation());
						initEndEffector(rightFoot, trackers[1]->getDeviceIndex(), trackers[1]->getDesPosition(), trackers[1]->getDesRotation());
					
						// Left or Right Knee
						std::sort(trackers.begin()+2, trackers.begin()+4, sortByZAxis());
						initEndEffector(leftKnee, trackers[2]->getDeviceIndex(), trackers[2]->getDesPosition(), trackers[2]->getDesRotation());
						initEndEffector(rightKnee, trackers[3]->getDeviceIndex(), trackers[3]->getDesPosition(), trackers[3]->getDesRotation());
					
						// Hip
						initEndEffector(hip, trackers[4]->getDeviceIndex(), trackers[4]->getDesPosition(), trackers[4]->getDesRotation());
					
						// Left Forearm And Right Forearm
						std::sort(trackers.begin()+5, trackers.begin()+7, sortByZAxis());
						initEndEffector(leftForeArm, trackers[5]->getDeviceIndex(), trackers[5]->getDesPosition(), trackers[5]->getDesRotation());
						initEndEffector(rightForeArm, trackers[6]->getDeviceIndex(), trackers[6]->getDesPosition(), trackers[6]->getDesRotation());
					} else {
						// Left or Right Leg
						std::sort(trackers.begin(), trackers.begin()+2, sortByZAxis());
						initEndEffector(leftFoot, trackers[0]->getDeviceIndex(), trackers[0]->getDesPosition(), trackers[0]->getDesRotation());
						initEndEffector(rightFoot, trackers[1]->getDeviceIndex(), trackers[1]->getDesPosition(), trackers[1]->getDesRotation());
						
						// Hip
						initEndEffector(hip, trackers[2]->getDeviceIndex(), trackers[2]->getDesPosition(), trackers[2]->getDesRotation());
						
						// Left or Right Forearm
						if (!simpleIK) {
							std::sort(trackers.begin()+3, trackers.begin()+5, sortByZAxis());
							initEndEffector(leftForeArm, trackers[3]->getDeviceIndex(), trackers[3]->getDesPosition(), trackers[3]->getDesRotation());
							initEndEffector(rightForeArm, trackers[4]->getDeviceIndex(), trackers[4]->getDesPosition(), trackers[4]->getDesRotation());
						}
					}
				}
				
				
			} else if (vrDevice.trackedDevice == TrackedDevice::Controller) {
				// Hand controller
				if (devicePos.z() > 0) {
					initEndEffector(rightHand, i, devicePos, deviceRot);
				} else {
					initEndEffector(leftHand, i, devicePos, deviceRot);
				}
			}
		}
		
		// HMD
		SensorState stateLeftEye = VrInterface::getSensorState(0);
		SensorState stateRightEye = VrInterface::getSensorState(1);
		vec3 leftEyePos = stateLeftEye.pose.vrPose.position;
		vec3 rightEyePos = stateRightEye.pose.vrPose.position;
		vec3 hmdPosCenter = (leftEyePos + rightEyePos) / 2;
		initEndEffector(head, 0, hmdPosCenter, stateLeftEye.pose.vrPose.orientation);
	}
	
	void gamepadButton(int buttonNr, float value) {
		//Kore::log(Info, "gamepadButton buttonNr = %i value = %f", buttonNr, value);

		// Grip button => set size and reset an avatar to a default T-Pose
		if (buttonNr == 2 && value == 1) {
			calibratedAvatar = false;
			initTransAndRot();
			avatar->resetPositionAndRotation();
			setSize();
		}
		
		// Menu button => calibrate
		if (buttonNr == 1 && value == 1) {
			assignControllerAndTracker();
			calibrate();
			calibratedAvatar = true;
			Kore::log(Info, "Calibrate avatar");
		}
		
		// Track a movement as long as trigger button is pressed
		if (buttonNr == 33 && value == 1) {
			// Trigger button pressed
			Kore::log(Info, "Trigger button pressed");
			if (calibratedAvatar) {
				record();
			}
		}
		
		if (buttonNr == 33 && value == 0) {
			// Trigger button released
			Kore::log(Info, "Trigger button released");
			if (calibratedAvatar) {
				record();
			}
		}
	}
	
	void initButtons() {
		VrPoseState controller;

		int count = 0;
		
		for (int i = 0; i < 16; ++i) {
			controller = VrInterface::getController(i);
			
			if (controller.trackedDevice == TrackedDevice::Controller) {
				Gamepad::get(i)->Button = gamepadButton;
				++count;
				Kore::log(Info, "Add gamepad controller %i", count);
			}
		}

		assert(count == 2);
		controllerButtonsInitialized = true;
	}
#endif

	void update() {
		float t = (float)(System::time() - startTime);
		double deltaT = t - lastTime;
		lastTime = t;
		
		// Move position of camera based on WASD keys
		float cameraMoveSpeed = 4.f;
		if (S) cameraPos -= camForward * (float)deltaT * cameraMoveSpeed;
		if (W) cameraPos += camForward * (float)deltaT * cameraMoveSpeed;
		if (A) cameraPos += camRight * (float)deltaT * cameraMoveSpeed;
		if (D) cameraPos -= camRight * (float)deltaT * cameraMoveSpeed;
		
		Graphics4::begin();
		Graphics4::clear(Graphics4::ClearColorFlag | Graphics4::ClearDepthFlag, Graphics1::Color::Black, 1.0f, 0);
		Graphics4::setPipeline(pipeline);
		
#ifdef KORE_STEAMVR
		VrInterface::begin();

		if (!controllerButtonsInitialized) initButtons();
		
		VrPoseState vrDevice;
		for (int i = 0; i < numOfEndEffectors; ++i) {
			if (endEffector[i]->getDeviceIndex() != -1) {

				if (i == head) {
					SensorState state = VrInterface::getSensorState(0);

					// Get HMD position and rotation
					endEffector[i]->setDesPosition(state.pose.vrPose.position);
					endEffector[i]->setDesRotation(state.pose.vrPose.orientation);
				} else {
					vrDevice = VrInterface::getController(endEffector[i]->getDeviceIndex());

					// Get VR device position and rotation
					endEffector[i]->setDesPosition(vrDevice.vrPose.position);
					endEffector[i]->setDesRotation(vrDevice.vrPose.orientation);
				}

				executeMovement(i);
			}
		}
		
		// Render for both eyes
		SensorState state;
		for (int j = 0; j < 2; ++j) {
			VrInterface::beginRender(j);
			
			Graphics4::clear(Graphics4::ClearColorFlag | Graphics4::ClearDepthFlag, Graphics1::Color::Black, 1.0f, 0);
			
			state = VrInterface::getSensorState(j);
			
			renderAvatar(state.pose.vrPose.eye, state.pose.vrPose.projection);
			
			if (renderTrackerAndController) renderAllVRDevices();
			
			if (renderAxisForEndEffector) renderCSForEndEffector();
			
			if (renderRoom) renderLivingRoom(state.pose.vrPose.eye, state.pose.vrPose.projection);
			
			VrInterface::endRender(j);
		}
		
		VrInterface::warpSwap();
		
		Graphics4::restoreRenderTarget();
		Graphics4::clear(Graphics4::ClearColorFlag | Graphics4::ClearDepthFlag, Graphics1::Color::Black, 1.0f, 0);
		
		// Render on monitor
		mat4 P = getProjectionMatrix();
		mat4 V = getViewMatrix();

		if (!firstPersonMonitor) renderAvatar(V, P);
		else renderAvatar(state.pose.vrPose.eye, state.pose.vrPose.projection);
		
		if (renderTrackerAndController) renderAllVRDevices();
		
		if (renderAxisForEndEffector) renderCSForEndEffector();
		
		if (renderRoom) {
			if (!firstPersonMonitor) renderLivingRoom(V, P);
			else renderLivingRoom(state.pose.vrPose.eye, state.pose.vrPose.projection);
		}
#else
		// Read line
		if ((t - lastFrameTime) >= fpsLimit) {
			float scaleFactor;
			EndEffectorIndices indices[numOfEndEffectors];
			Kore::vec3 desPosition[numOfEndEffectors];
			Kore::Quaternion desRotation[numOfEndEffectors];
			if (currentFile < Settings::files.size()) {
				bool dataAvailable = logger->readData(numOfEndEffectors, Settings::files[currentFile].c_str(), desPosition, desRotation, indices, scaleFactor);

				if (dataAvailable) {
					for (int i = 0; i < numOfEndEffectors; ++i) {
						EndEffectorIndices index = indices[i];
						endEffector[index]->setDesPosition(desPosition[i]);
						endEffector[index]->setDesRotation(desRotation[i]);
					}
				}

				if (!calibratedAvatar) {
					avatar->resetPositionAndRotation();
					avatar->setScale(scaleFactor);
					calibrate();
					calibratedAvatar = true;

					if (Settings::eval) {
						for (auto& evaluator : ikEvaluators.value()) {
							evaluator.clear();
						}
					}
				}

				for (int i = 0; i < numOfEndEffectors; ++i) {
					executeMovement(i);
				}

				if (!dataAvailable) {

					if (Settings::eval) {

						std::array<IKEvaluator::RunStatsAverage, numEndEffectors> ikStatsPerEndEffector;
						IKEvaluator::RunStatsAverage ikStatsTotal;

						// get average run stats per end effector and sum averages for total stats
						for (size_t idxEndEffector = 0; idxEndEffector < ikStatsPerEndEffector.size(); idxEndEffector++) {
							IKEvaluator::RunStatsAverage rsa = ikEvaluators.value()[idxEndEffector].getAverage();
							ikStatsPerEndEffector[idxEndEffector] = rsa;

							ikStatsTotal.numIterations.average += rsa.numIterations.average;
							ikStatsTotal.durationsMs.average += rsa.durationsMs.average;
							ikStatsTotal.durationAveragePerIterationMs.average += rsa.durationAveragePerIterationMs.average;
							ikStatsTotal.errorPositionMm.average += rsa.errorPositionMm.average;
							ikStatsTotal.errorRotation.average += rsa.errorRotation.average;
							ikStatsTotal.targetReachedPercent += rsa.targetReachedPercent;
							ikStatsTotal.stuckPercent += rsa.stuckPercent;
						}

						ikStatsTotal.numIterations.average /= numEndEffectors;
						ikStatsTotal.durationsMs.average /= numEndEffectors;
						ikStatsTotal.durationAveragePerIterationMs.average /= numEndEffectors;
						ikStatsTotal.errorPositionMm.average /= numEndEffectors;
						ikStatsTotal.errorRotation.average /= numEndEffectors;
						ikStatsTotal.targetReachedPercent /= numEndEffectors;
						ikStatsTotal.stuckPercent /= numEndEffectors;

						// calculate deviation of total stats
						for (size_t idxEndEffector = 0; idxEndEffector < ikStatsPerEndEffector.size(); idxEndEffector++) {
							IKEvaluator::RunStatsAverage statsEndEffector = ikEvaluators.value()[idxEndEffector].getAverage();

							ikStatsTotal.numIterations.deviation += Kore::pow(statsEndEffector.numIterations.average - ikStatsTotal.numIterations.average, 2);
							ikStatsTotal.durationsMs.deviation += Kore::pow(statsEndEffector.durationsMs.average - ikStatsTotal.durationsMs.average, 2);
							ikStatsTotal.durationAveragePerIterationMs.deviation += Kore::pow(statsEndEffector.durationAveragePerIterationMs.average - ikStatsTotal.durationAveragePerIterationMs.average, 2);
							ikStatsTotal.errorPositionMm.deviation += Kore::pow(statsEndEffector.errorPositionMm.average - ikStatsTotal.errorPositionMm.average, 2);
							ikStatsTotal.errorRotation.deviation += Kore::pow(statsEndEffector.errorRotation.average - ikStatsTotal.errorRotation.average, 2);
						}

						ikStatsTotal.numIterations.deviation = Kore::sqrt(ikStatsTotal.numIterations.deviation / numEndEffectors);
						ikStatsTotal.durationsMs.deviation = Kore::sqrt(ikStatsTotal.durationsMs.deviation / numEndEffectors);
						ikStatsTotal.durationAveragePerIterationMs.deviation = Kore::sqrt(ikStatsTotal.durationAveragePerIterationMs.deviation / numEndEffectors);
						ikStatsTotal.errorPositionMm.deviation = Kore::sqrt(ikStatsTotal.errorPositionMm.deviation / numEndEffectors);
						ikStatsTotal.errorRotation.deviation = Kore::sqrt(ikStatsTotal.errorRotation.deviation / numEndEffectors);

						for (size_t idxEndEffector = 0; idxEndEffector < numEndEffectors; idxEndEffector++) {
							Kore::log(
								LogLevel::Info,
								"Error %s = %f, %f",
								endEffector[idxEndEffector]->getName(),
								ikStatsPerEndEffector[idxEndEffector].errorPositionMm.average,
								ikStatsPerEndEffector[idxEndEffector].errorRotation.average
							);
						}

						Kore::log(
							LogLevel::Info,
							"Overall Error Pos = %f +- %f, Rot = %f +- %f",
							ikStatsTotal.errorPositionMm.average,
							ikStatsTotal.errorPositionMm.deviation,
							ikStatsTotal.errorRotation.average,
							ikStatsTotal.errorRotation.deviation
						);

						logger->saveEvaluationData(
							Settings::files[currentFile],
							avatar->getIkSolver(),
							ikStatsTotal,
							ikStatsPerEndEffector);

						if (FLOAT_EQ(evalValue[ikMode], evalMaxValue[ikMode])) {
							//if (evalValue[ikMode] >= evalMaxValue[ikMode]) {
							logger->endEvaluationLogger();

							evalValue[ikMode] = evalInitValue[ikMode];


							ikMode = ikMode + 1;
							if (ikMode > Settings::evalMaxIk) {
								ikMode = Settings::evalMinIk;
								currentFile++;
							}

							avatar->setIkSolver(IK_SOLVERS[ikMode]);
						}
						else {
							evalValue[ikMode] += evalStep[ikMode];
						}

						calibratedAvatar = false;

					}
					else {
						currentFile++;
						calibratedAvatar = false;
					}
				}
			}
			else {
				System::stop();
			}
			lastFrameTime = t;
		}
		
		// Get projection and view matrix
		mat4 P = getProjectionMatrix();
		mat4 V = getViewMatrix();
		
		renderAvatar(V, P);
		
		if (renderTrackerAndController) renderAllVRDevices();
		
		if (renderAxisForEndEffector) renderCSForEndEffector();
		
		if (renderRoom) renderLivingRoom(V, P);
#endif

		Graphics4::end();
		Graphics4::swapBuffers();
	}
	
	void keyDown(KeyCode code) {
		switch (code) {
			case KeyW:
				W = true;
				break;
			case KeyA:
				A = true;
				break;
			case KeyS:
				S = true;
				break;
			case KeyD:
				D = true;
				break;
			case KeyR:
#ifdef KORE_STEAMVR
				VrInterface::resetHmdPose();
#endif
				break;
			case KeyL:
				//Kore::log(Kore::LogLevel::Info, "cameraPos: (%f, %f, %f)", cameraPos.x(), cameraPos.y(), cameraPos.z());
				//Kore::log(Kore::LogLevel::Info, "camUp: (%f, %f, %f, %f)", camUp.x(), camUp.y(), camUp.z(), camUp.w());
				//Kore::log(Kore::LogLevel::Info, "camRight: (%f, %f, %f, %f)", camRight.x(), camRight.y(), camRight.z(), camRight.w());
				//Kore::log(Kore::LogLevel::Info, "camForward: (%f, %f, %f, %f)", camForward.x(), camForward.y(), camForward.z(), camForward.w());
				
				record();
				break;
			case KeyEscape:
			case KeyQ:
				System::stop();
				break;
			default:
				break;
		}
	}
	
	void keyUp(KeyCode code) {
		switch (code) {
			case KeyW:
				W = false;
				break;
			case KeyA:
				A = false;
				break;
			case KeyS:
				S = false;
				break;
			case KeyD:
				D = false;
				break;
			case KeyL:
				record();
				break;
			default:
				break;
		}
	}
	
	void mouseMove(int windowId, int x, int y, int movementX, int movementY) {
		Kore::Quaternion q1(vec3(0.0f, 1.0f, 0.0f), 0.01f * movementX);
		Kore::Quaternion q2(camRight, 0.01f * -movementY);
		
		camUp = q2.matrix() * camUp;
		camRight = q1.matrix() * camRight;
		
		q2.rotate(q1);
		mat4 mat = q2.matrix();
		camForward = mat * camForward;
	}
	
	
	void mousePress(int windowId, int button, int x, int y) {
		rotate = true;
	}
	
	void mouseRelease(int windowId, int button, int x, int y) {
		rotate = false;
	}
	
	void loadAvatarShader() {
		FileReader vs("shader.vert");
		FileReader fs("shader.frag");
		vertexShader = new Shader(vs.readAll(), vs.size(), VertexShader);
		fragmentShader = new Shader(fs.readAll(), fs.size(), FragmentShader);
		
		// This defines the structure of your Vertex Buffer
		structure.add("pos", Float3VertexData);
		structure.add("tex", Float2VertexData);
		structure.add("nor", Float3VertexData);
		
		pipeline = new PipelineState();
		pipeline = new PipelineState();
		pipeline->inputLayout[0] = &structure;
		pipeline->inputLayout[1] = nullptr;
		pipeline->vertexShader = vertexShader;
		pipeline->fragmentShader = fragmentShader;
		pipeline->depthMode = ZCompareLess;
		pipeline->depthWrite = true;
		pipeline->blendSource = Graphics4::SourceAlpha;
		pipeline->blendDestination = Graphics4::InverseSourceAlpha;
		pipeline->alphaBlendSource = Graphics4::SourceAlpha;
		pipeline->alphaBlendDestination = Graphics4::InverseSourceAlpha;
		pipeline->compile();
		
		tex = pipeline->getTextureUnit("tex");
		Graphics4::setTextureAddressing(tex, Graphics4::U, Repeat);
		Graphics4::setTextureAddressing(tex, Graphics4::V, Repeat);
		
		pLocation = pipeline->getConstantLocation("P");
		vLocation = pipeline->getConstantLocation("V");
		mLocation = pipeline->getConstantLocation("M");
	}
	
	void loadLivingRoomShader() {
		FileReader vs("shader_basic_shading.vert");
		FileReader fs("shader_basic_shading.frag");
		vertexShader_living_room = new Shader(vs.readAll(), vs.size(), VertexShader);
		fragmentShader_living_room = new Shader(fs.readAll(), fs.size(), FragmentShader);
		
		structure_living_room.add("pos", Float3VertexData);
		structure_living_room.add("tex", Float2VertexData);
		structure_living_room.add("nor", Float3VertexData);
		
		pipeline_living_room = new PipelineState();
		pipeline_living_room->inputLayout[0] = &structure_living_room;
		pipeline_living_room->inputLayout[1] = nullptr;
		pipeline_living_room->vertexShader = vertexShader_living_room;
		pipeline_living_room->fragmentShader = fragmentShader_living_room;
		pipeline_living_room->depthMode = ZCompareLess;
		pipeline_living_room->depthWrite = true;
		pipeline_living_room->blendSource = Graphics4::SourceAlpha;
		pipeline_living_room->blendDestination = Graphics4::InverseSourceAlpha;
		pipeline_living_room->alphaBlendSource = Graphics4::SourceAlpha;
		pipeline_living_room->alphaBlendDestination = Graphics4::InverseSourceAlpha;
		pipeline_living_room->compile();
		
		tex_living_room = pipeline_living_room->getTextureUnit("tex");
		Graphics4::setTextureAddressing(tex_living_room, Graphics4::U, Repeat);
		Graphics4::setTextureAddressing(tex_living_room, Graphics4::V, Repeat);
		
		pLocation_living_room = pipeline_living_room->getConstantLocation("P");
		vLocation_living_room = pipeline_living_room->getConstantLocation("V");
		mLocation_living_room = pipeline_living_room->getConstantLocation("M");
		mLocation_living_room_inverse = pipeline_living_room->getConstantLocation("MInverse");
		diffuse_living_room = pipeline_living_room->getConstantLocation("diffuseCol");
		specular_living_room = pipeline_living_room->getConstantLocation("specularCol");
		specular_power_living_room = pipeline_living_room->getConstantLocation("specularPow");
		lightPosLocation_living_room = pipeline_living_room->getConstantLocation("lightPos");
		lightCount_living_room = pipeline_living_room->getConstantLocation("numLights");
	}
	
	void init() {
		loadAvatarShader();

		avatar = new Avatar("avatar/avatar_male.ogex", "avatar/", structure, IK_SOLVERS[ikMode]);

		//avatar = new Avatar("avatar/avatar_female.ogex", "avatar/", structure, IK_SOLVERS[ikMode]);
		
		// Set camera initial position and orientation
		cameraPos = vec3(2.6, 1.8, 0.0);
		Kore::Quaternion q1(vec3(0.0f, 1.0f, 0.0f), Kore::pi / 2.0f);
		Kore::Quaternion q2(vec3(1.0f, 0.0f, 0.0f), -Kore::pi / 8.0f);
		camUp = q2.matrix() * camUp;
		camRight = q1.matrix() * camRight;
		q2.rotate(q1);
		mat4 mat = q2.matrix();
		camForward = mat * camForward;
		
		if (renderTrackerAndController) {
			viveObjects[0] = new MeshObject("vivemodels/vivetracker.ogex", "vivemodels/", structure, 1);
			viveObjects[1] = new MeshObject("vivemodels/vivecontroller.ogex", "vivemodels/", structure, 1);
		}
		
		if (renderTrackerAndController || renderAxisForEndEffector) {
			viveObjects[2] = new MeshObject("vivemodels/axis.ogex", "vivemodels/", structure, 1);
		}
		
		if (renderRoom) {
			loadLivingRoomShader();
			livingRoom = new LivingRoom("sherlock_living_room/sherlock_living_room.ogex", "sherlock_living_room/", structure_living_room, 1);
			Kore::Quaternion livingRoomRot = Kore::Quaternion(0, 0, 0, 1);
			livingRoomRot.rotate(Kore::Quaternion(vec3(1, 0, 0), -Kore::pi / 2.0));
			livingRoomRot.rotate(Kore::Quaternion(vec3(0, 0, 1), Kore::pi / 2.0));
			livingRoom->M = mat4::Translation(0, 0, 0) * livingRoomRot.matrix().Transpose();
			
			mat4 mirrorMatrix = mat4::Identity();
			mirrorMatrix.Set(2, 2, -1);
			livingRoomRot.rotate(Kore::Quaternion(vec3(0, 0, 1), Kore::pi));
			livingRoom->Mmirror = mirrorMatrix * mat4::Translation(mirrorOver.x(), mirrorOver.y(), mirrorOver.z()) * livingRoomRot.matrix().Transpose();
		}
		
		logger = new Logger();
		
		if(!Settings::eval) {
			std::copy(Settings::optimalLambda, Settings::optimalLambda + 6, lambda);
			std::copy(Settings::optimalErrorMaxPos, Settings::optimalErrorMaxPos + 6, errorMaxPos);
			std::copy(Settings::optimalErrorMaxRot, Settings::optimalErrorMaxRot + 6, errorMaxRot);
			std::copy(Settings::optimalMaxIterations, Settings::optimalMaxIterations + 6, maxIterations);
		}
		
		endEffector = new EndEffector*[numOfEndEffectors];
		endEffector[head] = new EndEffector(headBoneIndex);
		endEffector[hip] = new EndEffector(hipBoneIndex);
		endEffector[leftHand] = new EndEffector(leftHandBoneIndex);
		endEffector[leftForeArm] = new EndEffector(leftForeArmBoneIndex);
		endEffector[rightHand] = new EndEffector(rightHandBoneIndex);
		endEffector[rightForeArm] = new EndEffector(rightForeArmBoneIndex);
		endEffector[leftFoot] = new EndEffector(leftFootBoneIndex);
		endEffector[rightFoot] = new EndEffector(rightFootBoneIndex);
		endEffector[leftKnee] = new EndEffector(leftLegBoneIndex);
		endEffector[rightKnee] = new EndEffector(rightLegBoneIndex);
		initTransAndRot();
		
#ifdef KORE_STEAMVR
		VrInterface::init(nullptr, nullptr, nullptr); // TODO: Remove
#endif
	}
}

int kickstart(int argc, char** argv) {
	System::init("BodyTracking", width, height);
	
	init();
	
	System::setCallback(update);
	
	startTime = System::time();
	
	Keyboard::the()->KeyDown = keyDown;
	Keyboard::the()->KeyUp = keyUp;
	Mouse::the()->Move = mouseMove;
	Mouse::the()->Press = mousePress;
	Mouse::the()->Release = mouseRelease;
	
	// Sound initiation
	Audio1::init();
	Audio2::init();
	startRecordingSound = new Sound("sound/start.wav");
	stopRecordingSound = new Sound("sound/stop.wav");
	
	System::start();
	
	return 0;
}
