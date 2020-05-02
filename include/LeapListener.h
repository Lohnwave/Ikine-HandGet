#ifndef __LeapListener_H__
#define __LeapListener_H__
#include <LeapSDK\include\Leap.h>
#include <string>
using namespace std;
using namespace Leap;
/*拇指		食指	中指	无名指		小指*/
const std::string fingerNames[] = { "Thumb", "Index", "Middle", "Ring", "Pinky" };
const std::string boneNames[] = { "Metacarpal", "Proximal", "Middle", "Distal" };

class SampleListener : public Listener {
public:
	SampleListener(const float&dis01, const float&dis02)
		:finger_distance_point01(dis01), finger_distance_point02(dis02) {}
	virtual void onInit(const Controller&);
	virtual void onConnect(const Controller&);
	virtual void onDisconnect(const Controller&);
	virtual void onExit(const Controller&);
	virtual void onFrame(const Controller&);
	virtual void onFocusGained(const Controller&);
	virtual void onFocusLost(const Controller&);
	virtual void onDeviceChange(const Controller&);
	virtual void onServiceConnect(const Controller&);
	virtual void onServiceDisconnect(const Controller&);
	virtual void onServiceChange(const Controller&);
	virtual void onDeviceFailure(const Controller&);
	virtual void onLogMessage(const Controller&, MessageSeverity severity, int64_t timestamp, const char* msg);
	virtual float getCartesianDistance(const Vector&, const Vector&);
	Vector AcqureNormal()const;
	Vector AcqureDirection()const;
	Vector AcqurePosition()const;
	float AcqureFingerDistance01()const;
	float AcqureFingerDistance02()const;
private:
	Vector position;
	Vector normal;
	Vector direction;
	float finger_distance_point01;
	float finger_distance_point02;
	Vector finger0_point; // Thumb
	Vector finger1_point; // Index
	Vector finger2_point; // Middle
};

#pragma once
#endif // !__LeapListener_H__


