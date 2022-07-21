/**
*	WEART - Message composer
*	https://www.weart.it/
*/
#pragma once

#include "WeArtCommon.h"
#include <string>
#include <vector>
#include <cassert>

static HandSide StringToHandside(std::string &str) {
	if (str == "LEFT") {
		return HandSide::Left;
	}
	else if (str == "RIGHT") {
		return HandSide::Right;
	}
	else {
		assert(false);
		return HandSide::Left;
	}
};


static std::string HandsideToString(HandSide hs) {
	if (hs == HandSide::Left) {
		return "LEFT";
	}
	else if (hs == HandSide::Right) {
		return "RIGHT";
	}
	else {
		assert(false);
		return "";
	}
};


static ActuationPoint StringToActuationPoint(std::string& str) {
	if (str == "THUMB") {
		return ActuationPoint::Thumb;
	}
	else if (str == "INDEX") {
		return ActuationPoint::Index;
	}
	else if (str == "MIDDLE") {
		return ActuationPoint::Middle;
	}
	else if (str == "PALM") {
		return ActuationPoint::Palm;
	}
	else {
		assert(false);
		return ActuationPoint::Thumb;
	}
};


static std::string ActuationPointToString(ActuationPoint ap) {
	if (ap == ActuationPoint::Thumb) {
		return "THUMB";
	}
	else if (ap == ActuationPoint::Index) {
		return "INDEX";
	}
	else if (ap == ActuationPoint::Middle) {
		return "MIDDLE";
	}
	else if (ap == ActuationPoint::Palm) {
		return "PALM";
	}
	else {
		assert(false);
		return "";
	}
};

class WeArtMessage {

public:
	virtual std::string getID() = 0;
	virtual std::vector<std::string> getValues() = 0;
	virtual void setValues(std::vector<std::string>& values) = 0;

	virtual void setHandSide(HandSide hs) = 0;
	virtual void setActuationPoint(ActuationPoint ap) = 0;
};


class WeArtMessageNoParams : public WeArtMessage {

public:
	virtual void setHandSide(HandSide hs) override {};
	virtual void setActuationPoint(ActuationPoint ap) override {};

	// This message carries no values.
	virtual std::vector<std::string> getValues() override { return std::vector<std::string>(); };
	virtual void setValues(std::vector<std::string>& values) override {};
};


class WeArtMessageObjectSpecific : public WeArtMessage {

protected:
	HandSide handSide;
	ActuationPoint actuationPoint;
public:
	virtual void setHandSide(HandSide hs) override { handSide = hs; };
	virtual void setActuationPoint(ActuationPoint ap) override { actuationPoint = ap; };
};


class StartFromClientMessage : public WeArtMessageNoParams {

public:
	virtual std::string getID() override { return std::string("StartFromClient"); };
};

class StopFromClientMessage : public WeArtMessageNoParams {

public:
	virtual std::string getID() override { return std::string("StopFromClient"); };
};

class ExitMessage : public WeArtMessageNoParams {

public:
	virtual std::string getID() override { return std::string("exit"); };
};

class DisconnectMessage : public WeArtMessageNoParams {

public:
	virtual std::string getID() override { return std::string("disconnect"); };
};

class SetTemperatureMessage : public WeArtMessageObjectSpecific
{
public:
	SetTemperatureMessage(const float t) : temperature(t) {};
	SetTemperatureMessage() {};
	virtual std::string getID() override { return std::string("temperature"); };

	virtual std::vector<std::string> getValues() override {
		std::vector<std::string> ret;
		ret.push_back(std::to_string(temperature));
		ret.push_back(HandsideToString(handSide));
		ret.push_back(ActuationPointToString(actuationPoint));
		return ret;
	};

	virtual void setValues(std::vector<std::string>& values) override {
		assert(values.size() == 3);
		temperature = std::stof(values[0]);
		handSide = StringToHandside(values[1]);
		actuationPoint = StringToActuationPoint(values[2]);
	};

protected:

	float temperature;
};

class StopTemperatureMessage : public WeArtMessageObjectSpecific
{

public:
	virtual std::string getID() override { return std::string("stopTemperature"); };

	virtual std::vector<std::string> getValues() override {
		std::vector<std::string> ret;
		ret.push_back(HandsideToString(handSide));
		ret.push_back(ActuationPointToString(actuationPoint));
		return ret;
	};

	virtual void setValues(std::vector<std::string>& values) override {
		assert(values.size() == 2);
		handSide = StringToHandside(values[0]);
		actuationPoint = StringToActuationPoint(values[1]);
	};
};

class SetForceMessage : public WeArtMessageObjectSpecific
{

public:
	SetForceMessage(const float f[3]) : force{ f[0], f[1], f[2] } {};
	SetForceMessage() {};
	virtual std::string getID() override { return std::string("force"); };

	virtual std::vector<std::string> getValues() override {
		std::vector<std::string> ret;
		ret.push_back(std::to_string(force[0]));
		ret.push_back(std::to_string(force[1]));
		ret.push_back(std::to_string(force[2]));
		ret.push_back(HandsideToString(handSide));
		ret.push_back(ActuationPointToString(actuationPoint));
		return ret;
	};

	virtual void setValues(std::vector<std::string>& values) override {
		assert(values.size() == 5);
		force[0] = std::stof(values[0]);
		force[1] = std::stof(values[1]);
		force[2] = std::stof(values[2]);
		handSide = StringToHandside(values[3]);
		actuationPoint = StringToActuationPoint(values[4]);
	};

protected:

	float force[3];
};

class StopForceMessage : public WeArtMessageObjectSpecific
{

public:
	virtual std::string getID() override { return std::string("stopForce"); };

	virtual std::vector<std::string> getValues() override {
		std::vector<std::string> ret;
		ret.push_back(HandsideToString(handSide));
		ret.push_back(ActuationPointToString(actuationPoint));
		return ret;
	};

	virtual void setValues(std::vector<std::string>& values) override {
		assert(values.size() == 2);
		handSide = StringToHandside(values[0]);
		actuationPoint = StringToActuationPoint(values[1]);
	};
};

class SetTextureMessage : public WeArtMessageObjectSpecific
{

public:
	SetTextureMessage(const int idx, const float vel[3], const float vol) : index(idx), velocity{ vel[0], vel[1], vel[2] }, volume(vol)  {};
	SetTextureMessage() {};
	virtual std::string getID() override { return std::string("texture"); };

	virtual std::vector<std::string> getValues() override {
		std::vector<std::string> ret;

		// avoinding texture id out of bounds
		if (index < WeArtConstants::minTextureIndex || index > WeArtConstants::maxTextureIndex) {
			index = WeArtConstants::nullTextureIndex;
		}

		ret.push_back(std::to_string(index));
		ret.push_back(std::to_string(velocity[0]));
		ret.push_back(std::to_string(velocity[1]));
		ret.push_back(std::to_string(velocity[2]));
		ret.push_back(std::to_string(volume));
		ret.push_back(HandsideToString(handSide));
		ret.push_back(ActuationPointToString(actuationPoint));
		return ret;
	};

	virtual void setValues(std::vector<std::string>& values) override {
		assert(values.size() == 6);
		index = std::stoi(values[0]);
		velocity[0] = std::stof(values[1]);
		velocity[1] = std::stof(values[2]);
		velocity[2] = std::stof(values[3]);
		volume = std::stof(values[4]);
		handSide = StringToHandside(values[5]);
		actuationPoint = StringToActuationPoint(values[6]);
	};

protected:

	int index;
	float velocity[3];
	float volume;
};

class StopTextureMessage : public WeArtMessageObjectSpecific
{

public:
	virtual std::string getID() override { return std::string("stopTexture"); };

	virtual std::vector<std::string> getValues() override {
		std::vector<std::string> ret;
		ret.push_back(HandsideToString(handSide));
		ret.push_back(ActuationPointToString(actuationPoint));
		return ret;
	};

	virtual void setValues(std::vector<std::string>& values) override {
		assert(values.size() == 2);
		handSide = StringToHandside(values[0]);
		actuationPoint = StringToActuationPoint(values[1]);
	};
};


class TrackingMessage : public WeArtMessageNoParams
{
	uint8 RightThumbClosure;
	uint8 RightIndexClosure;
	uint8 RightMiddleClosure;
	uint8 RightPalmClosure;
	uint8 LeftThumbClosure;
	uint8 LeftIndexClosure;
	uint8 LeftMiddleClosure;
	uint8 LeftPalmClosure;

public:

	virtual std::string getID() override { return std::string("Tracking"); };

	virtual std::vector<std::string> getValues() override {
		std::vector<std::string> ret;
		ret.push_back(std::to_string(RightThumbClosure));
		ret.push_back(std::to_string(RightIndexClosure));
		ret.push_back(std::to_string(RightMiddleClosure));
		ret.push_back(std::to_string(RightPalmClosure));
		ret.push_back(std::to_string(LeftThumbClosure));
		ret.push_back(std::to_string(LeftIndexClosure));
		ret.push_back(std::to_string(LeftMiddleClosure));
		ret.push_back(std::to_string(LeftPalmClosure));
		return ret;
	};

	virtual void setValues(std::vector<std::string>& values) override {
		assert(values.size() == 8);
		RightThumbClosure = std::stof(values[0]);
		RightIndexClosure = std::stof(values[1]);
		RightMiddleClosure = std::stof(values[2]);
		RightPalmClosure = std::stof(values[3]);
		LeftThumbClosure = std::stof(values[4]);
		LeftIndexClosure = std::stof(values[5]);
		LeftMiddleClosure = std::stof(values[6]);
		LeftPalmClosure = std::stof(values[7]);
	};

	float GetClosure(HandSide handSide, ActuationPoint actuationPoint) {
		uint8 byteValue = 0x00;
		switch (handSide)
		{
		case HandSide::Left:
			switch (actuationPoint)
			{
			case ActuationPoint::Thumb:  byteValue = LeftThumbClosure;  break;
			case ActuationPoint::Index:  byteValue = LeftIndexClosure;  break;
			case ActuationPoint::Middle: byteValue = LeftMiddleClosure; break;
			case ActuationPoint::Palm:   byteValue = LeftPalmClosure;   break;
			}
			break;
		case HandSide::Right:
			switch (actuationPoint)
			{
			case ActuationPoint::Thumb:  byteValue = RightThumbClosure;  break;
			case ActuationPoint::Index:  byteValue = RightIndexClosure;  break;
			case ActuationPoint::Middle: byteValue = RightMiddleClosure; break;
			case ActuationPoint::Palm:   byteValue = RightPalmClosure;   break;
			}
			break;
		}

		float num(byteValue);
		float denom(255);
		float closure = num / denom;

		return closure;
	}
};
