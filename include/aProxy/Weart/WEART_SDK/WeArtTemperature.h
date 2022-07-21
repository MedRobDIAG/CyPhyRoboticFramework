/**
*	WEART - Temperature component
*	https://www.weart.it/
*/
#pragma once
#include <WEART_SDK/WeArtCommon.h>

struct WeArtTemperature {
private:
	float _value;
public:
	bool active;

	float value() const { return _value; }

	void value(float v) {
		float lower = 0.0f;
		float upper = 1.0f;
		_value = v <= lower ? lower : v >= upper ? upper : v;
	}

	WeArtTemperature() : active(false), _value(WeArtConstants::defaultTemperature) {};

	bool operator==(const WeArtTemperature& other) const {
		return (active == other.active && _value == other.value());
	};
};


