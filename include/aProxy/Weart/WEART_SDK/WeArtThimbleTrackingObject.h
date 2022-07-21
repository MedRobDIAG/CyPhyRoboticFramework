/**
*	WEART - Tracking thimble component
*	https://www.weart.it/
*/
#pragma once

#include "WeArtCommon.h"
#include "WeArtMessages.h"

class WeArtThimbleTrackingObject
{

public:

	WeArtThimbleTrackingObject(HandSide handSide, ActuationPoint actuationPoint);

	void OnMessageReceived(WeArtMessage *msg);

	float GetClosure() const { return Closure; }


protected:

	HandSide handSide;

	ActuationPoint actuationPoint;
	
	float Closure;
};

