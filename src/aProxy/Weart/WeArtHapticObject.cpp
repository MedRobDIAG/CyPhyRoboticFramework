/**
*	WEART - HapticObject component
*	https://www.weart.it/
*/


#include "pch.h"
#include "WeArtHapticObject.h"
#include "WeArtController.h"
#include <algorithm>

WeArtHapticObject::WeArtHapticObject(WeArtClient* client)
{
	weArtclient = client;
}


void WeArtHapticObject::AddEffect(WeArtEffect* effect)
{
	if (!ContainsEffect(effect))
	{
		activeEffects.push_back(effect);
		UpdateEffects();
	}
}


void WeArtHapticObject::RemoveEffect(WeArtEffect* effect)
{
	if (ContainsEffect(effect))
	{
		activeEffects.erase(std::remove(activeEffects.begin(), activeEffects.end(), effect), activeEffects.end());
		UpdateEffects();
	}
}


bool WeArtHapticObject::ContainsEffect(WeArtEffect* effect)
{
	return std::find(activeEffects.begin(), activeEffects.end(), effect) != activeEffects.end();
}


void WeArtHapticObject::UpdateEffects(void)
{
	if (activeEffects.empty()) {
		weArtForce.value(WeArtConstants::defaultForce);
		weArtForce.active = false;
		weArtTemperature.value(WeArtConstants::defaultTemperature);
		weArtTemperature.active = false;
		weArtTexture.textureType((TextureType)WeArtConstants::defaultTextureIndex);
		weArtTexture.textureVelocity(WeArtConstants::defaultTextureVelocity[0], WeArtConstants::defaultTextureVelocity[1], WeArtConstants::defaultTextureVelocity[2]);
		weArtTexture.active = false;

		StopTemperatureMessage msg1;
		SendMessage(&msg1);
		StopForceMessage msg2;
		SendMessage(&msg2);
		StopTextureMessage msg3;
		SendMessage(&msg3);
	}
	else {
		// Update temperature and send relevant messages
		WeArtEffect* tempEffect = nullptr;
		for (std::vector<WeArtEffect*>::reverse_iterator i = activeEffects.rbegin(); i != activeEffects.rend(); ++i) {
			if ((*i)->getTemperature().active)
			{
				tempEffect = *i;
				break;
			}
		}
		WeArtTemperature newTemp = tempEffect != nullptr ? tempEffect->getTemperature() : WeArtTemperature();
		if (!(newTemp == weArtTemperature)) {
			if (!newTemp.active) {
				StopTemperatureMessage msg;
				SendMessage(&msg);
			}
			else {
				SetTemperatureMessage msg(newTemp.value());
				SendMessage(&msg);
			}
			weArtTemperature = newTemp;
		}

		// Update force and set relevant messages
		WeArtEffect* forceEffect = nullptr;
		for (std::vector<WeArtEffect*>::reverse_iterator i = activeEffects.rbegin(); i != activeEffects.rend(); ++i) {
			if ((*i)->getForce().active)
			{
				forceEffect = *i;
				break;
			}
		}
		WeArtForce newForce = forceEffect != nullptr ? forceEffect->getForce() : WeArtForce();
		if (!(newForce == weArtForce)) {
			if (!newForce.active) {
				StopForceMessage msg;
				SendMessage(&msg);
			}
			else {
				const float fValue[3] = { newForce.value(), 0.0, 0.0 };
				SetForceMessage msg(fValue);
				SendMessage(&msg);
			}
			weArtForce = newForce;
		}

		// Update texture and set relevant messages
		WeArtEffect* texEffect = nullptr;
		for (std::vector<WeArtEffect*>::reverse_iterator i = activeEffects.rbegin(); i != activeEffects.rend(); ++i) {
			if ((*i)->getTexture().active)
			{
				texEffect = *i;
				break;
			}
		}
		WeArtTexture newTex = texEffect != nullptr ? texEffect->getTexture() : WeArtTexture();
		if (!(newTex == weArtTexture)) {
			if (!newTex.active) {
				StopTextureMessage msg;
				SendMessage(&msg);
			}
			else {
				const float texValue[3] = { newTex.textureVelocity()[0], newTex.textureVelocity()[1], newTex.textureVelocity()[2] };
				SetTextureMessage msg((int)newTex.textureType(), texValue, newTex.volume());
				SendMessage(&msg);
			}
			weArtTexture = newTex;
		}
	}
}


void WeArtHapticObject::SendMessage(WeArtMessage* msg)
{
	const HandSide hand_sides[] = { HandSide::Left, HandSide::Right };
	const ActuationPoint actuation_points[] = { ActuationPoint::Thumb, ActuationPoint::Index, ActuationPoint::Middle, ActuationPoint::Palm };

	for (HandSide hs : hand_sides) {
		if (hs & handSideFlag) {
			for (ActuationPoint ap : actuation_points) {
				if (ap & actuationPointFlag) {
					msg->setHandSide(hs);
					msg->setActuationPoint(ap);
					weArtclient->SendMessage(msg);
				}
			}
		}
	}
}
