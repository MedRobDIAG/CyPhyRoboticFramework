#pragma once
#include <WEART_SDK/WeArtEffect.h>

class TouchEffect : public WeArtEffect {

public:
	WeArtTemperature effTemperature;
	WeArtForce effForce;
	WeArtTexture effTexture;


	TouchEffect(WeArtTemperature temp, WeArtForce force, WeArtTexture texture)
	{
		effTemperature = temp;
		effForce = force;
		effTexture = texture;
	};

	bool Set(WeArtTemperature temp, WeArtForce force, WeArtTexture texture);

	virtual WeArtTemperature	getTemperature(void)	override { return effTemperature; };
	virtual WeArtForce			getForce(void)	override { return effForce; };
	virtual WeArtTexture		getTexture(void)	override { return effTexture; };

};

