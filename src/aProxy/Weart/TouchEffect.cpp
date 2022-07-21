#include "pch.h"
#include "TouchEffect.h"

bool TouchEffect::Set(WeArtTemperature temp, WeArtForce force, WeArtTexture texture)
{
	bool changed = false;

	// Temperature
	changed |= !(effTemperature == temp);
	effTemperature = temp;

	// Force
	changed |= !(effForce == force);
	effForce = force;

	// Texture
	changed |= !(effTexture == texture);
	// forcing Vz texture static
	texture.textureVelocity()[2] = 0.5f;

	effTexture = texture;

	return changed;
}