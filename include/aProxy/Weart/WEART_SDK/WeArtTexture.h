/**
*	WEART - Texture component
*	https://www.weart.it/
*/
#pragma once
#include <WEART_SDK/WeArtCommon.h>

struct WeArtTexture {

private:
	float _volume;
	TextureType _textureType;
	std::vector<float> _textureVelocity;

public:

	bool active;

	std::vector<float> textureVelocity() const { return _textureVelocity; }
	void textureVelocity(float Vx, float Vy, float Vz) {
		float lower = 0.0f;
		float upper = 1.0f;
		
		float _vx = 0;
		float _vy = 0;
		float _vz = 0;

		_vx = Vx <= lower ? lower : Vx >= upper ? upper : Vx;
		_vy = Vy <= lower ? lower : Vy >= upper ? upper : Vy;
		_vz = Vz <= lower ? lower : Vz >= upper ? upper : Vz;

		_textureVelocity = { _vx, _vy, _vz };
	}
	
	TextureType textureType() const { return _textureType; }
	void textureType(TextureType v) {
		TextureType lower = TextureType::ClickNormal;
		TextureType upper = TextureType::DoubleSidedTape;
		_textureType = v <= lower ? lower : v >= upper ? upper : v;
	}

	float volume() const { return _volume; }
	void volume(float v) {
		float lower = 0.0f;
		float upper = 100.0f;
		_volume = v <= lower ? lower : v >= upper ? upper : v;
	}

	WeArtTexture() : active(false),
		_textureType((TextureType)WeArtConstants::defaultTextureIndex),
		_textureVelocity { WeArtConstants::defaultTextureVelocity[0], WeArtConstants::defaultTextureVelocity[1], WeArtConstants::defaultTextureVelocity[2] },
		_volume(WeArtConstants::defaultVolumeTexture)
	{};

	bool operator==(const WeArtTexture& other) const {
		return (active == other.active && _textureType == other.textureType() && _textureVelocity == other.textureVelocity() && _volume == other.volume());
	};
};

