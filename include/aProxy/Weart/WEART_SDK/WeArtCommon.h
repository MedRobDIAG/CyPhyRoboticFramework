/**
*	WEART - Common utility 
*	https://www.weart.it/
*/

#pragma once
#include <vector>

enum HandSide
{
	//HSnone = 0	
	Left	= 1 << 0,
	Right	= 1 << 1,
};


enum ActuationPoint
{
	//APnone	= 0
	Thumb	= 1 << 0,
	Index	= 1 << 1,
	Middle	= 1 << 2,
	Palm	= 1 << 3,
};


enum HandClosingState
{
	Open = 0,
	Closing = 1,
	Closed = 2
};


enum GraspingState
{
	Grabbed = 0,
	Released = 1
};


enum class TextureType : uint8
{
	ClickNormal = 0, ClickSoft = 1, DoubleClick = 2,
	AluminiumFineMeshSlow = 3, AluminiumFineMeshFast = 4,
	PlasticMeshSlow = 5, ProfiledAluminiumMeshMedium = 6, ProfiledAluminiumMeshFast = 7,
	RhombAluminiumMeshMedium = 8,
	TextileMeshMedium = 9,
	CrushedRock = 10,
	VenetianGranite = 11,
	SilverOak = 12,
	LaminatedWood = 13,
	ProfiledRubberSlow = 14,
	VelcroHooks = 15,
	VelcroLoops = 16,
	PlasticFoil2 = 17,
	Leather = 18,
	Cotton1 = 19,
	Aluminium = 20,
	DoubleSidedTape = 21
};


// Constants shared by the WeArt components
namespace WeArtConstants
{
	const char* DEFAULT_TCP_PORT = "13031";

	const float defaultTemperature = 0.5f;
	const float minTemperature = 0.0f;
	const float maxTemperature = 1.0f;

	const float defaultForce = 0.0f;
	const float minForce = 0.0f;
	const float maxForce = 1.0f;

	const float defaultClosure = 0.0f;
	const float minClosure = 0.0f;
	const float maxClosure = 1.0f;

	const int defaultTextureIndex = 0;
	const int minTextureIndex = 0;
	const int maxTextureIndex = 21;
	const int nullTextureIndex = 255;

	const float defaultTextureVelocity[3] = { 0.5f, 0.0f, 0.0f };
	const float minTextureVelocity[3] = { 0.0f, 0.0f, 0.0f };
	const float maxTextureVelocity[3] = { 1.0f, 1.0f, 1.0f };

	const float defaultCollisionMultiplier = 20.0f;
	const float minCollisionMultiplier = 0.0f;
	const float maxCollisionMultiplier = 100.0f;

	const float defaultVolumeTexture = 100.0f;
	const float minVolumeTexture = 0.0f;
	const float maxVolumeTexture = 100.0f;

	const float thresholdThumbClosure = 0.5f;
	const float thresholdIndexClosure = 0.5f;
	const float thresholdMiddleClosure = 0.5f;

	const float defaultGraspForce = 0.3f;
	const float dynamicForceSensibility = 10.0f;
};
