/**
*	WEART - Message serializer
*	https://www.weart.it/
*/
#pragma once

#include "WeArtMessages.h"
#include <sstream>

class WeArtMessageSerializer
{

public:
	const char separator = ':';


	static constexpr unsigned long long int HashStringToInt(const char* str, unsigned long long int hash = 0) {
		return (*str == 0) ? hash : 101 * HashStringToInt(str + 1) + *str;
	}

	static WeArtMessage* createMessage(std::string& ID) {
		switch (HashStringToInt(ID.c_str()))
		{
		case HashStringToInt("StartFromClient"): {
			WeArtMessage* msg = new StartFromClientMessage();
			return msg;
			break;
		}

		case HashStringToInt("StopFromClient"): {
			WeArtMessage* msg = new StopFromClientMessage();
			return msg;
			break;
		}
		case HashStringToInt("exit"): {
			WeArtMessage* msg = new ExitMessage();
			return msg;
			break;
		}
		case HashStringToInt("disconnect"): {
			WeArtMessage* msg = new DisconnectMessage();
			return msg;
			break;
		}
		case HashStringToInt("temperature"): {
			WeArtMessage* msg = new SetTemperatureMessage();
			return msg;
			break;
		}
		case HashStringToInt("stopTemperature"): {
			WeArtMessage* msg = new StopTemperatureMessage();
			return msg;
			break;
		}
		case HashStringToInt("force"): {
			WeArtMessage* msg = new SetForceMessage();
			return msg;
			break;
		}
		case HashStringToInt("stopForce"): {
			WeArtMessage* msg = new StopForceMessage();
			return msg;
			break;
		}
		case HashStringToInt("texture"): {
			WeArtMessage* msg = new SetTextureMessage();
			return msg;
			break;
		}
		case HashStringToInt("stopTexture"): {
			WeArtMessage* msg = new StopTextureMessage();
			return msg;
			break;
		}
		case HashStringToInt("Tracking"): {
			WeArtMessage* msg = new TrackingMessage();
			return msg;
			break;
		}

		default:
			return nullptr;
			break;
		}
	};


	// WeArtMessage to std::string.
	std::string Serialize(WeArtMessage* message)
	{
		// Unlike C#, C++ does not allow dynamic runtime
		// reflection, hence we have to use dedicated functions
		// to get the values of the class members.
		std::string messageID = message->getID();
		std::vector<std::string> serializedValues = message->getValues();

		// Join the string arrays.
		serializedValues.insert(serializedValues.begin(), messageID);

		// Then merge using our separator.
		std::stringstream ss;
		auto it = serializedValues.begin();
		ss << *it++;
		for (; it != serializedValues.end(); it++) {
			ss << separator;
			ss << *it;
		}

		return ss.str();
	};

	// Std::string to bytestream.
	uint8* Serialize(std::string text)
	{
		// Simple reinterpret cast, not even necessary
		// to allocate new memory.
		uint8* byteData = reinterpret_cast<uint8*>(&text[0]);
		return byteData;
	}

	// Std::string to message class
	WeArtMessage* Deserialize(std::string data)
	{
		std::vector<std::string> strings;
		std::istringstream dataStream(data);
		std::string s;
		while (getline(dataStream, s, separator)) {
			strings.push_back(s);
		}

		// Create the correct message class from first (ID) string.
		std::string messageID = strings[0];
		WeArtMessage *msg = createMessage(messageID);

		// Initialize the message values using the rest of the strings.
		if (msg) {
			strings.erase(strings.begin());
			msg->setValues(strings);
		}

		return msg;
	}

	// Bytestream to std::string.
	std::string Deserialize(uint8* byteData, int byteCount)
	{
		std::string text = std::string(reinterpret_cast<char*>(byteData));
		return text;
	}
};
