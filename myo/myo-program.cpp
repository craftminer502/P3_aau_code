#include <array>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <myo/myo.hpp>
#include "SerialPort.h"

class PoseListener : public myo::DeviceListener {
public:

	void onPose(myo::Myo *myo, uint64_t timestamp, myo::Pose pose) {
		std::string poseString = pose.toString();
		if(!(poseString == lastPose)) {
			lastPose = poseString;
			poseUpdated = true;
		}
	}

	std::string lastPose = "";
	bool poseUpdated;
};

int main(int argc, char** argv) {
    try {
		myo::Hub hub("third.project.myo");

		std::cout << "Searching for Myo..." << std::endl;

		myo::Myo* myo = hub.waitForMyo(5000);

		if(!myo) {
			throw std::runtime_error("No Myo found ;(");
		}

		std::cout << "Myo connection established :D" << std::endl << std::endl;

		PoseListener poseListener;

		hub.addListener(&poseListener);

		char *port_name = "\\\\.\\COM7";

		std::cout << "Searching for Arduino..." << std::endl;

		SerialPort arduino(port_name);

		if (arduino.isConnected()) {
			std::cout << "Arduino cnnection Established :D" << std::endl;
		} else {
			std::cout << "No Arduino found ;(";
		}

		while(true) {
			hub.run(50);

			if(arduino.isConnected() && poseListener.poseUpdated) {
				char *cs = new char[poseListener.lastPose.size() + 1];
				std::copy(poseListener.lastPose.begin(), poseListener.lastPose.end(), cs);
				cs[poseListener.lastPose.size()] = '\n';
				arduino.writeSerialPort(cs, MAX_DATA_LENGTH);
				poseListener.poseUpdated = false;
			}
		}

    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        return 1;
    }
}
