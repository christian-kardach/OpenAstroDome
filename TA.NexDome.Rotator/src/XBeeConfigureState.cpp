
#include "XBeeConfigureState.h"
#include "XBeeApiDetectShutterState.h"
#include "XBeeWaitForCommandModeState.h"

void XBeeConfigureState::OnTimerExpired()
	{
#ifdef DEBUG_XBEE_CONFIG
	std::cout << " timeout" << std::endl;
#endif
	machine.ChangeState(new XBeeWaitForCommandModeState(machine));
	}

void XBeeConfigureState::OnEnter()
	{
	sendNextAtCommand();
	}

bool XBeeConfigureState::sendNextAtCommand()
{
	static std::string message;
	message.clear();
	message.append("AT");
	while (true)
	{
		auto ch = initSequence[index++];
		if (ch == 0) return false;
		if (ch == ',')
		{

#ifdef DEBUG_XBEE_CONFIG
			std::cout << message;
#endif
			delay(XbeeInterAtCommandDelay);
			message.push_back('\r');
			machine.sendToLocalXbee(message);
			timer.SetDuration(XBEE_AT_COMMAND_TIMEOUT);
			return true;
		}
		message.push_back(ch);
	}
}

void XBeeConfigureState::OnSerialLineReceived(const std::string& message)
	{
#ifdef DEBUG_XBEE_CONFIG
	std::cout << " - " << message << std::endl;
#endif
	if (message == "OK")
		{
		if (!sendNextAtCommand())
			machine.ChangeState(new XBeeApiDetectShutterState(machine));
		return;
		}
	// Any response but "OK" causes the configuration process to be restarted from scratch
	machine.ChangeState(new XBeeWaitForCommandModeState(machine));
	}

