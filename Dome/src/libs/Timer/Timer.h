/*
 Name:		Timer.h
 Created:	2/5/2019 1:56:49 PM
 Author:	Tim
 Editor:	http://www.visualmicro.com
*/

#ifndef _Timer_h
#define _Timer_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

/*
	A simple timer that works correctly with Arduino's 	millis() function and
	unsigned arithmetic, provided the timed interval is short compared to the
	maximum value of millis(), which is about 49 days.

	On an Arduino, millis() returns a 32-bit unsigned
	integer, with maximum value of 4,294,967,295 (2^32 - 1). This represents
	a duration of about 49 days, after which the value of millis() will wrap
	to zero. Therefore, any realistic interval can be timed
	provided it is less than "a few days" and the operation will not be affected
	by the 49-day wrap around.

	The timer does not need to be updated in order to keep time, but
	it needs to be periodically checked by calling the Expired() function.

	The timer is started by calling SetDuration() with a non-zero duration,
	after which Enabled() will return true.

	Subsequently, the timer may be re-armed with the same duration by calling
	Repeat(). Using Repeat() is a good way to time a regular repeating interval
	without introducing jitter or accumulation of timing errors, since it
	effectively maintains a fixed timing reference, whereas SetDuration()
	establishes a fresh timing reference each time it is called.

	The timer may be stopped by calling Stop() or setting the duration to zero.
	A stopped timer will never expire and Enabled() will return false.

	NOTE: Due to the nature of how millis() operates using unsigned arithmetic,
	an expired time would eventually revert to being unexpired after about
	49 days, which could produce unexpected results. Therefore, to avoid this,
	after expiry has been detected the timer should always be reset by calling
	SetDuration() or Reset(), or stopped by calling Stop().
	A stopped timer will never expire.
*/

using Duration = unsigned long;

class Timer
	{
public:
	explicit Timer();
	void SetDuration(Duration duration);
	bool Expired() const;
	Duration Elapsed() const;
	Duration Remaining() const;
	bool Enabled() const;
	void Stop();
	static constexpr Duration Milliseconds(unsigned long millis) { return Duration(millis); }
	static constexpr Duration Milliseconds(unsigned millis) { return Duration(millis); }
	static constexpr Duration Milliseconds(int millis) { return Duration(unsigned(millis)); }
	static constexpr Duration Seconds(float seconds) { return Duration(1000 * seconds); }
	static constexpr Duration Minutes(float minutes) { return Duration(Seconds(60 * minutes)); }
private:
	Duration startedAt;
	Duration interval;
	};


#endif

