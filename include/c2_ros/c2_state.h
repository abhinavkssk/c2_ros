#ifndef C2_STATE_H_
#define C2_STATE_H_

namespace C2 {


enum class C2_STATE {
	INIT = 0,
	RUN,	
	STANDBY,
	ABORT,
	ERROR
};

static const char* C2_StateName[] = { "INIT", "RUN", "STANDBY", "ABORT" };

}
#endif //C2_STATE_H_
