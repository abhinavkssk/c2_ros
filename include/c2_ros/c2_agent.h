#ifndef C2_AGENT_H_
#define C2_AGENT_H_

#include <map>

namespace C2{

class C2Agent{

public:
	enum AgentEnum {
		CAPTAIN = 0,
		PILOT,
		SAFETY_OFFICER,
		MBHV_ABORTER,
		MBHV_ADAPTIVESAMPLER,
		MBHV_WAYPOINTER,
		MBHV_LAWNMOWER,
		MBHV_LOITER,
		AGENT_COUNT // used for counter
	};

	std::map<std::string, AgentEnum> enumMap;
	AgentEnum state_;
	std::string text_;

	C2Agent(const AgentEnum& state = C2::C2Agent::AGENT_COUNT, const std::string& text = std::string("")) : state_(state), text_(text)
	{
		enumMap = {
				{"CAPTAIN",C2::C2Agent::CAPTAIN},
				{"PILOT",C2::C2Agent::PILOT},
				{"SAFETY_OFFICER",C2::C2Agent::SAFETY_OFFICER},
				{"MBHV_ABORTER",C2::C2Agent::MBHV_ABORTER},
				{"MBHV_ADAPTIVESAMPLER",C2::C2Agent::MBHV_ADAPTIVESAMPLER},
				{"MBHV_WAYPOINTER",C2::C2Agent::MBHV_WAYPOINTER},
				{"MBHV_LAWNMOWER",C2::C2Agent::MBHV_LAWNMOWER},
				{"MBHV_LOITER",C2::C2Agent::MBHV_LOITER},
		};
	}

	  inline bool operator==(const C2Agent& rhs) const
	  {
	    return (state_ == rhs.state_) ;
	  }

	  inline bool operator==(const C2Agent::AgentEnum& rhs) const
	  {
	    return (state_ == rhs);
	  }

	  inline bool operator!=(const C2Agent::AgentEnum& rhs) const
	  {
	    return !(*this == rhs);
	  }

	  inline bool operator!=(const C2Agent& rhs) const
	  {
	    return !(*this == rhs);
	  }

	AgentEnum toEnum(std::string name)
	{
		auto search = enumMap.find(name);
		if(search != enumMap.end())
			return search->second;
		else
			return C2::C2Agent::AGENT_COUNT;
	}

	AgentEnum toEnum()
	{
		auto search = enumMap.find(toString());
		if(search != enumMap.end())
			return search->second;
		else
			return C2::C2Agent::AGENT_COUNT;
	}

	std::string toString() const
	{
		switch(state_)
		{
		case CAPTAIN:
			return "CAPTAIN";
		case PILOT:
			return "PILOT";
		case SAFETY_OFFICER:
			return "SAFETY_OFFICER";
		case MBHV_ABORTER:
			return "MBHV_ABORTER";
		case MBHV_ADAPTIVESAMPLER:
			return "MBHV_ADAPTIVESAMPLER";
		case MBHV_WAYPOINTER:
			return "MBHV_WAYPOINTER";
		case MBHV_LAWNMOWER:
			return "MBHV_LAWNMOWER";
		case MBHV_LOITER:
			return "MBHV_LOITER";
		default:
			return "BUG-UNKNOWN";
			break;
		}
		return "BUG-UNKNOWN";
	}

};

}
#endif
