#ifndef STATECONTROL_H
#define STATECONTROL_H

/** Variables and structs for loading parsed xml-file of structure **/
struct sc_ActionStruct
{
	tBool enabled;
	tBool started;
    tUInt32 command;
};

struct sc_Filter_active
{
	tUInt32 filterID;
	sc_ActionStruct action;
};

struct sc_Request{
	tUInt32 request; //statusIN
	tUInt32 command;	
};

struct sc_Filter_passive
{
	tUInt32 filterID;
	std::vector<sc_Request> Requests;
};

struct sc_active
{
   	std::vector<sc_Filter_active> FilterList;
};

struct sc_passive
{
    std::vector<sc_Filter_passive> FilterList;
};

struct sc_ActivityLevel
{
	sc_active active;
	sc_passive passive;
};

struct sc_Step
{
	tUInt32 id;
    sc_ActivityLevel activityLvl;
};

struct sc_Maneuver
{
	tUInt32 id;
    std::vector<sc_Step> Steps;
};

/** Variables and structs for managing actual state in StateController **/

enum actActivityLevel{
	NOT_INITIALIZED = -1,
    ACTIVE = 1,
    PASSIVE = 2
};



#endif // STATECONTROL_H
