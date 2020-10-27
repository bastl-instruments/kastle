//
//  ADRS.h
//
//  Created by Nigel Redmon on 12/18/12.
//  EarLevel Engineering: earlevel.com
//  Copyright 2012 Nigel Redmon
//
//  For a complete explanation of the ADSR envelope generator and code,
//  read the series of articles by the author, starting here:
//  http://www.earlevel.com/main/2013/06/01/envelope-generators/
//
//  License:
//
//  This source code is provided as is, without warranty.
//  You may copy and distribute verbatim copies of this document.
//  You may modify and use this source code to create binary code for your own purposes, free or commercial.
//

#ifndef EXP_ADRS_h
#define EXP_ADRS_h


class ADSR {
public:
	ADSR(void);
	~ADSR(void);
	float process(void);
    float getOutput(void);
    int getState(void);
	void gate(int on);
	void reTrigger();
    void setAttackRate(float rate);
    void setHoldRate(long rate){ hold_value=rate;};
    void setDecayRate(float rate);
    void setReleaseRate(float rate);
	void setSustainLevel(float level);
    void setTargetRatioA(float targetRatio);
    void setTargetRatioDR(float targetRatio);
    void reset(void);
    void setHold(bool _hold);
    void sync();
    void setLoop(bool _loop){ looping=_loop;};

    enum envState {
        env_idle = 0,
        env_attack,
        env_decay,
        env_sustain,
        env_release,
        env_hold
    };

protected:
    bool gateState;
    bool hold;
    bool looping;
	int state;
	long hold_count;
	long hold_value;
	float output;
	float attackRate;
	float decayRate;
	float releaseRate;
	float attackCoef;
	float decayCoef;
	float releaseCoef;
	float sustainLevel;
    float targetRatioA;
    float targetRatioDR;
    float attackBase;
    float decayBase;
    float releaseBase;
 
    float calcCoef(float rate, float targetRatio);
};

inline float ADSR::process() {
	switch (state) {
		case env_hold:
			hold_count++;
			if(hold_count>=hold_value){
				hold_count=0;
				if(looping) state = env_release;
				else state = env_decay;
			}

			output = 1.0;
	            break;
        case env_idle:
        	if(looping){
        		state = env_attack;
        	}
            break;
        case env_attack:
            output = attackBase + output * attackCoef;
            if (output >= 1.0) {
                output = 1.0;
               if(hold) state = env_hold, hold_count=0;
               else if(looping) state = env_release;
               else state = env_decay;
            }
            break;
        case env_decay:
            output = decayBase + output * decayCoef;
            if (output <= sustainLevel) {
                output = sustainLevel;
                state = env_sustain;
            }
            break;
        case env_sustain:
        	if(sustainLevel<=0.0) state = env_idle;
            break;
        case env_release:
            output = releaseBase + output * releaseCoef;
            if (output <= 0.0) {
                output = 0.0;
                if(looping) state = env_attack;
                else state = env_idle;
            }
	}
	return output;
}

inline void ADSR::gate(int gate) {
	if (gate) gateState=true, state = env_attack;
	else if (state != env_idle){
		if(!hold){
			state = env_release;
			gateState=false;
		}
	}
}

inline int ADSR::getState() {
    return state;
}

inline void ADSR::reset() {
    state = env_idle;
    output = 0.0;
}

inline float ADSR::getOutput() {
	return output;
}

#endif
