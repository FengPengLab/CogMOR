//
// Created by fengpeng on 3/14/17.
//

#ifndef __mac_cogorm_timers_h__
#define __mac_cogorm_timers_h__
#include <list>
#include "mac.h"
#include "../common/scheduler.h"
#include "../tools/random.h"


/* ======================================================================
   Timers
   ====================================================================== */
class MacCogMOR;

class MacCogMORTimer : public Handler {
public:
	MacCogMORTimer(MacCogMOR* m) : mac(m) {
		busy_ = paused_ = 0; stime = rtime = 0.0;
	}

    virtual void handle(Event *e) = 0;

    virtual void start(double time);
    virtual void stop(void);
    virtual void pause(void) { assert(0); }
    virtual void resume(void) { assert(0); }

    inline int busy(void) { return busy_; }
    inline int paused(void) { return paused_; }
    inline double expire(void) {
        return ((stime + rtime) - Scheduler::instance().clock());
    }
    inline double past(void) {
        return (Scheduler::instance().clock() - stime);
    }

protected:
    MacCogMOR	*mac;
    int		busy_;
    int		paused_;
    Event		intr;
    double		stime;	// start time
    double		rtime;	// remaining time
};

class MacCogMORBackoffTimer : public MacCogMORTimer {
public:
    MacCogMORBackoffTimer(MacCogMOR *m) : MacCogMORTimer(m) {
        difs_wait = 0;
    }
    void	handle(Event *e);
    void	pause(void);
    void	resume(double difs);
private:
    double	difs_wait;
};

class MacCogMORDeferTimer : public MacCogMORTimer {
public:
    MacCogMORDeferTimer(MacCogMOR *m) : MacCogMORTimer(m) {}

    void	handle(Event *e);
};

class MacCogMORTimeoutTimer : public MacCogMORTimer {
public:
    MacCogMORTimeoutTimer(MacCogMOR *m) : MacCogMORTimer(m) {}

    void	handle(Event *e);
};

class MacCogMORUIFSTimer : public MacCogMORTimer {
public:
    MacCogMORUIFSTimer(MacCogMOR *m) : MacCogMORTimer(m) {}

    void	handle(Event *e);
};

class MacCogMORBIFSTimer : public MacCogMORTimer {
public:
    MacCogMORBIFSTimer(MacCogMOR *m) : MacCogMORTimer(m) {}

    void	handle(Event *e);
};

class MacCogMORStageTimer : public MacCogMORTimer {
public:
    MacCogMORStageTimer(MacCogMOR *m) : MacCogMORTimer(m) {}

    void    start(double time);
    void	handle(Event *e);
};

class MacCogMORReplyTimer : public MacCogMORTimer {
public:
    MacCogMORReplyTimer(MacCogMOR *m) : MacCogMORTimer(m) {}

    void	handle(Event *e);
};

class MacCogMORRxTimer : public MacCogMORTimer {
public:
	MacCogMORRxTimer(MacCogMOR *m) : MacCogMORTimer(m) {}

	void	handle(Event *e);
};

class MacCogMORTxTimer : public MacCogMORTimer {
public:
	MacCogMORTxTimer(MacCogMOR *m) : MacCogMORTimer(m) {}

	void	handle(Event *e);
};

class MacCogMORPuTimer : public MacCogMORTimer {
public:
    MacCogMORPuTimer(MacCogMOR *m) : MacCogMORTimer(m) {}

    void	handle(Event *e);
};

class MacCogMORCavTimer : public MacCogMORTimer {
public:
    MacCogMORCavTimer(int chan,int active, double cav, MacCogMOR *m) : MacCogMORTimer(m) {
        channel_ = chan;
        active_ = active;
        cav_ = cav;
    }
    void start(double time, double cav);
    void stop();
    void handle(Event *e);
    void handle();
    void setChannel(int chan){ channel_ = chan; }
    void setActive(int a) { active_ = a; }
    void setCav(double d) { cav_ = d; }
    int getChannel(void) { return channel_; }
    int getActive() { return active_; }
    double getCav() { return cav_; }
private:
    int active_;
    int channel_;
    double cav_;
};

class MacCogMORCAVList {
public:
    MacCogMORCAVList() { }
    int findCAV(int channel);
    void stopCAV(int channel);
    void startCAV(int channel, double time, double timer);
    double getCAV(int channel);
    double getExpire(int channel);
    std::list<MacCogMORCavTimer> cav_list;
};

#endif /* __mac_timers_h__ */

