//
// Created by fengpeng on 6/24/17.
//

#ifndef NS_2_35_MAC_SHCS_TIMER_H
#define NS_2_35_MAC_SHCS_TIMER_H
#include <list>
#include "mac.h"
#include "../common/scheduler.h"
#include "../tools/random.h"
/* ======================================================================
   Timers
   ====================================================================== */
class MacSHCS;

class MacSHCSTimer : public Handler {
public:
    MacSHCSTimer(MacSHCS* m) : mac(m) {
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

protected:
    MacSHCS	*mac;
    Event intr;

    int busy_;
    int paused_;

    double stime;	// start time
    double rtime;	// remaining time
};


class MacSHCSBackoffTimer : public MacSHCSTimer {
public:
    MacSHCSBackoffTimer(MacSHCS *m) : MacSHCSTimer(m) {
        difs_wait = 0;
    }
    void	start(int cw, int idle, double difs = 0.0);
    void    start(int idle, double time);
    void	handle(Event *e);
    void	pause(void);
    void	resume(double difs);
private:
    double	difs_wait;
};

class MacSHCSIFTimer : public MacSHCSTimer {
public:
    MacSHCSIFTimer(MacSHCS *m) : MacSHCSTimer(m) {}

    void	handle(Event *e);
    void    setTimeOut(double t) { timeout = t; }
    double timeout;
};

class MacSHCSDeferTimer : public MacSHCSTimer {
public:
    MacSHCSDeferTimer(MacSHCS *m) : MacSHCSTimer(m) {}

    void	handle(Event *e);
};

class MacSHCSReplyTimer : public MacSHCSTimer {
public:
    MacSHCSReplyTimer(MacSHCS *m) : MacSHCSTimer(m) {}

    void	handle(Event *e);
};

class MacSHCSRxTimer : public MacSHCSTimer {
public:
    MacSHCSRxTimer(MacSHCS *m) : MacSHCSTimer(m) {}

    void	handle(Event *e);
};

class MacSHCSTxTimer : public MacSHCSTimer {
public:
    MacSHCSTxTimer(MacSHCS *m) : MacSHCSTimer(m) {}

    void	handle(Event *e);
};

class MacSHCSPhaseTimer : public MacSHCSTimer {
public:
    MacSHCSPhaseTimer(MacSHCS *m) : MacSHCSTimer(m) {}

    void	handle(Event *e);
};

class MacSHCSPuTimer : public MacSHCSTimer {
public:
    MacSHCSPuTimer(MacSHCS *m) : MacSHCSTimer(m) {}

    void	handle(Event *e);
};

class MacSHCSCavTimer : public MacSHCSTimer {
public:
    MacSHCSCavTimer(int chan,int active, double cav, MacSHCS *m) : MacSHCSTimer(m) {
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

class MacSHCSCAVList {
public:
    MacSHCSCAVList() { }
    int findCAV(int channel);
    void stopCAV(int channel);
    void startCAV(int channel, double time, double timer);
    double getCAV(int channel);
    double getExpire(int channel);
    std::list<MacSHCSCavTimer> cav_list;
};

#endif //NS_2_35_MAC_SHCS_TIMER_H
