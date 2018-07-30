//
// Created by fengpeng on 3/14/17.
//


// #define DEBUG
// #include <debug.h>

#include "mac-cogmor-timer.h"
#include "mac-cogmor.h"
#include <ctime>
/* ======================================================================
   Timers
   ====================================================================== */

void MacCogMORTimer::start(double time)
{
	Scheduler &s = Scheduler::instance();
	if (busy_ != 0) {
        fprintf(stderr, "node[%d] - void MacCogMORTimer::start(double time) wrong! Stage:%d timer:%f time:%f \n",mac->getMacAddress(),mac->getProcessStage(),time,Scheduler::instance().clock());
        exit(1);
    }

	busy_ = 1;
	paused_ = 0;
	stime = s.clock();
	rtime = time;
	if (rtime < 0.0) {
        fprintf(stderr, "node[%d] - void MacCogMORTimer::start(double time) \n",mac->getMacAddress());
        exit(1);
    }

	s.schedule(this, &intr, rtime);
}

void MacCogMORTimer::stop(void)
{
	Scheduler &s = Scheduler::instance();

	if (busy_ != 1) {
        fprintf(stderr, "node[%d] - void MacCogMORTimer::stop(void) wrong! \n",mac->getMacAddress());
        exit(1);
    }

	if(paused_ == 0)
		s.cancel(&intr);

	busy_ = 0;
	paused_ = 0;
	stime = 0.0;
	rtime = 0.0;
}

/* ======================================================================
   Rx Timers
   ====================================================================== */
void MacCogMORRxTimer::handle(Event *)
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;

    mac->rxHandler();
}

/* ======================================================================
   Tx Timers
   ====================================================================== */
void MacCogMORTxTimer::handle(Event *)
{
	busy_ = 0;
	paused_ = 0;
	stime = 0.0;
	rtime = 0.0;

	mac->txHandler();
}

/* ======================================================================
   Reply Timers
   ====================================================================== */

void MacCogMORReplyTimer::handle(Event *)
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;

    mac->replyHandler();
}

/* ======================================================================
   Defer Timers
   ====================================================================== */

void MacCogMORDeferTimer::handle(Event *)
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;

    mac->deferHandler();
}

void MacCogMORTimeoutTimer::handle(Event *)
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;

    mac->timeoutHandler();
}

void MacCogMORUIFSTimer::handle(Event *)
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;

    mac->uifsHandler();
}

void MacCogMORBIFSTimer::handle(Event *)
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;

    mac->bifsHandler();
}

/* ======================================================================
   Stage Timer
   ====================================================================== */

void MacCogMORStageTimer::start(double time) {
    mac->stageStart();
    Scheduler &s = Scheduler::instance();
    if (busy_ != 0) {
        fprintf(stderr, "node[%d] - void MacCogMORStageTimer::start(double time) wrong! timer:%f time:%f \n",mac->getMacAddress(),time,Scheduler::instance().clock());
        exit(1);
    }

    busy_ = 1;
    paused_ = 0;
    stime = s.clock();
    rtime = time;
    if (rtime < 0.0) {
        fprintf(stderr, "node[%d] - void MacCogMORTimer::start(double time) \n",mac->getMacAddress());
        exit(1);
    }

    s.schedule(this, &intr, rtime);
}

void MacCogMORStageTimer::handle(Event *)
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;

    mac->stageHandler();
}

/* ======================================================================
   Pu Timers
   ====================================================================== */
void MacCogMORPuTimer::handle(Event *)
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;

    mac->puHandler();
}

/* ======================================================================
   Backoff Timer
   ====================================================================== */
void MacCogMORBackoffTimer::handle(Event *)
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;
    difs_wait = 0.0;

    mac->backoffHandler();
}

void MacCogMORBackoffTimer::pause()
{
    Scheduler &s = Scheduler::instance();

    //the caculation below make validation pass for linux though it
    // looks dummy

    double st = s.clock();
    double rt = stime + difs_wait;
    double sr = st - rt;
    double mst = (mac->macmib_.getSlotTime());
    int slots = int (sr/mst);
    if(slots < 0)
        slots = 0;
    if (busy_ == 0 || paused_ == 1) {
        fprintf(stderr, "node[%d] - void MacCogMORBackoffTimer::pause() wrong!",mac->getMacAddress());
        exit(1);
    }

    paused_ = 1;
    rtime -= (slots * mac->macmib_.getSlotTime());

    if (rtime < 0.0) {
        fprintf(stderr, "node[%d] - void MacCogMORBackoffTimer::pause() wrong!",mac->getMacAddress());
        exit(1);
    }

    difs_wait = 0.0;

    s.cancel(&intr);
}

void MacCogMORBackoffTimer::resume(double difs)
{
    Scheduler &s = Scheduler::instance();

    if (busy_ == 0 || paused_ == 0) {
        fprintf(stderr, "node[%d] - void MacCogMORBackoffTimer::resume(double difs) wrong!",mac->getMacAddress());
        exit(1);
    }

    paused_ = 0;
    stime = s.clock();

    /*
     * The media should be idle for DIFS time before we start
     * decrementing the counter, so I add difs time in here.
     */
    difs_wait = difs;

    if (rtime + difs_wait < 0.0) {
        fprintf(stderr, "node[%d] - void MacCogMORBackoffTimer::resume(double difs) wrong!",mac->getMacAddress());
        exit(1);
    }
    s.schedule(this, &intr, rtime + difs_wait);
}

/* ======================================================================
   CAV Timers
   ====================================================================== */

void MacCogMORCavTimer::start(double time, double cav)
{
    Scheduler &s = Scheduler::instance();
    if (busy_ == 1) {
        fprintf(stderr, "node[%d] - void MacCogMORCavTimer::start(double time, double cav) wrong!",mac->getMacAddress());
        exit(1);
    }

    busy_ = 1;
    paused_ = 0;
    stime = s.clock();
    rtime = time;
    active_ = 1;
    cav_ = cav;
    if (rtime < 0.0) {
        fprintf(stderr, "node[%d] - void MacCogMORCavTimer::start(double time, double cav) wrong!",mac->getMacAddress());
        exit(1);
    }

    s.schedule(this, &intr, rtime);
}

void MacCogMORCavTimer::stop(void)
{
    Scheduler &s = Scheduler::instance();

    if (busy_ == 0) {
        fprintf(stderr, "node[%d] - void MacCogMORCavTimer::stop(void) wrong!",mac->getMacAddress());
        exit(1);
    }

    if(paused_ == 0)
        s.cancel(&intr);

    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;
    active_ = 0;
    cav_ = 0.0;
}

void MacCogMORCavTimer::handle(Event *)
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;
    active_ = 0;
    cav_ = 0.0;
}

void MacCogMORCavTimer::handle()
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;
    active_ = 0;
    cav_ = 0.0;
}

/* ======================================================================
   CAVList
   ====================================================================== */
int MacCogMORCAVList::findCAV(int channel) {
    std::list<MacCogMORCavTimer>::iterator it;
    for (it=cav_list.begin(); it != cav_list.end(); it++) {

        if (((*it).getChannel() == channel) && ((*it).getActive() == 1)) {

            if ((*it).busy() == 0) {
                fprintf(stderr, "int MacCogMORCAVList::findCAV(int channel) wrong! \n");
                exit(1);
            }
            if ((*it).expire() < 0.000001) {
                (*it).stop();
                return 0;
            } else {
                return 1;
            }
        }
    }
    return 0;
}

double MacCogMORCAVList::getCAV(int channel) {
    std::list<MacCogMORCavTimer>::iterator it;
    for (it=cav_list.begin(); it != cav_list.end(); it++) {
        if (((*it).getChannel() == channel) && ((*it).getActive() == 1)) {
            return (*it).getCav();
        }
    }
    return 0;
}

double MacCogMORCAVList::getExpire(int channel) {
    std::list<MacCogMORCavTimer>::iterator it;
    for (it=cav_list.begin(); it != cav_list.end(); it++) {
        if (((*it).getChannel() == channel) && ((*it).getActive() == 1)) {
            return (*it).expire();
        }
    }
    return 0;
}

void MacCogMORCAVList::stopCAV(int channel) {
    std::list<MacCogMORCavTimer>::iterator it;
    for (it = cav_list.begin(); it != cav_list.end(); it++) {
        if (((*it).getChannel() == channel) && ((*it).getActive() == 1)) {
            if ((*it).busy() == 0) {
                fprintf(stderr, "void MacCogMORCAVList::stopCAV(int channel) wrong!");
                exit(1);
            }
            (*it).stop();
            return;
        }
    }
}

void MacCogMORCAVList::startCAV(int channel, double time, double cav) {
    std::list<MacCogMORCavTimer>::iterator it;
    for (it = cav_list.begin(); it != cav_list.end(); it++) {
        if (((*it).getChannel() == channel) && ((*it).getActive() == 0)) {
            if ((*it).busy() != 0) {
                fprintf(stderr, "void MacCogMORCAVList::startCAV(int channel, double time, double cav) wrong!");
                exit(1);
            }
            (*it).start(time,cav);
            return;
        }
    }
}