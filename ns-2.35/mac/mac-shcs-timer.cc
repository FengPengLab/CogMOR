//
// Created by fengpeng on 6/24/17.
//
#include "mac-shcs-timer.h"
#include "mac-shcs.h"
#include <ctime>
#include <stdio.h>
/* ======================================================================
   Timers
   ====================================================================== */

void MacSHCSTimer::start(double time)
{
    Scheduler &s = Scheduler::instance();
    if (busy_ != 0) {
        fprintf(stderr, "void MacSHCSTimer::start(double time) wrong! \n");
        exit(1);
    }

    busy_ = 1;
    paused_ = 0;
    stime = s.clock();
    rtime = time;
    if (rtime < 0.0) {
        fprintf(stderr, "void MacSHCSTimer::start(double time) \n");
        exit(1);
    }

    s.schedule(this, &intr, rtime);
}

void MacSHCSTimer::stop(void)
{
    Scheduler &s = Scheduler::instance();

    if (busy_ != 1) {
        fprintf(stderr, "void MacSHCSTimer::stop(void) wrong! \n");
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
void MacSHCSRxTimer::handle(Event *)
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;

    mac->recvHandler();
}

/* ======================================================================
   Tx Timers
   ====================================================================== */
void MacSHCSTxTimer::handle(Event *)
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;

    mac->sendHandler();
}

/* ======================================================================
   IF Timers
   ====================================================================== */
void MacSHCSIFTimer::handle(Event *)
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;

    mac->txHandler(timeout);
}

/* ======================================================================
   Reply Timers
   ====================================================================== */

void MacSHCSReplyTimer::handle(Event *)
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;

    mac->replyHandler();
}

/* ======================================================================
   Defer Timer
   ====================================================================== */
void MacSHCSDeferTimer::handle(Event *)
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;

    mac->deferHandler();
}

/* ======================================================================
   Phase Timer
   ====================================================================== */
void MacSHCSPhaseTimer::handle(Event *)
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;

    mac->phaseHandler();
}

/* ======================================================================
   Pu Timers
   ====================================================================== */
void MacSHCSPuTimer::handle(Event *)
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
void MacSHCSBackoffTimer::handle(Event *)
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;
    difs_wait = 0.0;

    mac->backoffHandler();
}

void MacSHCSBackoffTimer::start(int cw, int idle, double difs)
{
    Scheduler &s = Scheduler::instance();

    if (busy_ != 0) {
        fprintf(stderr, "void MacSHCSBackoffTimer::start(int cw, int idle, double difs) wrong! 11 %d\n", cw);
        exit(1);
    }

    busy_ = 1;
    paused_ = 0;
    stime = s.clock();

    rtime = (Random::random() % cw) * mac->macmib_.getSlotTime();

    difs_wait = difs;

    if(idle == 0)
        paused_ = 1;
    else {
        if (rtime + difs_wait < 0.0) {
            fprintf(stderr, "void MacSHCSBackoffTimer::start(int cw, int idle, double difs)\n");
            exit(1);
        }
        s.schedule(this, &intr, rtime + difs_wait);
    }
}

void MacSHCSBackoffTimer::start(int idle, double time) {
    Scheduler &s = Scheduler::instance();

    if (busy_ != 0) {
        fprintf(stderr, "void MacSHCSBackoffTimer::start(int idle, double time) wrong! 1\n");
        exit(1);
    }

    busy_ = 1;
    paused_ = 0;
    stime = s.clock();

    rtime = time;

    if(idle == 0)
        paused_ = 1;
    else {
        if (rtime < 0.0) {
            fprintf(stderr, "void MacSHCSBackoffTimer::start(int idle, double time) \n");
            exit(1);
        }
        s.schedule(this, &intr, rtime);
    }
}

void MacSHCSBackoffTimer::pause()
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
        fprintf(stderr, "void MacSHCSBackoffTimer::pause() wrong! 1 %d %d\n",busy_, paused_);
        exit(1);
    }
    paused_ = 1;
    rtime -= (slots * mac->macmib_.getSlotTime());

    if (rtime < 0.0) {
        fprintf(stderr, "void MacSHCSBackoffTimer::pause() wrong! 2 %f\n",rtime);
        exit(1);
    }

    difs_wait = 0.0;

    s.cancel(&intr);
}

void MacSHCSBackoffTimer::resume(double difs)
{
    Scheduler &s = Scheduler::instance();

    if (busy_ == 0 || paused_ == 0) {
        fprintf(stderr, "void MacSHCSBackoffTimer::resume(double difs) wrong!");
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
        fprintf(stderr, "void MacSHCSBackoffTimer::resume(double difs) wrong!");
        exit(1);
    }
    s.schedule(this, &intr, rtime + difs_wait);
}

/* ======================================================================
   CAV Timers
   ====================================================================== */

void MacSHCSCavTimer::start(double time, double cav)
{
    Scheduler &s = Scheduler::instance();
    if (busy_ == 1) {
        fprintf(stderr, "void MacSHCSCavTimer::start(double time, double cav) wrong!");
        exit(1);
    }

    busy_ = 1;
    paused_ = 0;
    stime = s.clock();
    rtime = time;
    active_ = 1;
    cav_ = cav;
    if (rtime < 0.0) {
        fprintf(stderr, "void MacSHCSCavTimer::start(double time, double cav) wrong!");
        exit(1);
    }

    s.schedule(this, &intr, rtime);
}

void MacSHCSCavTimer::stop(void)
{
    Scheduler &s = Scheduler::instance();

    if (busy_ == 0) {
        fprintf(stderr, "void MacSHCSCavTimer::stop(void) wrong!");
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
    mac->checkBackoffTimer();
}

void MacSHCSCavTimer::handle(Event *)
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;
    active_ = 0;
    cav_ = 0.0;
    mac->checkBackoffTimer();
}

void MacSHCSCavTimer::handle()
{
    busy_ = 0;
    paused_ = 0;
    stime = 0.0;
    rtime = 0.0;
    active_ = 0;
    cav_ = 0.0;
    mac->checkBackoffTimer();
}

/* ======================================================================
   CAVList
   ====================================================================== */
int MacSHCSCAVList::findCAV(int channel) {
    std::list<MacSHCSCavTimer>::iterator it;
    for (it=cav_list.begin(); it != cav_list.end(); it++) {
        if (((*it).getChannel() == channel) && ((*it).getActive() == 1)) {
            if ((*it).busy() == 0) {
                fprintf(stderr, "int MacSHCSCAVList::findCAV(int channel) wrong! \n");
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

double MacSHCSCAVList::getCAV(int channel) {
    std::list<MacSHCSCavTimer>::iterator it;
    for (it=cav_list.begin(); it != cav_list.end(); it++) {
        if (((*it).getChannel() == channel) && ((*it).getActive() == 1)) {
            return (*it).getCav();
        }
    }
    return 0;
}

double MacSHCSCAVList::getExpire(int channel) {
    std::list<MacSHCSCavTimer>::iterator it;
    for (it=cav_list.begin(); it != cav_list.end(); it++) {
        if (((*it).getChannel() == channel) && ((*it).getActive() == 1)) {
            return (*it).expire();
        }
    }
    return 0;
}

void MacSHCSCAVList::stopCAV(int channel) {
    std::list<MacSHCSCavTimer>::iterator it;
    for (it = cav_list.begin(); it != cav_list.end(); it++) {
        if (((*it).getChannel() == channel) && ((*it).getActive() == 1)) {
            if ((*it).busy() == 0) {
                fprintf(stderr, "void MacSHCSCAVList::stopCAV(int channel) wrong!");
                exit(1);
            }
            (*it).stop();
            return;
        }
    }
}

void MacSHCSCAVList::startCAV(int channel, double time, double cav) {
    std::list<MacSHCSCavTimer>::iterator it;
    for (it = cav_list.begin(); it != cav_list.end(); it++) {
        if (((*it).getChannel() == channel) && ((*it).getActive() == 0)) {
            if ((*it).busy() != 0) {
                fprintf(stderr, "void MacSHCSCAVList::startCAV(int channel, double time, double cav) wrong!");
                exit(1);
            }
            (*it).start(time,cav);
            return;
        }
    }
}
