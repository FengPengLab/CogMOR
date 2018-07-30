//
// Created by fengpeng on 6/24/17.
//
#include "mac-shcs.h"
#include <sys/types.h>
#include <cstdio>
#include "../common/mobilenode.h"
#include "../trace/cmu-trace.h"
/* ======================================================================
   TCL Hooks for the simulator
   ====================================================================== */
static class MacSHCSClass : public TclClass {
public:
    MacSHCSClass() : TclClass("Mac/SHCS") {}

    TclObject *create(int, const char *const *) {
        return (new MacSHCS());
    }
} class_mac_shcs;

/* ======================================================================
   Mac and Phy MIB Class Functions
   ====================================================================== */
MAC_SHCS_MIB::MAC_SHCS_MIB(MacSHCS *parent) {
    /*
     * Bind the mac mib objects.  Note that these will be bound
     * to Mac/SHCSC variables
     */
    parent->bind("CWMin_", &CWMin);
    parent->bind("CWMax_", &CWMax);
    parent->bind("SlotTime_", &SlotTime);
    parent->bind("SIFS_", &SIFSTime);
    parent->bind("PreambleLength_", &PreambleLength);
    parent->bind("PLCPHeaderLength_", &PLCPHeaderLength);
    parent->bind_bw("PLCPDataRate_", &PLCPDataRate);
    parent->bind("RetryLimit_", &RetryLimit);
    FailedCount = 0;
}

/* ======================================================================
   Mac Class Functions
   ====================================================================== */
MacSHCS::MacSHCS() :
        Mac(), macmib_(this), mhIF_(this), mhRecv_(this),
        mhSend_(this), mhReply_(this), mhDefer_(this),
        mhBackoff_(this), mhPhase_(this), mhPu_(this) {

    Tcl &tcl = Tcl::instance();
    tcl.evalf("Mac/SHCS set basicRate_");
    if (strcmp(tcl.result(), "0") != 0)
        bind_bw("basicRate_", &basicRate_);
    else
        basicRate_ = bandwidth_;

    tcl.evalf("Mac/SHCS set dataRate_");
    if (strcmp(tcl.result(), "0") != 0)
        bind_bw("dataRate_", &dataRate_);
    else
        dataRate_ = bandwidth_;

    et_ = new EventTrace();

    eotPacket_ = NULL;
    pktRTS_ = NULL;
    pktCTS_ = NULL;
    pktACK_ = NULL;
    pktTx_ = NULL;
    pktRx_ = NULL;

    EOTtarget_ = 0;
    logtarget_ = 0;
    cache_ = 0;
    cache_node_count_ = 0;
    T_sensing = 20 * macmib_.getSIFS();
    T_beacon = 10 * macmib_.getSIFS();
    T_reporting = 10 * macmib_.getSIFS();
    T_transfer = 0.01;
    T_switch_channel = macmib_.getSIFS();
    PU_interval = macmib_.getSIFS();
    T_hop_duration = T_sensing + T_beacon + T_reporting + T_transfer + T_switch_channel;
    CurrChannel_ = 0;
    RetryCount_ = 0;
    interface_ = -2;
    SucActive_ = 0;
    NoiseActive_ = 0;
    BeaconActive_ = 0;
    SwitchActive_ = 0;
    lostBeacon_ = 0;
    noiseStartTime_ = 0;
    transmitDst_ = -2;
    SUC_ID_ = index_;
    NodeID = index_;
    TransferType_ = MacSHCS_Silence_Active;
    SendType_ = MacSHCS_Null;

}

int MacSHCS::command(int argc, const char *const *argv) {
    if (argc == 3) {
        if (strcmp(argv[1], "eot-target") == 0) {
            EOTtarget_ = (NsObject *) TclObject::lookup(argv[2]);
            if (EOTtarget_ == 0)
                return TCL_ERROR;
            return TCL_OK;
        } else if (strcmp(argv[1], "log-target") == 0) {
            logtarget_ = (NsObject *) TclObject::lookup(argv[2]);
            if (logtarget_ == 0)
                return TCL_ERROR;
            return TCL_OK;
        } else if(strcmp(argv[1], "eventtrace") == 0) {
            // command added to support event tracing
            et_ = (EventTrace *)TclObject::lookup(argv[2]);
            return (TCL_OK);
        } else if (strcmp(argv[1], "channelNum") == 0) {
            MaxChannelNum_ = atoi(argv[2]);
            for (int i = 0; i < getMaxChannel(); i++) {
                MacSHCSCavTimer cav(i, 0, 0, this);
                mhCAVList_.cav_list.push_back(cav);
            }
            T_frame_duration = T_hop_duration * MaxChannelNum_;
            T_extended_transfer = T_frame_duration - T_hop_duration;
            T_init = T_frame_duration - T_sensing - T_beacon;
            return TCL_OK;
        } else if (strcmp(argv[1], "noiseChannel") == 0) {
            noiseChannel_ = atoi(argv[2]);
            CurrChannel_ = noiseChannel_;
            hoppingChannel_ = noiseChannel_;
            return TCL_OK;
        } else if (strcmp(argv[1], "noiseStartTime") == 0) {
            noiseStartTime_ = atoi(argv[2]);
            if ((nodeWorkMode_ == 2) || (noiseStartTime_ > 0)) {
                if (mhPu_.busy() == 0) {
                    mhPu_.start(noiseStartTime_);
                }
            }
            return TCL_OK;
        } else if (strcmp(argv[1], "nodeWorkMode") == 0) {
            nodeWorkMode_ = atoi(argv[2]);
            /*
             * 0 - 表示节点以正常SU模式工作
             * 1 - 表示节点以SUC模式工作
             * 2 - 表示节点以噪声干扰模式工作
             */
            switch (nodeWorkMode_) {
                case 0:
                    /*
                     * 若节点为SU模式，则执行以下过程
                     */
                    SUC_ID_ = -1;
                    findSUC_ = 0;
                    setPhaseState(MacSHCS_Init);
                    mhPhase_.start(T_frame_duration);
                    break;

                case 1:
                    /*
                     * 若节点为SUC模式，则执行以下过程
                     */
                    SUC_ID_ = index_;
                    setPhaseState(MacSHCS_Init);
                    mhPhase_.start(0.00002);
                    break;
                case 2:
                    /*
                     * 若节点为噪声干扰节点，则执行以下过程
                     */

                    break;
                default:
                    fprintf(stderr,"command():Invalid nodeWorkMode_\n");
                    exit(1);
            }
            return TCL_OK;
        } else if (strcmp(argv[1], "nodes") == 0) {
            if(cache_) return TCL_ERROR;
            cache_node_count_ = atoi(argv[2]);
            cache_ = new SHCSHost[cache_node_count_ + 1];
            assert(cache_);
            bzero(cache_, sizeof(SHCSHost) * (cache_node_count_+1 ));
            return TCL_OK;
        }
    }
    return Mac::command(argc, argv);
}

void MacSHCS::trace_event(char *eventtype, Packet *p)
{
    if (et_ == NULL) return;
    char *wrk = et_->buffer();
    char *nwrk = et_->nbuffer();

    struct hdr_mac_shcs* dh = HDR_MAC_SHCS(p);

    //struct hdr_cmn *ch = HDR_CMN(p);

    if(wrk != 0) {
        sprintf(wrk, "E -t "TIME_FORMAT" %s %2x ",
                et_->round(Scheduler::instance().clock()),
                eventtype,
                //ETHER_ADDR(dh->dh_sa)
                ETHER_ADDR(dh->dh_ta)
        );
    }
    if(nwrk != 0) {
        sprintf(nwrk, "E -t "TIME_FORMAT" %s %2x ",
                et_->round(Scheduler::instance().clock()),
                eventtype,
                //ETHER_ADDR(dh->dh_sa)
                ETHER_ADDR(dh->dh_ta)
        );
    }
    et_->dump();
}

void MacSHCS::recv(Packet *p, Handler *h) {
    struct hdr_cmn *ch = HDR_CMN(p);
    struct hdr_mac_shcs *dh = HDR_MAC_SHCS(p);

    assert(initialized());

    /*
     *  1.Handle outgoing packets.
     */
    if (ch->direction() == hdr_cmn::DOWN) {
        if (nodeWorkMode_ == 0) {
            send(p,h);
        }
        return;
    }

    /*
     *  2.Handle incoming packets.
     */
    ch->iface() = addr();

    if (ch->localif() != getCurrChannel()) {
        Packet::free(p);
        return;
    }

    if (getTxActive()) {
        Packet::free(p);
        return;
    }

    if (getSwitchActive()) {
        Packet::free(p);
        return;
    }

    if (ch->nosise_enabled_ == 1) {
        if (getPhaseState() == MacSHCS_Sensing) {
            NoiseActive_ = 1;
        }
    }

    if ((nodeWorkMode_ == 1) || (nodeWorkMode_ == 2)) {
        Packet::free(p);
        return;
    }

    if (((getPhaseState() == MacSHCS_Beacon) && (ExtendedActive_ == 0)) || (getPhaseState() == MacSHCS_Init)) {
        if (dh->dh_fc.fc_type == MAC_SHCS_Type_Beacon) {
            recvBeacon(p);
            return;
        }
    }

    if ((getPhaseState() == MacSHCS_Reporting) && (ExtendedActive_ == 0)) {
        if (dh->dh_fc.fc_type == MAC_SHCS_Type_Signal) {
            recvSignal(p);
            return;
        }
    }

    if (ch->error()) {
        Packet::free(p);
        return;
    }

    if (ch->nosise_enabled_ == 1) {
        if (mhRecv_.busy()) {
            mhRecv_.stop();
            drop(pktRx_, DROP_MAC_COLLISION);
        }
        pktRx_ = p;
        mhRecv_.start(txtime(p));
        setRxState(MacSHCS_Coll);
        setRxActive(1);
        return;
    }

    if (((getPhaseState() == MacSHCS_Transfer) && (ExtendedActive_ == 0)) || (ExtendedActive_ == 1)) {
        switch (getRxState()) {
            case MacSHCS_Idle:
                setRxActive(1);
                pktRx_ = p;
                mhRecv_.start(txtime(p));
                setRxState(MacSHCS_Recv);
                break;
            case MacSHCS_Recv:
            case MacSHCS_Coll:
                setRxActive(1);
                if (pktRx_->txinfo_.RxPr / p->txinfo_.RxPr >= p->txinfo_.CPThresh) {
                    Packet::free(p);
                } else {
                    if (txtime(p) > mhRecv_.expire()) {
                        mhRecv_.stop();
                        drop(pktRx_, DROP_MAC_COLLISION);
                        pktRx_ = p;
                        mhRecv_.start(txtime(pktRx_));
                        ch = HDR_CMN(pktRx_);
                        ch->error() = 1;
                        setRxState(MacSHCS_Coll);
                    } else {
                        ch = HDR_CMN(pktRx_);
                        ch->error() = 1;
                        drop(p, DROP_MAC_COLLISION);
                    }
                }
                break;
            default:
                fprintf(stderr, "node[%d] - recv() ERROR RxState \n", index_);
                exit(1);
        }
    }
}

void MacSHCS::recvHandler() {
    struct hdr_cmn *ch = HDR_CMN(pktRx_);
    struct hdr_mac_shcs *dh = HDR_MAC_SHCS(pktRx_);
    double now, t, k;
    int src = ETHER_ADDR(dh->dh_ta);
    int dst = ETHER_ADDR(dh->dh_ra);

    if ((getRxState() != MacSHCS_Recv) && (getRxState() != MacSHCS_Coll)) {
        fprintf(stderr, "node[%d] - recvHandler() ERROR getRxState() \n", index_);
        exit(1);
    }

    setRxActive(0);
    setRxState(MacSHCS_Idle);

    if (getSwitchActive()) {
        drop(pktRx_, DROP_MAC_PACKET_ERROR);
        pktRx_ = NULL;
        return;
    }

    if (getTxActive()) {
        drop(pktRx_, DROP_MAC_PACKET_ERROR);
        pktRx_ = NULL;
        return;
    }

    if (ch->nosise_enabled_ == 1) {
        drop(pktRx_, DROP_MAC_PACKET_ERROR);
        pktRx_ = NULL;
        return;
    }

    if (ch->error()) {
        /*t = macmib_.getEIFS();
        if (mhCAVList_.findCAV(getCurrChannel()) == 1) {
            now = Scheduler::instance().clock();
            k = mhCAVList_.getCAV(getCurrChannel());
            if ((now + t) > k) {
                mhCAVList_.stopCAV(getCurrChannel());
                mhCAVList_.startCAV(getCurrChannel(), t, now + t);
            }
        } else {
            now = Scheduler::instance().clock();
            mhCAVList_.startCAV(getCurrChannel(), t, now + t);
        }*/

        drop(pktRx_, DROP_MAC_PACKET_ERROR);
        pktRx_ = NULL;
        return;
    }

    if (tap_
        && dh->dh_fc.fc_type == MAC_SHCS_Type_Data
        && dh->dh_fc.fc_subtype == MAC_SHCS_Subtype_DATA)
        tap_->tap(pktRx_);

    if (netif_->node()->energy_model() && netif_->node()->energy_model()->adaptivefidelity()) {
        netif_->node()->energy_model()->add_neighbor(ETHER_ADDR(dh->dh_ra));
    }

    if ((dst != index_) && (dst != MAC_BROADCAST)) {
        if (mhCAVList_.findCAV(getCurrChannel()) == 1) {
            now = Scheduler::instance().clock();
            t = sec(dh->dh_duration);
            k = mhCAVList_.getCAV(getCurrChannel());
            if ((now + t) > k) {
                mhCAVList_.stopCAV(getCurrChannel());
                mhCAVList_.startCAV(getCurrChannel(), t, now + t);
            }
        } else {
            now = Scheduler::instance().clock();
            t = sec(dh->dh_duration);
            mhCAVList_.startCAV(getCurrChannel(), t, now + t);
        }

        Packet::free(pktRx_);
        pktRx_ = NULL;
        return;
    }

    switch (dh->dh_fc.fc_type) {
        case MAC_SHCS_Type_Control:
            switch (dh->dh_fc.fc_subtype) {
                case MAC_SHCS_Subtype_RTS:
                    recvRTS(pktRx_);
                    pktRx_ = NULL;
                    break;
                case MAC_SHCS_Subtype_CTS:
                    recvCTS(pktRx_);
                    pktRx_ = NULL;
                    break;
                case MAC_SHCS_Subtype_ACK:
                    recvACK(pktRx_);
                    pktRx_ = NULL;
                    break;
                default:
                    fprintf(stderr,"recvHander() ERROR MAC header Subtype frame\n");
                    exit(1);
            }
            break;
        case MAC_SHCS_Type_Data:
            recvDATA(pktRx_);
            pktRx_ = NULL;
            break;
        default:
            fprintf(stderr,"recvHander() ERROR MAC header type frame\n");
            exit(1);
    }
}

void MacSHCS::recvBeacon(Packet *p) {
    struct hdr_cmn *ch = HDR_CMN(p);
    if (BeaconActive_ == 0) {
        struct hdr_mac_shcs_beacon *beacon = (struct hdr_mac_shcs_beacon*)p->access(hdr_mac::offset_);
        BeaconActive_ = 1;
        SUC_ID_ = beacon->suc_id;
        if (getPhaseState() == MacSHCS_Init) {
            if (mhPhase_.busy() == 0) {
                fprintf(stderr, "node[%d] - recvBeacon() ERROR mhPhase_.busy() == 0 \n", index_);
                exit(1);
            }
            mhPhase_.stop();
            setPhaseState(MacSHCS_Beacon);
            mhPhase_.start(T_beacon);
        }

        ch->macLayerDuration = Scheduler::instance().clock() - ch->macLayerTimeStamp;
        mac_log(p);
    } else {
        Packet::free(p);
    }
}

void MacSHCS::recvSignal(Packet *p) {
    struct hdr_cmn *ch = HDR_CMN(p);
    if (CurrChannel_ != hoppingChannel_) {
        fprintf(stderr, "node[%d] - recvSignal() ERROR CurrChannel_ != hoppingChannel_ channel:%d \n", index_, CurrChannel_);
        exit(1);
    }
    fprintf(stderr, "node[%d] - recvSignal() channel:%d \n", index_, CurrChannel_);
    NoiseActive_ = 1;
    ch->macLayerDuration = Scheduler::instance().clock() - ch->macLayerTimeStamp;
    mac_log(p);

}

void MacSHCS::recvRTS(Packet *p) {
    double t;
    double timeout;
    struct hdr_cmn *th;
    struct hdr_cmn *ch;
    struct hdr_mac_shcs *dh;
    int dst;
    int src;

    ch = HDR_CMN(p);
    dh = HDR_MAC_SHCS(p);
    dst = ETHER_ADDR(dh->dh_ra);
    src = ETHER_ADDR(dh->dh_ta);

    timeout = sec(dh->dh_duration)
              - macmib_.getSIFS()
              - txtime(macmib_.getCTSlen(), basicRate_)
              - MacSHCSPropagationDelay;

    ch->macLayerDuration = Scheduler::instance().clock() - ch->macLayerTimeStamp;
    t = ch->macLayerTimeStamp;
    mac_log(p);

    fprintf(stderr, "node[%d] - recvRTS -dst:%d -src:%d \n", index_, dst , src);

    if (getReplyState() != MacSHCS_Idle) {
        fprintf(stderr, "node[%d] - recvRTS() getReplyState() != MacSHCS_Idle \n", index_);
        exit(1);
    }

    sendCTS(src, dst, timeout);
    th = HDR_CMN(pktCTS_);
    th->macLayerTimeStamp = t;

    setReplyState(MacSHCS_Send_CTS);
    if (mhReply_.busy() == 1) {
        fprintf(stderr, "node[%d] - recvRTS() mhReply_.busy() == 0 \n", index_);
        exit(1);
    }
    mhReply_.start(macmib_.getSIFS());
}

void MacSHCS::recvCTS(Packet *p) {
    struct hdr_cmn *ch;
    struct hdr_mac_shcs *dh;
    int dst;
    int src;
    ch = HDR_CMN(p);
    dh = HDR_MAC_SHCS(p);
    dst = ETHER_ADDR(dh->dh_ra);
    src = ETHER_ADDR(dh->dh_ta);

    if (getSendState() == MacSHCS_Send_RTS) {
        if (mhSend_.busy() == 0) {
            fprintf(stderr, "node[%d] - recvCTS() mhSend_.busy() == 0 \n", index_);
            exit(1);
        }
        mhSend_.stop();
        setSendState(MacSHCS_Idle);
    }

    ch->macLayerDuration = Scheduler::instance().clock() - ch->macLayerTimeStamp;

    mac_log(p);

    fprintf(stderr, "node[%d] - recvCTS  -dst:%d -src:%d \n", index_, dst, src);
    if (pktTx_ == NULL) {
        fprintf(stderr, "node[%d] - recvCTS() pktTx_ == NULL) \n", index_);
        exit(1);
    }
    if (mhBackoff_.busy() == 1) {
        fprintf(stderr, "node[%d] - recvCTS() mhBackoff_.busy() == 1 \n", index_);
        exit(1);
    }
    if (mhDefer_.busy() == 1) {
        fprintf(stderr, "node[%d] - recvCTS() mhDefer_.busy() == 1 \n", index_);
        exit(1);
    }
    if (is_idle()) {
        setDeferState(MacSHCS_Send_DATA);
        mhDefer_.start(macmib_.getSIFS());
    } else {
        dh = HDR_MAC_SHCS(pktTx_);
        dst = ETHER_ADDR(dh->dh_ra);
        src = ETHER_ADDR(dh->dh_ta);
        inc_cw();
        sendRTS(dst, src);
        setBackoffState(MacSHCS_Send_RTS);
        if (mhBackoff_.busy() == 1) {
            fprintf(stderr, "node[%d] - send() mhBackoff_.busy() == 1 \n", index_);
            exit(1);
        }
        mhBackoff_.start(cw_, is_idle(), macmib_.getDIFS());
    }
}

void MacSHCS::recvDATA(Packet *p) {
    double t;
    struct hdr_cmn *th;
    struct hdr_cmn *ch;
    struct hdr_mac_shcs *dh;
    int dst;
    int src;
    ch = HDR_CMN(p);
    dh = HDR_MAC_SHCS(p);
    dst = ETHER_ADDR(dh->dh_ra);
    src = ETHER_ADDR(dh->dh_ta);

    ch->size() -= macmib_.getHdrLen();
    ch->num_forwards() += 1;
    ch->macLayerDuration = Scheduler::instance().clock() - ch->macLayerTimeStamp;
    t = ch->macLayerTimeStamp;

    fprintf(stderr, "node[%d] - recvDATA  -dst:%d -src:%d \n", index_, dst, src);

    uptarget_->recv(p, (Handler *) 0);

    if (dst != MAC_BROADCAST) {
        if (mhReply_.busy()) {
            return;
        }

        if (is_idle()) {
            sendACK(src, dst);
            th = HDR_CMN(pktACK_);
            th->macLayerTimeStamp = t;
            setReplyState(MacSHCS_Send_ACK);
            if (mhReply_.busy() == 1) {
                fprintf(stderr, "node[%d] - recvDATA() Invalid mhReply_.busy() == 1 \n", index_);
                exit(1);
            }
            mhReply_.start(macmib_.getSIFS());
        }
    }
}

void MacSHCS::recvACK(Packet *p) {
    struct hdr_cmn *ch;
    struct hdr_mac_shcs *dh;
    int dst;
    int src;
    ch = HDR_CMN(p);
    dh = HDR_MAC_SHCS(p);
    dst = ETHER_ADDR(dh->dh_ra);
    src = ETHER_ADDR(dh->dh_ta);

    fprintf(stderr, "node[%d] - recvACK  -dst:%d -src:%d \n", index_, dst, src);

    if (getSendState() == MacSHCS_Send_DATA) {
        if (mhSend_.busy() == 0) {
            fprintf(stderr, "node[%d] - recvACK() mhSend_.busy() == 0 \n", index_);
            exit(1);
        }
        mhSend_.stop();
        setSendState(MacSHCS_Idle);
    }

    ch->macLayerDuration = Scheduler::instance().clock() - ch->macLayerTimeStamp;

    mac_log(p);
    if (pktTx_ == NULL) {
        fprintf(stderr, "node[%d] - recvACK() pktTx_ == NULL \n", index_);
        exit(1);
    }
    Packet::free(pktTx_);
    pktTx_ = NULL;
    setSendType(MacSHCS_Null);

    TransferType_ = MacSHCS_Extended_Active;

    if (callback_) {
        Handler *h = callback_;
        callback_ = 0;
        h->handle((Event *) 0);
    }
}

void MacSHCS::send(Packet *p, Handler *h) {
    EnergyModel *em = netif_->node()->energy_model();
    if (em && em->sleep()) {
        em->set_node_sleep(0);
        em->set_node_state(EnergyModel::INROUTE);
    }

    sendDATA(p);

    callback_ = h;
    RetryCount_ = 0;
    rst_cw();

    if (mhBackoff_.busy() != 0) {
        fprintf(stderr, "node[%d] - send() ERROR mhBackoff_.busy() != 0 \n", index_);
        exit(1);
    }

    struct hdr_mac_shcs *dh = HDR_MAC_SHCS(p);
    int src = index_;
    int dst = ETHER_ADDR(dh->dh_ra);

    if ((TransferType_ == MacSHCS_Basic_Active) || (TransferType_ == MacSHCS_Extended_Active)) {
        if (transmitDst_ != dst) {
            TransferType_ = MacSHCS_Basic_Active;
        }
    }

    if (nodeWorkMode_ == 0) {
        if (TransferType_ == MacSHCS_Basic_Active) {
            if (dst == MAC_BROADCAST) {
                SendType_ = MacSHCS_Broadcast;
                setBackoffState(MacSHCS_Send_DATA);
            } else {
                sendRTS(dst, src);
                SendType_ = MacSHCS_Unicast;
                setBackoffState(MacSHCS_Send_RTS);
            }

            if (is_idle()) {
                mhBackoff_.start(cw_, 1, macmib_.getDIFS());
            } else {
                mhBackoff_.start(cw_, 0, macmib_.getDIFS());
            }
        } else if (TransferType_ == MacSHCS_Extended_Active) {
            if (is_idle()) {
                if (dst == MAC_BROADCAST) {
                    SendType_ = MacSHCS_Broadcast;
                    setBackoffState(MacSHCS_Send_DATA);
                } else {
                    SendType_ = MacSHCS_Unicast;
                    setBackoffState(MacSHCS_Send_DATA);
                }
                mhBackoff_.start(cw_, 1, macmib_.getDIFS());
            } else {
                TransferType_ = MacSHCS_Basic_Active;

                if (dst == MAC_BROADCAST) {
                    SendType_ = MacSHCS_Broadcast;
                    setBackoffState(MacSHCS_Send_DATA);
                } else {
                    sendRTS(dst, src);
                    SendType_ = MacSHCS_Unicast;
                    setBackoffState(MacSHCS_Send_RTS);
                }
                mhBackoff_.start(cw_, 0, macmib_.getDIFS());
            }
        } else if (TransferType_ == MacSHCS_Silence_Active) {
            if (dst == MAC_BROADCAST) {
                SendType_ = MacSHCS_Broadcast;
                setBackoffState(MacSHCS_Send_DATA);
            } else {
                sendRTS(dst, src);
                SendType_ = MacSHCS_Unicast;
                setBackoffState(MacSHCS_Send_RTS);
            }
            mhBackoff_.start(cw_, 0, macmib_.getDIFS());
        } else {
            fprintf(stderr, "node[%d] - send() ERROR TransferType_ is wrong \n", index_);
            exit(1);
        }
    } else {
        fprintf(stderr, "node[%d] - send() ERROR nodeWorkMode_ \n", index_);
        exit(1);
    }
}

void MacSHCS::sendDATA(Packet *p) {
    struct hdr_cmn *ch = HDR_CMN(p);
    struct hdr_mac *mh = HDR_MAC(p);
    struct hdr_mac_shcs *dh = HDR_MAC_SHCS(p);

    int src = mh->macSA();
    int dst = mh->macDA();

    STORE4BYTE(&dst, dh->dh_ra);
    STORE4BYTE(&src, dh->dh_ta);

    ch->macLayerTimeStamp = Scheduler::instance().clock();
    ch->macLayerDuration = 0;
    ch->size() += macmib_.getHdrLen();
    ch->nosise_enabled_ = 0;
    ch->channelindex_ = getCurrChannel();

    dh->dh_fc.fc_protocol_version = MAC_SHCS_ProtocolVersion;
    dh->dh_fc.fc_type = MAC_SHCS_Type_Data;
    dh->dh_fc.fc_subtype = MAC_SHCS_Subtype_DATA;
    dh->dh_fc.fc_operation = MAC_SHCS_Operation_Basic;
    dh->dh_fc.fc_to_ds = 0;
    dh->dh_fc.fc_from_ds = 0;
    dh->dh_fc.fc_more_frag = 0;     // 分片标志
    dh->dh_fc.fc_retry = 0;         // 重发帧标志
    dh->dh_fc.fc_pwr_mgt = 0;       // 是否处于省电模式
    dh->dh_fc.fc_more_data = 0;     // AP缓存主机数据包标识
    dh->dh_fc.fc_wep = 0;           // 加密标志
    dh->dh_fc.fc_order = 0;         // PCF模式下功能标志

    if (dst == MAC_BROADCAST) {
        ch->txtime() = txtime(ch->size(), basicRate_);
        dh->dh_duration = 0;
    } else {
        ch->txtime() = txtime(ch->size(), dataRate_);
        dh->dh_duration = usec(txtime(macmib_.getACKlen(), basicRate_) + macmib_.getSIFS());
    }

    pktTx_ = p;
}

void MacSHCS::sendBeacon() {
    Packet *p = Packet::alloc();
    hdr_cmn *ch = HDR_CMN(p);
    struct hdr_mac_shcs_beacon *beacon = ((struct hdr_mac_shcs_beacon *)hdr_mac::access(p));

    ch->macLayerTimeStamp = Scheduler::instance().clock();
    ch->macLayerDuration = 0;
    ch->uid() = 0;
    ch->ptype() = PT_MAC;
    ch->size() = macmib_.getBeaconlen();
    ch->iface() = -2;
    ch->error() = 0;
    ch->nosise_enabled_ = 0;
    ch->channelindex_ = getCurrChannel();
    ch->txtime() = txtime(ch->size(), basicRate_);

    bzero(beacon, sizeof(struct hdr_mac_shcs_beacon));

    beacon->dh_fc.fc_protocol_version = MAC_SHCS_ProtocolVersion;
    beacon->dh_fc.fc_type = MAC_SHCS_Type_Beacon;
    beacon->dh_fc.fc_subtype = MAC_SHCS_Subtype_Active_Beacon;
    beacon->dh_fc.fc_operation = MAC_SHCS_Operation_Reserved;
    beacon->dh_fc.fc_to_ds = 0;
    beacon->dh_fc.fc_from_ds = 0;
    beacon->dh_fc.fc_more_frag = 0;
    beacon->dh_fc.fc_retry = 0;
    beacon->dh_fc.fc_pwr_mgt = 0;
    beacon->dh_fc.fc_more_data = 0;
    beacon->dh_fc.fc_wep = 0;
    beacon->dh_fc.fc_order = 0;
    beacon->suc_id = SUC_ID_;

    pktBeacon_ = p;
}

void MacSHCS::sendSignal() {
    Packet *p = Packet::alloc();
    hdr_cmn *ch = HDR_CMN(p);
    struct hdr_mac_shcs_signal *signal = ((struct hdr_mac_shcs_signal *)hdr_mac::access(p));

    ch->macLayerTimeStamp = Scheduler::instance().clock();
    ch->macLayerDuration = 0;
    ch->uid() = 0;
    ch->ptype() = PT_MAC;
    ch->size() = macmib_.getBeaconlen();
    ch->iface() = -2;
    ch->error() = 0;
    ch->nosise_enabled_ = 0;
    ch->channelindex_ = CurrChannel_;
    ch->txtime() = txtime(ch->size(), basicRate_);

    bzero(signal, sizeof(struct hdr_mac_shcs_signal));

    signal->dh_fc.fc_protocol_version = MAC_SHCS_ProtocolVersion;
    signal->dh_fc.fc_type = MAC_SHCS_Type_Signal;
    signal->dh_fc.fc_subtype = MAC_SHCS_Subtype_Busy_Signal;
    signal->dh_fc.fc_operation = MAC_SHCS_Operation_Reserved;
    signal->dh_fc.fc_to_ds = 0;
    signal->dh_fc.fc_from_ds = 0;
    signal->dh_fc.fc_more_frag = 0;
    signal->dh_fc.fc_retry = 0;
    signal->dh_fc.fc_pwr_mgt = 0;
    signal->dh_fc.fc_more_data = 0;
    signal->dh_fc.fc_wep = 0;
    signal->dh_fc.fc_order = 0;

    pktSignal_ = p;
}

void MacSHCS::sendPu() {
    Packet *p = Packet::alloc();
    struct hdr_cmn *ch = HDR_CMN(p);
    struct hdr_mac_shcs *dh = HDR_MAC_SHCS(p);
    int src = index_;
    int dst = -1;
    STORE4BYTE(&dst, dh->dh_ra);
    STORE4BYTE(&src, dh->dh_ta);

    ch->macLayerTimeStamp = Scheduler::instance().clock();
    ch->macLayerDuration = 0;
    ch->uid() = 0;
    ch->ptype() = PT_MAC;
    ch->size() = 100 + macmib_.getHdrLen();
    ch->iface() = -2;
    ch->error() = 0;
    ch->nosise_enabled_ = 1;
    ch->channelindex_ = getCurrChannel();
    ch->txtime() = txtime(ch->size(), basicRate_);
    ch->channelindex_ = getCurrChannel();

    dh->dh_fc.fc_protocol_version = MAC_SHCS_ProtocolVersion;
    dh->dh_fc.fc_type = MAC_SHCS_Type_Data;
    dh->dh_fc.fc_subtype = MAC_SHCS_Subtype_DATA;
    dh->dh_fc.fc_operation = MAC_SHCS_Operation_Basic;
    dh->dh_fc.fc_to_ds = 0;
    dh->dh_fc.fc_from_ds = 0;
    dh->dh_fc.fc_more_frag = 0;     // 分片标志
    dh->dh_fc.fc_retry = 0;         // 重发帧标志
    dh->dh_fc.fc_pwr_mgt = 0;       // 是否处于省电模式
    dh->dh_fc.fc_more_data = 0;     // AP缓存主机数据包标识
    dh->dh_fc.fc_wep = 0;           // 加密标志
    dh->dh_fc.fc_order = 0;         // PCF模式下功能标志
    dh->dh_duration = 0;

    pktTx_ = p;
}

void MacSHCS::sendRTS(int dst, int src) {
    Packet *p = Packet::alloc();
    hdr_cmn *ch = HDR_CMN(p);
    struct hdr_mac_shcs *rts = HDR_MAC_SHCS(p);

    ch->macLayerTimeStamp = Scheduler::instance().clock();
    ch->macLayerDuration = 0;
    ch->uid() = 0;
    ch->ptype() = PT_MAC;
    ch->size() = macmib_.getRTSlen();
    ch->iface() = -2;
    ch->error() = 0;
    ch->nosise_enabled_ = 0;
    ch->channelindex_ = CurrChannel_;
    ch->txtime() = txtime(ch->size(), basicRate_);

    bzero(rts, sizeof(struct hdr_mac_shcs));

    rts->dh_fc.fc_protocol_version = MAC_SHCS_ProtocolVersion;
    rts->dh_fc.fc_type = MAC_SHCS_Type_Control;
    rts->dh_fc.fc_subtype = MAC_SHCS_Subtype_RTS;
    rts->dh_fc.fc_operation = MAC_SHCS_Operation_Basic;
    rts->dh_fc.fc_to_ds = 0;
    rts->dh_fc.fc_from_ds = 0;
    rts->dh_fc.fc_more_frag = 0;
    rts->dh_fc.fc_retry = 0;
    rts->dh_fc.fc_pwr_mgt = 0;
    rts->dh_fc.fc_more_data = 0;
    rts->dh_fc.fc_wep = 0;
    rts->dh_fc.fc_order = 0;

    STORE4BYTE(&dst, rts->dh_ra);
    STORE4BYTE(&src, rts->dh_ta);

    rts->dh_duration = usec(macmib_.getSIFS()
                            + txtime(macmib_.getCTSlen(), basicRate_)
                            + MacSHCSPropagationDelay
                            + macmib_.getSIFS()
                            + txtime(pktTx_)
                            + MacSHCSPropagationDelay
                            + macmib_.getSIFS()
                            + txtime(macmib_.getACKlen(), basicRate_)
                            + MacSHCSPropagationDelay);
    pktRTS_ = p;
}

void MacSHCS::sendCTS(int dst, int src, double time) {
    Packet *p = Packet::alloc();
    hdr_cmn *ch = HDR_CMN(p);
    struct hdr_mac_shcs *cts = HDR_MAC_SHCS(p);

    ch->macLayerTimeStamp = Scheduler::instance().clock();
    ch->macLayerDuration = 0;
    ch->uid() = 0;
    ch->ptype() = PT_MAC;
    ch->size() = macmib_.getCTSlen();
    ch->iface() = -2;
    ch->error() = 0;
    ch->nosise_enabled_ = 0;
    ch->channelindex_ = CurrChannel_;
    ch->txtime() = txtime(ch->size(), basicRate_);

    bzero(cts, sizeof(struct hdr_mac_shcs));

    cts->dh_fc.fc_protocol_version = MAC_SHCS_ProtocolVersion;
    cts->dh_fc.fc_type = MAC_SHCS_Type_Control;
    cts->dh_fc.fc_subtype = MAC_SHCS_Subtype_CTS;
    cts->dh_fc.fc_operation = MAC_SHCS_Operation_Basic;
    cts->dh_fc.fc_to_ds = 0;
    cts->dh_fc.fc_from_ds = 0;
    cts->dh_fc.fc_more_frag = 0;
    cts->dh_fc.fc_retry = 0;
    cts->dh_fc.fc_pwr_mgt = 0;
    cts->dh_fc.fc_more_data = 0;
    cts->dh_fc.fc_wep = 0;
    cts->dh_fc.fc_order = 0;
    cts->dh_duration = usec(time);

    STORE4BYTE(&dst, cts->dh_ra);
    STORE4BYTE(&src, cts->dh_ta);

    pktCTS_ = p;
}

void MacSHCS::sendACK(int dst, int src) {
    Packet *p = Packet::alloc();
    hdr_cmn *ch = HDR_CMN(p);
    struct hdr_mac_shcs *ack = HDR_MAC_SHCS(p);

    ch->macLayerTimeStamp = Scheduler::instance().clock();
    ch->macLayerDuration = 0;
    ch->uid() = 0;
    ch->ptype() = PT_MAC;
    ch->size() = macmib_.getACKlen();
    ch->iface() = -2;
    ch->error() = 0;
    ch->nosise_enabled_ = 0;
    ch->channelindex_ = CurrChannel_;
    ch->txtime() = txtime(ch->size(), basicRate_);

    bzero(ack, sizeof(struct hdr_mac_shcs));

    ack->dh_fc.fc_protocol_version = MAC_SHCS_ProtocolVersion;
    ack->dh_fc.fc_type = MAC_SHCS_Type_Control;
    ack->dh_fc.fc_subtype = MAC_SHCS_Subtype_ACK;
    ack->dh_fc.fc_operation = MAC_SHCS_Operation_Basic;
    ack->dh_fc.fc_to_ds = 0;
    ack->dh_fc.fc_from_ds = 0;
    ack->dh_fc.fc_more_frag = 0;
    ack->dh_fc.fc_retry = 0;
    ack->dh_fc.fc_pwr_mgt = 0;
    ack->dh_fc.fc_more_data = 0;
    ack->dh_fc.fc_wep = 0;
    ack->dh_fc.fc_order = 0;
    ack->dh_duration = 0;

    STORE4BYTE(&dst, ack->dh_ra);
    STORE4BYTE(&src, ack->dh_ta);

    pktACK_ = p;
}

void MacSHCS::backoffHandler(void) {
    struct hdr_mac_shcs *dh = HDR_MAC_SHCS(pktTx_);
    int dst = ETHER_ADDR(dh->dh_ra);
    int src = ETHER_ADDR(dh->dh_ta);
    if (mhBackoff_.busy() != 0) {
        fprintf(stderr, "node[%d] - backoffHandler() ERROR mhBackoff_.busy() != 0 \n", index_);
        exit(1);
    }
    if (mhDefer_.busy() != 0) {
        fprintf(stderr, "node[%d] - backoffHandler() ERROR mhDefer_.busy() != 0 \n", index_);
        exit(1);
    }
    switch (getBackoffState()) {
        case MacSHCS_Send_RTS:
            setBackoffState(MacSHCS_Idle);
            if (pktRTS_ == NULL) {
                fprintf(stderr, "node[%d] - backoffHandler() ERROR pktRTS_ == NULL \n", index_);
                exit(1);
            }
            if (is_idle()) {
                setDeferState(MacSHCS_Send_RTS);
                mhDefer_.start(T_sensing);
            } else {
                inc_cw();
                setBackoffState(MacSHCS_Send_RTS);
                mhBackoff_.start(cw_, 0, macmib_.getDIFS());
            }
            break;
        case MacSHCS_Send_DATA:
            setBackoffState(MacSHCS_Idle);
            if (is_idle()) {
                setDeferState(MacSHCS_Send_DATA);
                mhDefer_.start(T_sensing);
            } else {
                inc_cw();
                if (getSendType() == MacSHCS_Unicast) {
                    sendRTS(dst, src);
                    setBackoffState(MacSHCS_Send_RTS);
                    mhBackoff_.start(cw_, 0, macmib_.getDIFS());
                }
                if (getSendType() == MacSHCS_Broadcast) {
                    setBackoffState(MacSHCS_Send_DATA);
                    mhBackoff_.start(cw_, 0, macmib_.getDIFS());
                }
            }
            break;
        default:
            fprintf(stderr, "node[%d] - backoffHandler() ERROR invalid BackoffState \n", index_);
            exit(1);
    }
}

void MacSHCS::deferHandler(void) {
    struct hdr_mac_shcs *dh = HDR_MAC_SHCS(pktTx_);
    int dst = ETHER_ADDR(dh->dh_ra);
    int src = ETHER_ADDR(dh->dh_ta);
    double timeout;
    switch (getDeferState()) {
        case MacSHCS_Send_RTS:
            setDeferState(MacSHCS_Idle);
            if (pktRTS_ == NULL) {
                fprintf(stderr, "node[%d] - backoffHandler() ERROR pktRTS_ == NULL \n", index_);
                exit(1);
            }
            if (is_idle()) {
                if (mhIF_.busy() != 0) {
                    fprintf(stderr, "node[%d] - backoffHandler() ERROR mhIF_.busy() != 0 \n", index_);
                    exit(1);
                }
                fprintf(stderr,"node[%d] - transmit:RTS -dst:%d -src:%d -time:%f channel:%d\n",
                        index_, dst, src, Scheduler::instance().clock(),CurrChannel_);
                timeout = macmib_.getSIFS()
                          + txtime(macmib_.getCTSlen(), basicRate_)
                          + MacSHCSPropagationDelay;

                setTxState(MacSHCS_Send_RTS);
                transmit(pktRTS_,timeout);
            } else {
                inc_cw();
                setBackoffState(MacSHCS_Send_RTS);
                if (mhBackoff_.busy() != 0) {
                    fprintf(stderr, "node[%d] - backoffHandler() ERROR mhBackoff_.busy() != 0 \n", index_);
                    exit(1);
                }
                mhBackoff_.start(cw_, 0, macmib_.getDIFS());
            }
            break;
        case MacSHCS_Send_DATA:
            setDeferState(MacSHCS_Idle);
            if (pktTx_ == NULL) {
                fprintf(stderr, "node[%d] - deferHandler() ERROR pktTx_ == NULL \n", index_);
                exit(1);
            }
            if (is_idle()) {
                if (mhIF_.busy() != 0) {
                    fprintf(stderr, "node[%d] - deferHandler() ERROR mhIF_.busy() != 0 \n", index_);
                    exit(1);
                }
                fprintf(stderr,"node[%d] - transmit:DATA -dst:%d -src:%d -time:%f channel:%d\n",
                        index_, dst, src, Scheduler::instance().clock(),CurrChannel_);
                if (getSendType() == MacSHCS_Unicast) {
                    timeout = macmib_.getSIFS()
                              + txtime(macmib_.getACKlen(), basicRate_)
                              + MacSHCSPropagationDelay;
                }

                if (getSendType() == MacSHCS_Broadcast) {
                    timeout = 0;
                }
                setTxState(MacSHCS_Send_DATA);
                transmit(pktTx_,timeout);
            } else {
                if (mhBackoff_.busy() != 0) {
                    fprintf(stderr, "node[%d] - deferHandler() ERROR mhBackoff_.busy() != 0 \n", index_);
                    exit(1);
                }
                inc_cw();
                if (getSendType() == MacSHCS_Unicast) {
                    sendRTS(dst, src);
                    setBackoffState(MacSHCS_Send_RTS);
                }
                if (getSendType() == MacSHCS_Broadcast) {
                    setBackoffState(MacSHCS_Send_DATA);
                }
                mhBackoff_.start(cw_, 0, macmib_.getDIFS());
            }
            break;
        default:
            fprintf(stderr,"deferHandler() ERROR DeferWorkMode\n");
            exit(1);
    }
}

void MacSHCS::sendHandler(void) {
    struct hdr_mac_shcs *dh;
    int dst, src;

    switch (getSendState()) {
        case MacSHCS_Send_Beacon:
        case MacSHCS_Send_Signal:
        case MacSHCS_Send_CTS:
        case MacSHCS_Send_ACK:
            setSendState(MacSHCS_Idle);
            break;
        case MacSHCS_Send_RTS:
        case MacSHCS_Send_DATA:
            setSendState(MacSHCS_Idle);
            RetryCount_++;

            if (RetryCount_ >= macmib_.getRetryLimit()) {
                macmib_.FailedCount++;
                RetryCount_ = 0;
                rst_cw();
                hdr_cmn *ch = HDR_CMN(pktTx_);
                if (ch->xmit_failure_) {
                    ch->size() -= macmib_.getHdrLen();
                    ch->xmit_reason_ = XMIT_REASON_ACK;
                    ch->xmit_failure_(pktTx_->copy(),
                                      ch->xmit_failure_data_);
                }

                drop(pktTx_, DROP_MAC_RETRY_COUNT_EXCEEDED);
                pktTx_ = NULL;
                setSendType(MacSHCS_Null);
                if(callback_) {
                    Handler *h = callback_;
                    callback_ = 0;
                    h->handle((Event*) 0);
                }
                return;
            }

            if (TransferType_ == MacSHCS_Extended_Active) {
                TransferType_ = MacSHCS_Basic_Active;
            }

            if (pktTx_ == NULL) {
                fprintf(stderr, "node[%d] - sendHandler() ERROR pktTx_ == NULL \n", index_);
                exit(1);
            }
            if (mhBackoff_.busy() != 0) {
                fprintf(stderr, "node[%d] - sendHandler() ERROR mhBackoff_.busy() != 0 \n", index_);
                exit(1);
            }

            dh = HDR_MAC_SHCS(pktTx_);
            dst = ETHER_ADDR(dh->dh_ra);
            src = ETHER_ADDR(dh->dh_ta);

            if (getSendType() == MacSHCS_Broadcast) {
                setBackoffState(MacSHCS_Send_DATA);

            } else if (getSendType() == MacSHCS_Unicast) {
                sendRTS(dst, src);
                setBackoffState(MacSHCS_Send_RTS);
            }
            inc_cw();
            mhBackoff_.start(cw_, is_idle(), macmib_.getDIFS());
            break;
        default:
            fprintf(stderr, "node[%d] - sendHandler() ERROR TxState %d\n", index_,getSendState());
            exit(1);
    }
}

void MacSHCS::phaseHandler(void) {
    struct hdr_cmn *ch;
    struct hdr_mac_shcs *dh;
    int dst;
    int src;
    int flag;
    switch (getPhaseState()) {
        case MacSHCS_Init:
            if (nodeWorkMode_ == 0) {
                setPhaseState(MacSHCS_Switching);
                mhPhase_.start(T_switch_channel);

                switchChannel();
                setSwitchActive(1);
                setExtendedActive(0);
                findSUC_ = 0;
                BeaconActive_ = 0;
                NoiseActive_ = 0;
                CurrChannel_ = hoppingChannel_;
                TransferType_ = MacSHCS_Silence_Active;
            }

            if (nodeWorkMode_ == 1) {
                setPhaseState(MacSHCS_Sensing);
                mhPhase_.start(T_sensing);
                NoiseActive_ = 0;
            }
            break;
        case MacSHCS_Sensing:
            setPhaseState(MacSHCS_Beacon);
            mhPhase_.start(T_beacon);

            if (nodeWorkMode_ == 1) {
                if (NoiseActive_ == 0) {
                    sendBeacon();
                    setTxState(MacSHCS_Send_Beacon);
                    transmit(pktBeacon_, 0);
                }
            }
            break;
        case MacSHCS_Beacon:
            setPhaseState(MacSHCS_Reporting);
            mhPhase_.start(T_reporting);

            if ((nodeWorkMode_ == 0) && (ExtendedActive_ == 0)) {
                if (NoiseActive_ == 1) {
                    sendSignal();
                    setTxState(MacSHCS_Send_Signal);
                    transmit(pktSignal_, 0);
                }
            }
            break;
        case MacSHCS_Reporting:
            setPhaseState(MacSHCS_Transfer);
            mhPhase_.start(T_transfer);

            if ((nodeWorkMode_ == 0)
                && (ExtendedActive_ == 0)
                && (BeaconActive_ == 1)
                && (NoiseActive_ == 0)) {

                TransferType_ = MacSHCS_Basic_Active;

            }
            break;
        case MacSHCS_Transfer:
            setPhaseState(MacSHCS_Switching);
            mhPhase_.start(T_switch_channel);

            switchChannel();
            if (nodeWorkMode_ == 1) {
                setSwitchActive(1);
                NoiseActive_ = 0;
                CurrChannel_ = hoppingChannel_;
            }

            if (nodeWorkMode_ == 0) {

                /*
                 * Extended Data Transfer would create many isolated nodes.
                 * It may work for fixed tx/rx pairs with 1 hopping,
                 * But it is not suitable for the network with high layer protocol stack.
                 * When the destination is always changing,
                 * it actually have very bad impact.
                 *
                 * If want to open the Extended Data Transfer function,
                 * so you can just uncomment it.
                 */

                /*
                 * Extended Data Transfer (beginning)
                 */
                if (getRxActive()) {
                    if (getRxState() == MacSHCS_Recv) {
                        ch = HDR_CMN(pktRx_);
                        dh = HDR_MAC_SHCS(pktRx_);
                        dst = ETHER_ADDR(dh->dh_ra);
                        if ((ch->nosise_enabled_ == 0) && (ch->error() == 0)){
                            if (dst == index_) {
                                flag = 1;
                            } else {
                                flag = 0;
                            }
                        } else {
                            flag = 0;
                        }
                    } else {
                        flag = 0;
                    }
                } else if (getTxActive()) {
                    if (getSendType() == MacSHCS_Unicast) {
                        flag = 1;
                    } else {
                        flag = 0;
                    }
                } else {
                    flag = 0;
                }

                if ((flag == 1)
                    && (CurrChannel_ != hoppingChannel_)
                    && (TransferType_ != MacSHCS_Silence_Active)
                    && (NoiseActive_ == 0)
                    && ((BeaconActive_ == 1) || ((BeaconActive_ == 0) && (ExtendedActive_ == 1)))) {
                    setExtendedActive(1);
                    setSwitchActive(0);
                    BeaconActive_ = 0;
                    NoiseActive_ = 0;
                    TransferType_ = MacSHCS_Basic_Active;
                } else {
                    setExtendedActive(0);
                    setSwitchActive(1);
                    BeaconActive_ = 0;
                    NoiseActive_ = 0;
                    CurrChannel_ = hoppingChannel_;
                    TransferType_ = MacSHCS_Silence_Active;

                    if (mhRecv_.busy()) {
                        mhRecv_.stop();
                        setRxActive(0);
                        setRxState(MacSHCS_Idle);
                        if (pktRx_ == NULL) {
                            fprintf(stderr, "node[%d] - sendHandler() ERROR pktRx_ == NULL \n", index_);
                            exit(1);
                        }
                        Packet::free(pktRx_);
                        pktRx_ = NULL;
                    }

                    if (mhIF_.busy()) {
                        mhIF_.stop();
                        switch (getTxState()) {
                            case MacSHCS_Send_RTS:
                                if (pktRTS_ == NULL) {
                                    fprintf(stderr, "node[%d] - sendHandler() ERROR pktRTS_ == NULL \n", index_);
                                    exit(1);
                                }
                                Packet::free(pktRTS_);
                                pktRTS_ = NULL;
                            case MacSHCS_Send_DATA:
                                setTxActive(0);
                                setTxState(MacSHCS_Idle);
                                RetryCount_++;
                                if (RetryCount_ >= macmib_.getRetryLimit()) {
                                    macmib_.FailedCount++;
                                    RetryCount_ = 0;
                                    rst_cw();
                                    hdr_cmn *ch = HDR_CMN(pktTx_);
                                    if (ch->xmit_failure_) {
                                        ch->size() -= macmib_.getHdrLen();
                                        ch->xmit_reason_ = XMIT_REASON_ACK;
                                        ch->xmit_failure_(pktTx_->copy(),
                                                          ch->xmit_failure_data_);
                                    }

                                    drop(pktTx_, DROP_MAC_RETRY_COUNT_EXCEEDED);
                                    pktTx_ = NULL;
                                    setSendType(MacSHCS_Null);
                                    setSendState(MacSHCS_Idle);
                                    if(callback_) {
                                        Handler *h = callback_;
                                        callback_ = 0;
                                        h->handle((Event*) 0);
                                    }
                                    return;
                                }

                                if (pktTx_ == NULL) {
                                    fprintf(stderr, "node[%d] - sendHandler() ERROR pktTx_ == NULL \n", index_);
                                    exit(1);
                                }
                                if (mhBackoff_.busy() != 0) {
                                    fprintf(stderr, "node[%d] - sendHandler() ERROR mhBackoff_.busy() != 0 \n", index_);
                                    exit(1);
                                }
                                dh = HDR_MAC_SHCS(pktTx_);
                                dst = ETHER_ADDR(dh->dh_ra);
                                src = ETHER_ADDR(dh->dh_ta);
                                inc_cw();
                                switch (getSendType()) {
                                    case MacSHCS_Broadcast:
                                        setBackoffState(MacSHCS_Send_DATA);
                                        mhBackoff_.start(cw_, is_idle(), macmib_.getDIFS());
                                        break;
                                    case MacSHCS_Unicast:
                                        sendRTS(dst, src);
                                        setBackoffState(MacSHCS_Send_RTS);
                                        mhBackoff_.start(cw_, is_idle(), macmib_.getDIFS());
                                        break;
                                    case MacSHCS_Null:
                                    default:
                                        fprintf(stderr, "node[%d] - sendHandler() ERROR SendType \n", index_);
                                        exit(1);
                                }
                                break;
                            case MacSHCS_Send_CTS:
                                setTxActive(0);
                                setTxState(MacSHCS_Idle);
                                if (pktCTS_ == NULL) {
                                    fprintf(stderr, "node[%d] - sendHandler() ERROR pktCTS_ == NULL \n", index_);
                                    exit(1);
                                }
                                Packet::free(pktCTS_);
                                pktCTS_ = NULL;
                                break;
                            case MacSHCS_Send_ACK:
                                setTxActive(0);
                                setTxState(MacSHCS_Idle);
                                if (pktACK_ == NULL) {
                                    fprintf(stderr, "node[%d] - sendHandler() ERROR pktACK_ == NULL \n", index_);
                                    exit(1);
                                }
                                Packet::free(pktACK_);
                                pktACK_ = NULL;
                                break;
                            default:
                                fprintf(stderr, "node[%d] - phaseHandler() ERROR getTxState() \n", index_);
                                exit(1);
                        }
                    }

                    if (mhReply_.busy()) {
                        mhReply_.stop();
                        switch (getReplyState()) {
                            case MacSHCS_Send_CTS:
                                setReplyState(MacSHCS_Idle);
                                drop(pktCTS_, DROP_MAC_BUSY);
                                pktCTS_ = NULL;
                                break;
                            case MacSHCS_Send_ACK:
                                setReplyState(MacSHCS_Idle);
                                drop(pktACK_, DROP_MAC_BUSY);
                                pktACK_ = NULL;
                                break;
                            default:
                                fprintf(stderr, "node[%d] - phaseHandler() ERROR ReplyWorkMode\n", index_);
                                exit(1);
                        }
                    }

                    if (mhDefer_.busy()) {
                        mhDefer_.stop();
                        switch (getDeferState()) {
                            case MacSHCS_Send_RTS:
                                if (pktRTS_ == NULL) {
                                    fprintf(stderr, "node[%d] - sendHandler() ERROR pktRTS_ == NULL \n", index_);
                                    exit(1);
                                }
                                Packet::free(pktRTS_);
                                pktRTS_ = NULL;
                            case MacSHCS_Send_DATA:
                                setDeferState(MacSHCS_Idle);
                                if (pktTx_ == NULL) {
                                    fprintf(stderr, "node[%d] - phaseHandler() ERROR pktTx_ == NULL \n", index_);
                                    exit(1);
                                }
                                if (mhBackoff_.busy() != 0) {
                                    fprintf(stderr, "node[%d] - sendHandler() ERROR mhBackoff_.busy() != 0 \n", index_);
                                    exit(1);
                                }
                                dh = HDR_MAC_SHCS(pktTx_);
                                dst = ETHER_ADDR(dh->dh_ra);
                                src = ETHER_ADDR(dh->dh_ta);
                                inc_cw();
                                switch (getSendType()) {
                                    case MacSHCS_Broadcast:
                                        setBackoffState(MacSHCS_Send_DATA);
                                        mhBackoff_.start(cw_, is_idle(), macmib_.getDIFS());
                                        break;
                                    case MacSHCS_Unicast:
                                        sendRTS(dst, src);
                                        setBackoffState(MacSHCS_Send_RTS);
                                        mhBackoff_.start(cw_, is_idle(), macmib_.getDIFS());
                                        break;
                                    case MacSHCS_Null:
                                    default:
                                        fprintf(stderr, "node[%d] - sendHandler() ERROR SendType \n", index_);
                                        exit(1);
                                }
                                break;
                            default:
                                fprintf(stderr, "node[%d] - phaseHandler() ERROR getDeferState()\n", index_);
                                exit(1);
                        }
                    }
                }
                /*
                 * Extended Data Transfer (End)
                 */
            }
            break;
        case MacSHCS_Switching:
            /*
             * 完成切换信道工作
             */
            if ((nodeWorkMode_ == 0) && (findSUC_ == 1) && (ExtendedActive_ == 0)) {
                setPhaseState(MacSHCS_Sensing);
                mhPhase_.start(T_sensing);

                setSwitchActive(0);
                NoiseActive_ = 0;
                BeaconActive_ = 0;
            }  else if ((nodeWorkMode_ == 0) && (findSUC_ == 1) && (ExtendedActive_ == 1)) {
                setPhaseState(MacSHCS_Sensing);
                mhPhase_.start(T_sensing);
            } else if ((nodeWorkMode_ == 0) && (findSUC_ == 0)) {
                setPhaseState(MacSHCS_Init);
                mhPhase_.start(T_frame_duration);

                setSwitchActive(0);
                NoiseActive_ = 0;
                BeaconActive_ = 0;
            } else if (nodeWorkMode_ == 1) {
                setPhaseState(MacSHCS_Sensing);
                mhPhase_.start(T_sensing);

                setSwitchActive(0);
                NoiseActive_ = 0;
                BeaconActive_ = 0;
            }
            break;
        default:
            fprintf(stderr, "node[%d] - send(): ERROR phaseState_ \n", index_);
            exit(1);
    }
}

void MacSHCS::replyHandler(void) {
    double timeout;
    struct hdr_mac_shcs *dh;
    switch (getReplyState()) {
        case MacSHCS_Send_CTS:
            setReplyState(MacSHCS_Idle);
            if (is_idle()) {
                dh = HDR_MAC_SHCS(pktCTS_);
                timeout = sec(dh->dh_duration);

                fprintf(stderr,"node[%d] - transmit:CTS -time:%f channel:%d\n",
                        index_, Scheduler::instance().clock(),CurrChannel_);
                setTxState(MacSHCS_Send_CTS);
                transmit(pktCTS_,timeout);
            } else {
                drop(pktCTS_, DROP_MAC_BUSY);
                pktCTS_ = NULL;
            }
            break;
        case MacSHCS_Send_ACK:
            setReplyState(MacSHCS_Idle);
            if (is_idle()) {
                timeout = 0;

                fprintf(stderr,"node[%d] - transmit:ACK  -time:%f channel:%d\n",
                        index_, Scheduler::instance().clock(),CurrChannel_);
                setTxState(MacSHCS_Send_ACK);
                transmit(pktACK_,timeout);
            } else {
                drop(pktACK_, DROP_MAC_BUSY);
                pktACK_ = NULL;
            }
            break;
        default:
            fprintf(stderr,"deferHandler() ERROR ReplyWorkMode\n");
            exit(1);
    }
}

void MacSHCS::txHandler(double timeout) {
    if (EOTtarget_) {
        if (eotPacket_ == NULL) {
            fprintf(stderr,"txHandler() ERROR eotPacket_ == NULL \n");
            exit(1);
        }
        EOTtarget_->recv(eotPacket_, (Handler *) 0);
        eotPacket_ = NULL;
    }

    if (nodeWorkMode_ == 1) {
        setTxState(MacSHCS_Idle);
        setTxActive(0);
        Packet::free(pktBeacon_);
        pktBeacon_ = NULL;
        return;
    }

    if (nodeWorkMode_ == 2) {
        setTxState(MacSHCS_Idle);
        setTxActive(0);
        Packet::free(pktTx_);
        pktTx_ = NULL;
        if (mhPu_.busy() == 0) {
            mhPu_.start(PU_interval);
        }
        return;
    }

    if (nodeWorkMode_ != 0) {
        fprintf(stderr, "node[%d] - txHandler() ERROR nodeWorkMode_ \n", index_);
        exit(1);
    }

    switch (getTxState()) {
        case MacSHCS_Send_Signal:
            setTxState(MacSHCS_Idle);
            setTxActive(0);
            if (pktSignal_ == NULL) {
                fprintf(stderr, "node[%d] - txHandler() ERROR pktSignal_ == NULL \n", index_);
                exit(1);
            }
            Packet::free(pktSignal_);
            pktSignal_ = NULL;
            break;
        case MacSHCS_Send_RTS:
            setTxState(MacSHCS_Idle);
            setTxActive(0);
            setSendState(MacSHCS_Send_RTS);
            if (mhSend_.busy() == 1) {
                fprintf(stderr, "node[%d] - txHandler() ERROR mhSend_.busy() == 1 \n", index_);
                exit(1);
            }
            mhSend_.start(timeout);
            if (pktRTS_ == NULL) {
                fprintf(stderr, "node[%d] - txHandler() ERROR pktRTS_ == NULL \n", index_);
                exit(1);
            }
            Packet::free(pktRTS_);
            pktRTS_ = NULL;
            break;
        case MacSHCS_Send_CTS:
            setTxState(MacSHCS_Idle);
            setTxActive(0);
            if (pktCTS_ == NULL) {
                fprintf(stderr, "node[%d] - txHandler() ERROR pktCTS_ == NULL \n", index_);
                exit(1);
            }
            Packet::free(pktCTS_);
            pktCTS_ = NULL;
            break;
        case MacSHCS_Send_DATA:
            setTxState(MacSHCS_Idle);
            setTxActive(0);
            switch (getSendType()) {
                case MacSHCS_Unicast:
                    setSendState(MacSHCS_Send_DATA);
                    if (mhSend_.busy() == 1) {
                        fprintf(stderr, "node[%d] - txHandler() ERROR mhSend_.busy() == 1 \n", index_);
                        exit(1);
                    }
                    mhSend_.start(timeout);
                    break;
                case MacSHCS_Broadcast:
                    if (pktTx_ == NULL) {
                        fprintf(stderr, "node[%d] - txHandler() ERROR pktTx_ == NULL \n", index_);
                        exit(1);
                    }
                    Packet::free(pktTx_);
                    pktTx_ = NULL;
                    setSendType(MacSHCS_Null);
                    if ((ExtendedActive_ == 1)
                        || ((ExtendedActive_ == 0) && (getPhaseState() == MacSHCS_Transfer))) {
                        TransferType_ = MacSHCS_Extended_Active;
                    }

                    if(callback_) {
                        Handler *h = callback_;
                        callback_ = 0;
                        h->handle((Event*) 0);
                    }
                    break;
                case MacSHCS_Null:
                default:
                    fprintf(stderr, "node[%d] - txHandler() ERROR TxState \n", index_);
                    exit(1);
            }
            break;
        case MacSHCS_Send_ACK:
            setTxState(MacSHCS_Idle);
            setTxActive(0);
            if (pktACK_ == NULL) {
                fprintf(stderr, "node[%d] - txHandler() ERROR pktACK_ == NULL \n", index_);
                exit(1);
            }
            Packet::free(pktACK_);
            pktACK_ = NULL;
            break;
        default:
            fprintf(stderr, "node[%d] - txHandler() ERROR TxState \n", index_);
            exit(1);
    }
}

void MacSHCS::puHandler(void) {
    sendPu();
    transmit(pktTx_, 0);
}

void MacSHCS::transmit(Packet *p, double timeout) {
    struct hdr_cmn *ch = HDR_CMN(p);
    struct hdr_mac_shcs *dh = HDR_MAC_SHCS(p);
    setTxActive(1);

    if ((nodeWorkMode_ == 0) && (ExtendedActive_ == 0)) {
        if (getPhaseState() != MacSHCS_Transfer) {
            if (getPhaseState() != MacSHCS_Reporting) {
                fprintf(stderr, "node[%d] - transmit: can't transmit this moment -phase:%d\n", index_,getPhaseState());
                exit(1);
            }
        }
    }

    ch->channelindex_ = CurrChannel_;

    if (nodeWorkMode_ == 0) {
        if (TransferType_ == MacSHCS_Basic_Active) {
            dh->dh_fc.fc_operation = MAC_SHCS_Operation_Basic;
            transmitDst_ = ETHER_ADDR(dh->dh_ra);
        }

        if (TransferType_ == MacSHCS_Extended_Active) {
            dh->dh_fc.fc_operation = MAC_SHCS_Operation_Extended;
            transmitDst_ = ETHER_ADDR(dh->dh_ra);
        }

        if (TransferType_ == MacSHCS_Silence_Active) {
            dh->dh_fc.fc_operation = MAC_SHCS_Operation_Reserved;
        }
    } else {
        dh->dh_fc.fc_operation = MAC_SHCS_Operation_Reserved;
    }

    if (getRxActive()) {
        ch = HDR_CMN(pktRx_);
        ch->error() = 1;
    }

    if (EOTtarget_) {
        if (eotPacket_ != NULL) {
            fprintf(stderr, "node[%d] - transmit() ERROR eotPacket_ != NULL \n", index_);
            exit(1);
        }
        eotPacket_ = p->copy();
    }

    if (mhIF_.busy() == 1) {
        fprintf(stderr, "node[%d] - transmit() ERROR mhIF_.busy() == 0 \n", index_);
        exit(1);
    }

    mhIF_.start(txtime(p));
    mhIF_.setTimeOut(timeout + macmib_.getSIFS());
    downtarget_->recv(p->copy(), this);
}

void MacSHCS::setRxActive(int i) {
    RxActive_ = i;
    checkBackoffTimer();
}

void MacSHCS::setTxActive(int i) {
    TxActive_ = i;
    checkBackoffTimer();
}

void MacSHCS::setSwitchActive(int k) {
    SwitchActive_ = k;
    checkBackoffTimer();
}

void MacSHCS::setPhaseState(MacSHCSPhaseState p) {
    PhaseState_ = p;
    checkBackoffTimer();
}

void MacSHCS::setExtendedActive(int p) {
    ExtendedActive_ = p;
    checkBackoffTimer();
}

void MacSHCS::checkBackoffTimer() {
    if (mhBackoff_.busy()) {
        if ((getTxActive() == 0)
            && (getRxActive() == 0)
            && (getSwitchActive() == 0)
            && mhBackoff_.paused()
            && (((ExtendedActive_ == 0) && (getPhaseState() == MacSHCS_Transfer)) || (ExtendedActive_ == 1))) {
            mhBackoff_.resume(macmib_.getDIFS());
        }

        if ((getTxActive() || getRxActive() || getSwitchActive())
            && (mhBackoff_.paused() == 0)) {
            mhBackoff_.pause();
        }
    }
}

int MacSHCS::is_idle() {
    if (ExtendedActive_ == 0) {
        if (getPhaseState() != MacSHCS_Transfer) {
            return 0;
        }

        if (getTxActive()) {
            return 0;
        }

        if (getRxActive()) {
            return 0;
        }

        if (getSwitchActive() == 1) {
            return 0;
        }

        if (mhCAVList_.findCAV(getCurrChannel()) == 1) {
            return 0;
        }
    }

    if (ExtendedActive_ == 1) {
        if (getTxActive()) {
            return 0;
        }

        if (getRxActive()) {
            return 0;
        }

        if (getSwitchActive() == 1) {
            return 0;
        }

        if (mhCAVList_.findCAV(getCurrChannel()) == 1) {
            return 0;
        }
    }

    return 1;
}

void MacSHCS::switchChannel() {
    hoppingChannel_ = hoppingChannel_ + 1;
    if (hoppingChannel_ >= MaxChannelNum_) {
        hoppingChannel_ = MacSHCS_Default_Channel;
    }
}

double MacSHCS::txtime(Packet *p) {
    struct hdr_cmn *ch = HDR_CMN(p);
    double t = ch->txtime();
    if (t < 0.0) {
        drop(p, "XXX");
        fprintf(stderr,"txtime:Invalid txtime()\n");
        exit(1);
    }
    return t;
}

double MacSHCS::txtime(double psz, double drt) {
    double dsz = psz - macmib_.getPLCPhdrLen();
    int plcp_hdr = macmib_.getPLCPhdrLen() << 3;
    int datalen = (int) dsz << 3;
    double t = (((double) plcp_hdr) / macmib_.getPLCPDataRate()) + (((double) datalen) / drt);
    return t;
}