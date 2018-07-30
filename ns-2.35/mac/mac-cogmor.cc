//
// Created by fengpeng on 2/20/17.
//

// #define DEBUG 99
#include <time.h>
#include <cstdio>
#include "mac-cogmor.h"
#include "../common/mobilenode.h"
#include "../trace/cmu-trace.h"
/* ======================================================================
   TCL Hooks for the simulator
   ====================================================================== */
static class MacCogMORClass : public TclClass {
public:
    MacCogMORClass() : TclClass("Mac/CogMOR") {}

    TclObject *create(int, const char *const *) {
        return (new MacCogMOR());
    }
} class_mac_cogmor;

/* ======================================================================
   Mac and Phy MIB Class Functions
   ====================================================================== */
MAC_COGMOR_MIB::MAC_COGMOR_MIB(MacCogMOR *parent) {
    /*
     * Bind the mac mib objects.  Note that these will be bound
     * to Mac/CogMOR variables
     */
    parent->bind("CWMin_", &CWMin);
    parent->bind("CWMax_", &CWMax);
    parent->bind("SlotTime_", &SlotTime);
    parent->bind("SIFS_", &SIFSTime);
    parent->bind("DIFS_", &DIFSTime);
    parent->bind("EIFS_", &EIFSTime);
    parent->bind("NIFS_", &NIFSTime);
    parent->bind("PreambleLength_", &PreambleLength);
    parent->bind("PLCPHeaderLength_", &PLCPHeaderLength);
    parent->bind_bw("PLCPDataRate_", &PLCPDataRate);
    parent->bind("RetryLimit_", &RetryLimit);
    parent->bind("RetryShortLimit_", &RetryShortLimit);
    FailedCount = 0;

}

/* ======================================================================
   Mac Class Functions
   ====================================================================== */
MacCogMOR::MacCogMOR() :
        Mac(), macmib_(this), mhTx_(this), mhRx_(this),
        mhStage_(this), mhReply_(this), mhBackoff_(this), mhPu_(this),
        mhDefer_(this), mhUIFS_(this), mhBIFS_(this), mhTimeout_(this) {

    Tcl &tcl = Tcl::instance();
    tcl.evalf("Mac/CogMOR set basicRate_");
    if (strcmp(tcl.result(), "0") != 0)
        bind_bw("basicRate_", &basicRate_);
    else
        basicRate_ = bandwidth_;

    tcl.evalf("Mac/CogMOR set dataRate_");
    if (strcmp(tcl.result(), "0") != 0)
        bind_bw("dataRate_", &dataRate_);
    else
        dataRate_ = bandwidth_;

    et_ = new EventTrace();

    eotPacket_ = NULL;
    pktBeacon_ = NULL;
    pktPTS_ = NULL;
    pktPTR_ = NULL;
    pktACK_ = NULL;
    pktCx_ = NULL;
    pktTx_ = NULL;
    pktRx_ = NULL;

    EOTtarget_ = 0;
    logtarget_ = 0;
    cache_ = 0;
    cache_node_count_ = 0;

    RetryCount_ = 0;
    RetryShortCount_ = 0;
    macmib_.rst_cw();

    CurrChannel_ = -2;
    noiseStartTime_ = 0;

    ReservationChannel_ = -2;
    DataTransferChannel_ = -2;

    Rx_address = -2;
    Tx_address = -2;

    Reservation_dst = -2;
    Reservation_src = -2;

    DataTransfer_dst = -2;
    DataTransfer_src = -2;

    ReservationPeriodNum_ = 0;
    DataTransferPeriodNum_ = 0;

    MaxReservationPeriodNum_ = 10;
    MaxDataTransferPeriodNum_ = 10;

    RxActive_ = Inactive;
    TxActive_ = Inactive;
    DxActive_ = Inactive;
    CxActive_ = Inactive;
    ProrityActive_ = Inactive;

    mhRxState_ = Rx_Idle;
    mhTxState_ = Tx_Idle;
    mhReplyState_ = Reply_Idle;
    mhDeferState_ = Defer_Idle;

    SendType_ = SendType_Null;
    MaxDate_Sizeof_ = 1500;
    PU_interval = macmib_.getSIFS();

    T_B2 = macmib_.getSIFS() + txtime(macmib_.getBEACONlen(), basicRate_) + MacCogMORPropagationDelay;
    T_Sx1 = 50 * macmib_.getSIFS();
    T_Sx2 = 20 * macmib_.getSIFS();
    T_Sx3 = 20 * macmib_.getSIFS();
    T_Tu = getMaxUIFS() + txtime(macmib_.getPTSlen(), basicRate_) + MacCogMORPropagationDelay;
    T_Ru = macmib_.getSIFS() + txtime(macmib_.getPTRlen(), basicRate_) + MacCogMORPropagationDelay;
    T_Tb = getMaxBIFS() + txtime(macmib_.getHdrLen() + MaxDate_Sizeof_, basicRate_)  + MacCogMORPropagationDelay;
    T_Rb = macmib_.getSIFS() + txtime(macmib_.getACKlen(), basicRate_) + MacCogMORPropagationDelay;
    T_Tx = getMaxUIFS() + txtime(macmib_.getHdrLen() + MaxDate_Sizeof_, basicRate_)  + MacCogMORPropagationDelay;
    T_Rx = macmib_.getSIFS() + txtime(macmib_.getACKlen(), basicRate_) + MacCogMORPropagationDelay;
    T_Cx = macmib_.getSIFS() + txtime(macmib_.getCONlen(), basicRate_) + MacCogMORPropagationDelay;
    T_broadcast_timeout = 1;
    T_unicast_timeout = 1;

    ProcessStage_ = Stage_Sx0;
    stageStart();
}

int MacCogMOR::command(int argc, const char *const *argv) {
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
            for (int i = 0; i < MaxChannelNum_; i++) {
                MacCogMORCavTimer cav(i, 0, 0, this);
                mhCAVList_.cav_list.push_back(cav);
            }
            return TCL_OK;
        } else if (strcmp(argv[1], "nodeWorkMode") == 0) {
            nodeWorkMode_ = atoi(argv[2]);
            if (nodeWorkMode_ == NodeWorkMode_Cognitive_Node) {
                // initialize Cognitive Node
                transceiverMode_ = Sensing;
                ProcessStage_ = Stage_Sx0;
            }
            return TCL_OK;
        } else if (strcmp(argv[1], "noiseChannel") == 0) {
            NoiseChannel_ = atoi(argv[2]);
            return TCL_OK;
        } else if (strcmp(argv[1], "noiseStartTime") == 0) {
            noiseStartTime_ = atoi(argv[2]);
            if ((nodeWorkMode_ == NodeWorkMode_Interference_Source) || (noiseStartTime_ > 0)) {
                if (mhPu_.busy() == 0) {
                    mhPu_.start(noiseStartTime_);
                }
            }
            return TCL_OK;
        } else if (strcmp(argv[1], "nodes") == 0) {
            if(cache_) return TCL_ERROR;
            cache_node_count_ = atoi(argv[2]);
            cache_ = new CogMORHost[cache_node_count_ + 1];
            assert(cache_);
            bzero(cache_, sizeof(CogMORHost) * (cache_node_count_+1 ));
            return TCL_OK;
        }
    }
    return Mac::command(argc, argv);
}

void MacCogMOR::trace_event(char *eventtype, Packet *p)
{
    if (et_ == NULL) return;
    char *wrk = et_->buffer();
    char *nwrk = et_->nbuffer();

    struct hdr_mac_cogmor* dh = HDR_MAC_COGMOR(p);

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

int MacCogMOR::integer(int cw) {
    srand(time(NULL));
    for (int i = 0; i <= index_; i++) {
        output = (rand() % (cw - 1)) + 1;
    }
    return output;
}

int MacCogMOR::getMacAddress() {
    return index_;
}

int MacCogMOR::getProcessStage() {
    return ProcessStage_;
}

void MacCogMOR::recv(Packet *p, Handler *h) {
    std::list<ChannelInfo>::iterator it;
    double duration;
    struct hdr_cmn *ch = HDR_CMN(p);
    int chan = ch->localif();
    double now = Scheduler::instance().clock();
    assert(initialized());


    /*
     *  1.Handle outgoing packets.
     */
    if (ch->direction() == hdr_cmn::DOWN) {
        if (nodeWorkMode_ == NodeWorkMode_Interference_Source) {
            fprintf(stderr, "void MacCogMOR::recv() ERROR nodeWorkMode_ \n");
            exit(1);
        } else {
            send(p, h);
        }
        return;
    }
    /*
     *  2.Handle incoming packets.
     */
    ch->iface() = addr();

    /*struct hdr_mac_cogmor *dh = HDR_MAC_COGMOR(p);
    switch (dh->dh_fc.fc_subtype) {
        case MAC_CogMOR_Subtype_CON:
            fprintf(stderr, "node[%d] MAC_CogMOR_Subtype_CON channel:%d enabled:%d\n", index_,chan,ch->nosise_enabled_);
            break;
        case MAC_CogMOR_Subtype_PTS:
            fprintf(stderr, "node[%d] MAC_CogMOR_Subtype_PTS channel:%d enabled:%d\n", index_,chan,ch->nosise_enabled_);
            break;
        case MAC_CogMOR_Subtype_PTR:
            fprintf(stderr, "node[%d] MAC_CogMOR_Subtype_PTR channel:%d enabled:%d\n", index_,chan,ch->nosise_enabled_);
            break;
        case MAC_CogMOR_Subtype_Beacon_B1_U:
            fprintf(stderr, "node[%d] MAC_CogMOR_Subtype_Beacon_B1_U channel:%d enabled:%d\n", index_,chan,ch->nosise_enabled_);
            break;
        case MAC_CogMOR_Subtype_Beacon_B2_U:
            fprintf(stderr, "node[%d] MAC_CogMOR_Subtype_Beacon_B2_U channel:%d enabled:%d\n", index_,chan,ch->nosise_enabled_);
            break;
        case MAC_CogMOR_Subtype_Beacon_B1_B:
            fprintf(stderr, "node[%d] MAC_CogMOR_Subtype_Beacon_B1_B channel:%d enabled:%d\n", index_,chan,ch->nosise_enabled_);
            break;
        case MAC_CogMOR_Subtype_Beacon_B2_B:
            fprintf(stderr, "node[%d] MAC_CogMOR_Subtype_Beacon_B2_B channel:%d enabled:%d\n", index_,chan,ch->nosise_enabled_);
            break;
        case MAC_CogMOR_Subtype_Data:
            fprintf(stderr, "node[%d] MAC_CogMOR_Subtype_Data channel:%d enabled:%d\n", index_,chan,ch->nosise_enabled_);
            break;
        case MAC_CogMOR_Subtype_ACK:
            fprintf(stderr, "node[%d] MAC_CogMOR_Subtype_ACK channel:%d enabled:%d\n", index_,chan,ch->nosise_enabled_);
            break;
        default:
            fprintf(stderr, "node[%d] default channel:%d enabled:%d\n", index_,chan,ch->nosise_enabled_);
            break;
    }*/
    if (nodeWorkMode_ == NodeWorkMode_Interference_Source) {
        Packet::free(p);
        return;
    }

    if (nodeWorkMode_ == NodeWorkMode_Cognitive_Node) {
        if (ch->nosise_enabled_ == 1) {
            duration = macmib_.getNIFS();
            if (mhCAVList_.findCAV(chan) == 1) {
                if ((now + duration) > mhCAVList_.getCAV(chan)) {
                    mhCAVList_.stopCAV(chan);
                    mhCAVList_.startCAV(chan, duration, now + duration);
                }
            } else {
                mhCAVList_.startCAV(chan, duration, now + duration);
            }
        }
        switch (transceiverMode_) {
            case Receiving:
                switch (mhRxState_) {
                    case Rx_Idle:
                        if (chan == CurrChannel_) {
                            pktRx_ = p;
                            RxActive_ = Active;
                            mhRxState_ = Rx_Recv;
                            mhRx_.start(txtime(p));
                        } else {
                            Packet::free(p);
                        }
                        break;
                    case Rx_Recv:
                    case Rx_Coll:
                        if (chan == CurrChannel_) {
                            RxActive_ = Active;
                            if (pktRx_->txinfo_.RxPr / p->txinfo_.RxPr >= p->txinfo_.CPThresh) {
                                Packet::free(p);
                            } else {
                                if (txtime(p) > mhRx_.expire()) {
                                    mhRxState_ = Rx_Coll;
                                    mhRx_.stop();
                                    drop(pktRx_, DROP_MAC_COLLISION);
                                    pktRx_ = p;
                                    ch = HDR_CMN(p);
                                    ch->collision() = 1;
                                    mhRx_.start(txtime(pktRx_));
                                } else {
                                    ch = HDR_CMN(pktRx_);
                                    ch->collision() = 1;
                                    drop(p, DROP_MAC_COLLISION);
                                }
                            }
                        } else {
                            Packet::free(p);
                        }
                        break;
                    default:
                        fprintf(stderr, "node[%d] - recv() ERROR mhRxState_:%d mode:%d \n", index_,mhRxState_,transceiverMode_);
                        exit(1);
                }
                break;
            case Sensing:
                if (mhRxState_ == Rx_Idle) {
                    if ((ProcessStage_ == Stage_Sx0)
                        && (ch->frame_type == 1)
                        && (ch->error() == 0)) {

                        pktRx_ = p;
                        RxActive_ = Active;
                        transceiverMode_ = Receiving;
                        mhRxState_ = Rx_Recv;
                        mhRx_.start(txtime(p));
                        ReservationChannel_ = chan;
                        CurrChannel_ = chan;
                        break;
                    } else {
                        duration = macmib_.getEIFS();
                        if (mhCAVList_.findCAV(chan) == 1) {
                            if ((now + duration) > mhCAVList_.getCAV(chan)) {
                                mhCAVList_.stopCAV(chan);
                                mhCAVList_.startCAV(chan, duration, now + duration);
                            }
                        } else {
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    }

                    if (CurrChannel_ == chan) {
                        DxActive_ = Active;
                    }
                    Packet::free(p);
                }
                break;
            case Transmitting:
                Packet::free(p);
                break;
            default:
                fprintf(stderr, "void MacCogMOR::recv() ERROR nodeWorkMode_ \n");
                exit(1);
        }
    }
}

void MacCogMOR::send(Packet *p, Handler *h) {
    std::list<ChannelInfo>::iterator it;
    sendDATA(p);
    struct hdr_cmn *ch;
    struct hdr_mac_cogmor *dh = HDR_MAC_COGMOR(p);
    SendType_src = ETHER_ADDR(dh->dh_ta);
    SendType_dst = ETHER_ADDR(dh->dh_ra);

    if (SendType_dst == MAC_BROADCAST) {
        SendType_ = SendType_Broadcast;
        mhTimeout_.start(T_broadcast_timeout);
    } else {
        SendType_ = SendType_Unicast;
        mhTimeout_.start(T_unicast_timeout);
    }

    callback_ = h;
    RetryCount_ = 0;
    RetryShortCount_ = 0;
    macmib_.rst_cw();
    mhDeferState_ = Defer_Idle;

    EnergyModel *em = netif_->node()->energy_model();
    if (em && em->sleep()) {
        em->set_node_sleep(0);
        em->set_node_state(EnergyModel::INROUTE);
    }

    if ((RxActive_ != Active)&&(TxActive_ != Active)) {
        switch (ProcessStage_) {
            case Stage_Sx0:
                Reservation_src = SendType_src;
                Reservation_dst = SendType_dst;
                ReservationChannel_ = -2;
                init_channel_list();
                ReservationChannel_ = selectRC();
                switch (SendType_) {
                    case SendType_Unicast:
                        if (mhUIFS_.busy() == 0) {
                            ProcessStage_ = Stage_B1_U;
                            stageStart();
                            mhTxState_ = Tx_Beacon_B1_U;
                            sendBeacon(SendType_dst,SendType_src,Beacon_B1_U);
                            transmit(pktBeacon_, CurrChannel_);
                            ProrityActive_ = Active;
                            fprintf(stderr,"node[%d] - transmit():Beacon_B1 -dst:%d -src:%d -chan:%d -time:%f\n",
                                    index_, SendType_dst , SendType_src, CurrChannel_, Scheduler::instance().clock());

                        } else {
                            mhBackoff_.start(getUIFS());
                        }
                        break;
                    case SendType_Broadcast:
                        if (mhBIFS_.busy() == 0) {
                            ProcessStage_ = Stage_B1_B;
                            stageStart();
                            mhTxState_ = Tx_Beacon_B1_B;
                            sendBeacon(SendType_dst, SendType_src, Beacon_B1_B);
                            transmit(pktBeacon_, CurrChannel_);
                            ProrityActive_ = Active;
                            fprintf(stderr,"node[%d] - transmit():Beacon_B1 -dst:%d -src:%d -chan:%d -time:%f\n",
                                    index_, SendType_dst , SendType_src, CurrChannel_, Scheduler::instance().clock());

                        } else {
                            mhBackoff_.start(getBIFS());
                        }
                        break;
                    default:
                        fprintf(stderr, "node[%d] - void MacCogMOR::send ERROR SendType_ \n", index_);
                        exit(1);

                }
                break;
            case Stage_Tu:
                if ((SendType_ == SendType_Unicast) && (mhStage_.expire() > txtime(macmib_.getPTSlen(), basicRate_))) {
                    if (is_idle(CurrChannel_)) {
                        transceiverMode_ = Transmitting;
                        mhTxState_ = Tx_PTS;
                        init_channel_list();
                        remove_channel_list(CurrChannel_);
                        DataTransferChannel_ = selectDC();
                        sendPTS(Rx_address,Tx_address, DataTransferChannel_);
                        transmit(pktPTS_, CurrChannel_);
                        fprintf(stderr,"node[%d] - transmit():PTS -dst:%d -src:%d -chan:%d -datachan:%d -is_idle:%d -time:%f\n", index_, SendType_dst , SendType_src, CurrChannel_, DataTransferChannel_,is_idle(DataTransferChannel_),Scheduler::instance().clock());
                    }
                }
                break;
            case Stage_Tx:
                ch = HDR_CMN(p);
                if ((SendType_ == SendType_Unicast) && (mhStage_.expire() > txtime(ch->size(), basicRate_))) {
                    if (is_idle(CurrChannel_)) {
                        transceiverMode_ = Transmitting;
                        mhTxState_ = Tx_DATA_U;
                        transmit(pktTx_, CurrChannel_);
                        fprintf(stderr, "node[%d] - transmit():DATA_U -dst:%d -src:%d -chan:%d -time:%f\n",
                                index_, SendType_dst, SendType_src, CurrChannel_, Scheduler::instance().clock());
                    }
                }
                break;
            default:
                break;
        }
    }
}

void MacCogMOR::sendDATA(Packet *p) {
    struct hdr_cmn *ch = HDR_CMN(p);
    struct hdr_mac *mh = HDR_MAC(p);
    struct hdr_mac_cogmor *dh = HDR_MAC_COGMOR(p);
    int src = mh->macSA();
    int dst = mh->macDA();

    ch->nosise_enabled_ = 0;
    ch->macLayerTimeStamp = Scheduler::instance().clock();
    ch->macLayerDuration = 0;
    ch->size() += macmib_.getHdrLen();
    ch->channelindex_ = -2;
    ch->collision() = 0;
    ch->frame_type = 0;

    STORE4BYTE(&dst, dh->dh_ra);
    STORE4BYTE(&src, dh->dh_ta);

    dh->dh_fc.fc_protocol_version = MAC_CogMOR_ProtocolVersion;
    dh->dh_fc.fc_type = MAC_CogMOR_Type_Data;
    dh->dh_fc.fc_subtype = MAC_CogMOR_Subtype_Data;
    dh->dh_fc.fc_more_frag = 0;
    dh->dh_fc.fc_retry = 0;
    if (dst == MAC_BROADCAST) {
        dh->dh_fc.fc_flag = 1;
    } else {
        dh->dh_fc.fc_flag = 0;
    }
    dh->dh_fc.fc_order = 0;
    dh->dh_fc.fc_reserved = 0;

    if (dst == MAC_BROADCAST) {
        ch->txtime() = txtime(ch->size(), basicRate_);
        dh->dh_duration = usec(txtime(macmib_.getACKlen(), basicRate_)
                               + MacCogMORPropagationDelay
                               + macmib_.getSIFS());
    } else {
        ch->txtime() = txtime(ch->size(), dataRate_);
        dh->dh_duration = usec(txtime(macmib_.getACKlen(), basicRate_)
                               + MacCogMORPropagationDelay
                               + macmib_.getSIFS());
    }

    pktTx_ = p;
}

void MacCogMOR::sendBeacon(int dst, int src, int type) {
    Packet *p = Packet::alloc();
    struct hdr_cmn *ch = HDR_CMN(p);
    struct hdr_mac_cogmor_control_short *bh = ((hdr_mac_cogmor_control_short *)hdr_mac::access(p));

    ch->macLayerTimeStamp = Scheduler::instance().clock();
    ch->macLayerDuration = 0;
    ch->uid() = 0;
    ch->ptype() = PT_MAC;
    ch->size() = macmib_.getBEACONlen();
    ch->iface() = -2;
    ch->error() = 0;
    ch->collision() = 0;
    ch->nosise_enabled_ = 0;
    ch->channelindex_ = -2;
    ch->txtime() = txtime(ch->size(), basicRate_);
    ch->frame_type = 1;

    bzero(bh, sizeof(struct hdr_mac_cogmor_control_short));

    STORE4BYTE(&dst, bh->dh_ra);
    STORE4BYTE(&src, bh->dh_ta);

    bh->dh_fc.fc_protocol_version = MAC_CogMOR_ProtocolVersion;
    bh->dh_fc.fc_type = MAC_CogMOR_Type_Beacon;
    switch (type) {
        case Beacon_B1_U:
            bh->dh_fc.fc_subtype = MAC_CogMOR_Subtype_Beacon_B1_U;
            break;
        case Beacon_B2_U:
            bh->dh_fc.fc_subtype = MAC_CogMOR_Subtype_Beacon_B2_U;
            break;
        case Beacon_B1_B:
            bh->dh_fc.fc_subtype = MAC_CogMOR_Subtype_Beacon_B1_B;
            break;
        case Beacon_B2_B:
            bh->dh_fc.fc_subtype = MAC_CogMOR_Subtype_Beacon_B2_B;
            break;
        default:
            fprintf(stderr,"void MacCogMOR::sendBeacon() ERROR type\n");
            exit(1);
    }

    pktBeacon_ = p;
}

void MacCogMOR::sendPTS(int dst, int src, int channel) {
    Packet *p = Packet::alloc();
    struct hdr_cmn *ch = HDR_CMN(p);
    struct hdr_mac_cogmor_control_long *pts = (hdr_mac_cogmor_control_long *)hdr_mac::access(p);

    ch->macLayerTimeStamp = Scheduler::instance().clock();
    ch->macLayerDuration = 0;
    ch->uid() = 0;
    ch->ptype() = PT_MAC;
    ch->size() = macmib_.getPTSlen();
    ch->iface() = -2;
    ch->error() = 0;
    ch->collision() = 0;
    ch->nosise_enabled_ = 0;
    ch->channelindex_ = -2;
    ch->txtime() = txtime(ch->size(), basicRate_);
    ch->frame_type = 0;
    bzero(pts, sizeof(struct hdr_mac_cogmor_control_long));

    STORE4BYTE(&dst, pts->dh_ra);
    STORE4BYTE(&src, pts->dh_ta);

    pts->dh_fc.fc_protocol_version = MAC_CogMOR_ProtocolVersion;
    pts->dh_fc.fc_type = MAC_CogMOR_Type_Control;
    pts->dh_fc.fc_subtype = MAC_CogMOR_Subtype_PTS;
    pts->dh_fc.fc_flag = 0;
    pts->dh_fc.fc_more_frag = 0;
    pts->dh_fc.fc_retry = 0;
    pts->dh_fc.fc_order = 0;
    pts->dh_fc.fc_reserved = 0;

    pts->dh_channel = (u_int16_t) channel;
    pts->dh_duration = usec(macmib_.getSIFS()
                            + MacCogMORPropagationDelay
                            + txtime(macmib_.getPTRlen(), basicRate_)
                            + T_Sx3
                            + macmib_.getSIFS()
                            + MacCogMORPropagationDelay
                            + txtime(pktTx_));

    pktPTS_ = p;
}

void MacCogMOR::sendPTR(int dst, int src, int channel, double timeout) {
    Packet *p = Packet::alloc();
    struct hdr_cmn *ch = HDR_CMN(p);
    struct hdr_mac_cogmor_control_long *ptr = (hdr_mac_cogmor_control_long *)hdr_mac::access(p);

    ch->macLayerTimeStamp = Scheduler::instance().clock();
    ch->macLayerDuration = 0;
    ch->uid() = 0;
    ch->ptype() = PT_MAC;
    ch->size() = macmib_.getPTRlen();
    ch->iface() = -2;
    ch->error() = 0;
    ch->collision() = 0;
    ch->nosise_enabled_ = 0;
    ch->channelindex_ = -2;
    ch->txtime() = txtime(ch->size(), basicRate_);
    ch->frame_type = 0;
    bzero(ptr, sizeof(struct hdr_mac_cogmor_control_long));

    STORE4BYTE(&dst, ptr->dh_ra);
    STORE4BYTE(&src, ptr->dh_ta);

    ptr->dh_fc.fc_protocol_version = MAC_CogMOR_ProtocolVersion;
    ptr->dh_fc.fc_type = MAC_CogMOR_Type_Control;
    ptr->dh_fc.fc_subtype = MAC_CogMOR_Subtype_PTR;
    ptr->dh_fc.fc_flag = 0;
    ptr->dh_fc.fc_more_frag = 0;
    ptr->dh_fc.fc_retry = 0;
    ptr->dh_fc.fc_order = 0;
    ptr->dh_fc.fc_reserved = 0;
    ptr->dh_channel = (u_int16_t) channel;
    ptr->dh_duration = usec(timeout);

    pktPTR_ = p;
}

void MacCogMOR::sendACK(int dst, int src) {
    Packet *p = Packet::alloc();
    struct hdr_cmn *ch = HDR_CMN(p);
    struct hdr_mac_cogmor_control_short *ack = (hdr_mac_cogmor_control_short *)hdr_mac::access(p);

    ch->macLayerTimeStamp = Scheduler::instance().clock();
    ch->macLayerDuration = 0;
    ch->uid() = 0;
    ch->ptype() = PT_MAC;
    ch->size() = macmib_.getACKlen();
    ch->iface() = -2;
    ch->error() = 0;
    ch->collision() = 0;
    ch->nosise_enabled_ = 0;
    ch->channelindex_ = -2;
    ch->txtime() = txtime(ch->size(), basicRate_);
    ch->frame_type = 0;
    bzero(ack, sizeof(struct hdr_mac_cogmor_control_short));
    STORE4BYTE(&dst, ack->dh_ra);
    STORE4BYTE(&src, ack->dh_ta);
    ack->dh_fc.fc_protocol_version = MAC_CogMOR_ProtocolVersion;
    ack->dh_fc.fc_type = MAC_CogMOR_Type_Control;
    ack->dh_fc.fc_subtype = MAC_CogMOR_Subtype_ACK;
    ack->dh_fc.fc_flag = 0;
    ack->dh_fc.fc_more_frag = 0;
    ack->dh_fc.fc_retry = 0;
    ack->dh_fc.fc_order = 0;
    ack->dh_fc.fc_reserved = 0;

    pktACK_ = p;
}

void MacCogMOR::sendCON(int dst, int src) {
    Packet *p = Packet::alloc();
    struct hdr_cmn *ch = HDR_CMN(p);
    struct hdr_mac_cogmor_control_short *cx = (hdr_mac_cogmor_control_short *)hdr_mac::access(p);

    ch->macLayerTimeStamp = Scheduler::instance().clock();
    ch->macLayerDuration = 0;
    ch->uid() = 0;
    ch->ptype() = PT_MAC;
    ch->size() = macmib_.getACKlen();
    ch->iface() = -2;
    ch->error() = 0;
    ch->collision() = 0;
    ch->nosise_enabled_ = 0;
    ch->channelindex_ = -2;
    ch->txtime() = txtime(ch->size(), basicRate_);
    ch->frame_type = 0;
    bzero(cx, sizeof(struct hdr_mac_cogmor_control_short));
    STORE4BYTE(&dst, cx->dh_ra);
    STORE4BYTE(&src, cx->dh_ta);
    cx->dh_fc.fc_protocol_version = MAC_CogMOR_ProtocolVersion;
    cx->dh_fc.fc_type = MAC_CogMOR_Type_Control;
    cx->dh_fc.fc_subtype = MAC_CogMOR_Subtype_CON;
    cx->dh_fc.fc_flag = 0;
    cx->dh_fc.fc_more_frag = 0;
    cx->dh_fc.fc_retry = 0;
    cx->dh_fc.fc_order = 0;
    cx->dh_fc.fc_reserved = 0;

    pktCx_ = p;
}

void MacCogMOR::rxHandler() {
    struct hdr_cmn *ch = HDR_CMN(pktRx_);
    struct hdr_mac_cogmor *dh = HDR_MAC_COGMOR(pktRx_);
    struct hdr_mac_cogmor_control_short *cs = ((hdr_mac_cogmor_control_short *)hdr_mac::access(pktRx_));
    struct hdr_mac_cogmor_control_long *cl = ((hdr_mac_cogmor_control_long *)hdr_mac::access(pktRx_));

    int chan = ch->localif();
    double now = Scheduler::instance().clock();
    double duration;

    RxActive_ = Inactive;
    mhRxState_ = Rx_Idle;

    if (TxActive_ == Active) {
        ch->collision() = 1;
    }

    if (tap_ && dh->dh_fc.fc_type == MAC_CogMOR_Type_Data) {
        tap_->tap(pktRx_);
    }

    if (netif_->node()->energy_model() && netif_->node()->energy_model()->adaptivefidelity()) {
        netif_->node()->energy_model()->add_neighbor(ETHER_ADDR(dh->dh_ra));
    }

    switch (ProcessStage_) {
        case Stage_Sx1:
        case Stage_Sx2:
        case Stage_Sx3:
            if (ch->nosise_enabled_ == 1) {
                duration = macmib_.getNIFS();
                if (mhCAVList_.findCAV(chan) == 1) {
                    if ((now + duration) > mhCAVList_.getCAV(chan)) {
                        mhCAVList_.stopCAV(chan);
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }
                } else {
                    mhCAVList_.startCAV(chan, duration, now + duration);
                }
            } else {
                duration = macmib_.getEIFS();
                if (mhCAVList_.findCAV(chan) == 1) {
                    if ((now + duration) > mhCAVList_.getCAV(chan)) {
                        mhCAVList_.stopCAV(chan);
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }
                } else {
                    mhCAVList_.startCAV(chan, duration, now + duration);
                }
            }
            if (CurrChannel_ == chan) {
                DxActive_ = Active;
            }

            Packet::free(pktRx_);
            pktRx_ = NULL;
            break;
        case Stage_Cx:
            if ((ch->error()) || (ch->collision())) {
                duration = macmib_.getEIFS();
                if (mhCAVList_.findCAV(chan) == 1) {
                    if ((now + duration) > mhCAVList_.getCAV(chan)) {
                        mhCAVList_.stopCAV(chan);
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }
                } else {
                    mhCAVList_.startCAV(chan, duration, now + duration);
                }
                CxActive_ = Active;
                Packet::free(pktRx_);
                pktRx_ = NULL;
                return;
            }

            switch (dh->dh_fc.fc_subtype) {
                case MAC_CogMOR_Subtype_CON:
                    CxActive_ = Active;
                    ch->macLayerDuration = Scheduler::instance().clock() - ch->macLayerTimeStamp;
                    mac_log(pktRx_);
                    pktRx_ = NULL;
                    break;
                case MAC_CogMOR_Subtype_PTS:
                case MAC_CogMOR_Subtype_PTR:
                    duration = sec(cl->dh_duration);
                    chan = (int) cl->dh_channel;
                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }

                    Packet::free(pktRx_);
                    pktRx_ = NULL;
                    break;
                case MAC_CogMOR_Subtype_Beacon_B1_U:
                case MAC_CogMOR_Subtype_Beacon_B2_U:
                case MAC_CogMOR_Subtype_Beacon_B1_B:
                case MAC_CogMOR_Subtype_Beacon_B2_B:
                case MAC_CogMOR_Subtype_Data:
                case MAC_CogMOR_Subtype_ACK:
                default:
                    duration = macmib_.getEIFS();

                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }

                    Packet::free(pktRx_);
                    pktRx_ = NULL;
                    break;
            }
            break;
        case Stage_Sx0:
            if ((ch->error()) || (ch->collision())) {
                duration = macmib_.getEIFS();

                if (mhCAVList_.findCAV(chan) == 1) {
                    if ((now + duration) > mhCAVList_.getCAV(chan)) {
                        mhCAVList_.stopCAV(chan);
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }
                } else {
                    mhCAVList_.startCAV(chan, duration, now + duration);
                }

                if (mhStage_.busy()) {
                    mhStage_.stop();
                }
                ProcessStage_ = Stage_Sx0;
                stageStart();
                Packet::free(pktRx_);
                pktRx_ = NULL;
                return;
            }

            switch (dh->dh_fc.fc_subtype) {
                case MAC_CogMOR_Subtype_Beacon_B1_U:
                    ProrityActive_ = Inactive;
                    ReservationChannel_ = chan;
                    ReservationPeriodNum_ = 0;
                    Reservation_src = ETHER_ADDR(cs->dh_ta);
                    Reservation_dst = ETHER_ADDR(cs->dh_ra);

                    duration = T_B2 + T_Sx1 + T_Tu;

                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }

                    if (mhStage_.busy()) {
                        mhStage_.stop();
                    }
                    ProcessStage_ = Stage_B2_U;
                    mhStage_.start(T_B2);

                    if (Reservation_dst == index_) {
                        Rx_address = Reservation_src;
                        Tx_address = index_;

                        mhReplyState_ = Reply_Beacon_B2_U;
                        mhReply_.start(macmib_.getSIFS());
                    }
                    ch->macLayerDuration = Scheduler::instance().clock() - ch->macLayerTimeStamp;
                    mac_log(pktRx_);
                    pktRx_ = NULL;
                    break;
                case MAC_CogMOR_Subtype_Beacon_B1_B:
                    ProrityActive_ = Inactive;
                    ReservationChannel_ = chan;
                    ReservationPeriodNum_ = 0;
                    Reservation_dst = MAC_BROADCAST;
                    Reservation_src = ETHER_ADDR(cs->dh_ta);

                    duration = T_B2 + T_Tb;

                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }

                    if (mhStage_.busy()) {
                        mhStage_.stop();
                    }
                    ProcessStage_ = Stage_B2_B;
                    mhStage_.start(T_B2);

                    Rx_address = Reservation_src;
                    Tx_address = index_;

                    mhReplyState_ = Reply_Beacon_B2_B;
                    mhReply_.start(macmib_.getSIFS());

                    ch->macLayerDuration = Scheduler::instance().clock() - ch->macLayerTimeStamp;
                    mac_log(pktRx_);
                    pktRx_ = NULL;
                    break;
                case MAC_CogMOR_Subtype_Beacon_B2_U:
                    ProrityActive_ = Inactive;
                    ReservationChannel_ = chan;
                    ReservationPeriodNum_ = 0;
                    Reservation_dst = ETHER_ADDR(cs->dh_ta);
                    Reservation_src = ETHER_ADDR(cs->dh_ra);

                    duration = T_Sx1 + T_Tu;

                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }


                    if (mhStage_.busy()) {
                        mhStage_.stop();
                    }
                    ProcessStage_ = Stage_Sx1;
                    mhStage_.start(T_Sx1);

                    ch->macLayerDuration = Scheduler::instance().clock() - ch->macLayerTimeStamp;
                    mac_log(pktRx_);
                    pktRx_ = NULL;
                    break;
                case MAC_CogMOR_Subtype_Beacon_B2_B:
                    ProrityActive_ = Inactive;

                    ReservationChannel_ = chan;
                    ReservationPeriodNum_ = 0;
                    Reservation_dst = MAC_BROADCAST;
                    Reservation_src = ETHER_ADDR(cs->dh_ra);

                    duration = T_Tb;
                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }

                    // broadcast must be transmitter's neighbor
                    // if Beacon B1 is not hear, can only hear Beacon B2, it will not go into MOR scheme
                    if (mhStage_.busy()) {
                        mhStage_.stop();
                    }
                    ProcessStage_ = Stage_Sx0;
                    stageStart();

                    ch->macLayerDuration = Scheduler::instance().clock() - ch->macLayerTimeStamp;
                    mac_log(pktRx_);
                    pktRx_ = NULL;
                    break;
                case MAC_CogMOR_Subtype_PTS:
                case MAC_CogMOR_Subtype_PTR:
                    DataTransferChannel_ = (int) cl->dh_channel;
                    duration = sec(cl->dh_duration);
                    chan = DataTransferChannel_;
                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }

                    if (mhStage_.busy()) {
                        mhStage_.stop();
                    }
                    ProcessStage_ = Stage_Sx0;
                    stageStart();

                    Packet::free(pktRx_);
                    pktRx_ = NULL;
                    break;
                case MAC_CogMOR_Subtype_Data:
                    duration = sec(dh->dh_duration);
                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }

                    if (mhStage_.busy()) {
                        mhStage_.stop();
                    }
                    ProcessStage_ = Stage_Sx0;
                    stageStart();

                    Packet::free(pktRx_);
                    pktRx_ = NULL;
                    break;
                case MAC_CogMOR_Subtype_ACK:
                case MAC_CogMOR_Subtype_CON:
                default:
                    if (mhStage_.busy()) {
                        mhStage_.stop();
                    }
                    ProcessStage_ = Stage_Sx0;
                    stageStart();

                    Packet::free(pktRx_);
                    pktRx_ = NULL;
                    break;
            }
            break;
        case Stage_B2_U:
            if ((ch->error()) || (ch->collision())) {
                duration = macmib_.getEIFS();
                if (mhCAVList_.findCAV(chan) == 1) {
                    if ((now + duration) > mhCAVList_.getCAV(chan)) {
                        mhCAVList_.stopCAV(chan);
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }
                } else {
                    mhCAVList_.startCAV(chan, duration, now + duration);
                }

                Packet::free(pktRx_);
                pktRx_ = NULL;
                return;
            }

            switch (dh->dh_fc.fc_subtype) {
                case MAC_CogMOR_Subtype_Beacon_B2_U:
                    if (ProrityActive_  != Active) {
                        duration = T_Sx1 + T_Tu;
                        if (mhCAVList_.findCAV(chan) == 1) {
                            if ((now + duration) > mhCAVList_.getCAV(chan)) {
                                mhCAVList_.stopCAV(chan);
                                mhCAVList_.startCAV(chan, duration, now + duration);
                            }
                        } else {
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    }

                    if (mhStage_.busy()) {
                        mhStage_.stop();
                    }
                    ProcessStage_ = Stage_Sx1;
                    mhStage_.start(T_Sx1);

                    ch->macLayerDuration = Scheduler::instance().clock() - ch->macLayerTimeStamp;
                    mac_log(pktRx_);
                    pktRx_ = NULL;
                    break;
                case MAC_CogMOR_Subtype_PTS:
                case MAC_CogMOR_Subtype_PTR:
                    duration = sec(cl->dh_duration);
                    chan = (int) cl->dh_channel;
                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }

                    Packet::free(pktRx_);
                    pktRx_ = NULL;
                    break;
                case MAC_CogMOR_Subtype_Beacon_B1_U:
                case MAC_CogMOR_Subtype_Beacon_B1_B:
                case MAC_CogMOR_Subtype_Beacon_B2_B:
                case MAC_CogMOR_Subtype_Data:
                case MAC_CogMOR_Subtype_ACK:
                case MAC_CogMOR_Subtype_CON:
                default:
                    duration = macmib_.getEIFS();
                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }

                    Packet::free(pktRx_);
                    pktRx_ = NULL;
                    break;
            }
            break;
        case Stage_B2_B:
            if (ch->error()) {
                duration = macmib_.getEIFS();
                if (mhCAVList_.findCAV(chan) == 1) {
                    if ((now + duration) > mhCAVList_.getCAV(chan)) {
                        mhCAVList_.stopCAV(chan);
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }
                } else {
                    mhCAVList_.startCAV(chan, duration, now + duration);
                }

                Packet::free(pktRx_);
                pktRx_ = NULL;
                return;
            }

            if (ch->collision()) {
                if (ProrityActive_ == Active) {
                    if (mhStage_.busy()) {
                        mhStage_.stop();
                    }
                    ProcessStage_ = Stage_Tb;
                    mhStage_.start(T_Tb);
                }
                Packet::free(pktRx_);
                pktRx_ = NULL;
                return;
            }

            switch (dh->dh_fc.fc_subtype) {
                case MAC_CogMOR_Subtype_Beacon_B2_B:
                    if (ProrityActive_ != Active) {
                        duration = T_Tb;

                        if (mhCAVList_.findCAV(chan) == 1) {
                            if ((now + duration) > mhCAVList_.getCAV(chan)) {
                                mhCAVList_.stopCAV(chan);
                                mhCAVList_.startCAV(chan, duration, now + duration);
                            }
                        } else {
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    }

                    if (ProrityActive_ == Active) {
                        if (mhStage_.busy()) {
                            mhStage_.stop();
                        }
                        ProcessStage_ = Stage_Tb;
                        mhStage_.start(T_Tb);
                    }
                    ch->macLayerDuration = Scheduler::instance().clock() - ch->macLayerTimeStamp;
                    mac_log(pktRx_);
                    pktRx_ = NULL;
                    break;
                case MAC_CogMOR_Subtype_PTS:
                case MAC_CogMOR_Subtype_PTR:
                    duration = sec(cl->dh_duration);
                    chan = (int) cl->dh_channel;
                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }

                    Packet::free(pktRx_);
                    pktRx_ = NULL;
                    break;
                case MAC_CogMOR_Subtype_Beacon_B1_U:
                case MAC_CogMOR_Subtype_Beacon_B1_B:
                case MAC_CogMOR_Subtype_Beacon_B2_U:
                case MAC_CogMOR_Subtype_Data:
                case MAC_CogMOR_Subtype_ACK:
                case MAC_CogMOR_Subtype_CON:
                default:
                    duration = macmib_.getEIFS();
                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }

                    Packet::free(pktRx_);
                    pktRx_ = NULL;
                    break;
            }
            break;
        case Stage_Tu:
        case Stage_Ru:
            if ((ch->error()) || (ch->collision())) {
                duration = macmib_.getEIFS();
                if (mhCAVList_.findCAV(chan) == 1) {
                    if ((now + duration) > mhCAVList_.getCAV(chan)) {
                        mhCAVList_.stopCAV(chan);
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }
                } else {
                    mhCAVList_.startCAV(chan, duration, now + duration);
                }

                Packet::free(pktRx_);
                pktRx_ = NULL;
                return;
            }

            switch (dh->dh_fc.fc_subtype) {
                case MAC_CogMOR_Subtype_PTS:
                    CxActive_ = Active;
                    DataTransferChannel_ = (int) cl->dh_channel;
                    DataTransferPeriodNum_ = 0;
                    DataTransfer_dst = ETHER_ADDR(cl->dh_ra);
                    DataTransfer_src = ETHER_ADDR(cl->dh_ta);

                    duration = sec(cl->dh_duration);
                    chan = DataTransferChannel_;
                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }

                    if (mhStage_.busy()) {
                        mhStage_.stop();
                    }
                    ProcessStage_ = Stage_Ru;
                    mhStage_.start(T_Ru);

                    if (DataTransfer_dst == index_) {
                        Rx_address = DataTransfer_src;
                        Tx_address = DataTransfer_dst;

                        mhReplyState_ = Reply_PTR;
                        Reply_PTR_timeout = cl->dh_duration - usec(macmib_.getSIFS()
                                                                   + MacCogMORPropagationDelay
                                                                   + txtime(macmib_.getPTRlen(), basicRate_));
                        mhReply_.start(macmib_.getSIFS());
                    } else {

                    }
                    ch->macLayerDuration = Scheduler::instance().clock() - ch->macLayerTimeStamp;
                    mac_log(pktRx_);
                    pktRx_ = NULL;
                    break;
                case MAC_CogMOR_Subtype_PTR:
                    CxActive_ = Active;
                    DataTransferChannel_ = (int) cl->dh_channel;
                    DataTransferPeriodNum_ = 0;
                    DataTransfer_dst = ETHER_ADDR(cl->dh_ta);
                    DataTransfer_src = ETHER_ADDR(cl->dh_ra);

                    if (DataTransfer_src != index_) {
                        duration = sec(cl->dh_duration);
                        chan = DataTransferChannel_;
                        if (mhCAVList_.findCAV(chan) == 1) {
                            if ((now + duration) > mhCAVList_.getCAV(chan)) {
                                mhCAVList_.stopCAV(chan);
                                mhCAVList_.startCAV(chan, duration, now + duration);
                            }
                        } else {
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    }


                    if ((DataTransfer_src == SendType_dst) || (DataTransfer_dst == SendType_dst)) {
                        if (mhStage_.busy()) {
                            mhStage_.stop();
                        }
                        ProcessStage_ = Stage_Sx3;
                        mhStage_.start(T_Sx3);
                    } else {
                        if (ReservationPeriodNum_ == MaxReservationPeriodNum_) {
                            if (mhStage_.busy()) {
                                mhStage_.stop();
                            }
                            ProcessStage_ = Stage_Sx0;
                            stageStart();
                        } else {
                            if (mhStage_.busy()) {
                                mhStage_.stop();
                            }
                            ProcessStage_ = Stage_Sx2;
                            mhStage_.start(T_Sx2);
                        }
                    }
                    ch->macLayerDuration = Scheduler::instance().clock() - ch->macLayerTimeStamp;
                    mac_log(pktRx_);
                    pktRx_ = NULL;
                    break;
                case MAC_CogMOR_Subtype_Beacon_B1_U:
                case MAC_CogMOR_Subtype_Beacon_B2_U:
                case MAC_CogMOR_Subtype_Beacon_B1_B:
                case MAC_CogMOR_Subtype_Beacon_B2_B:
                case MAC_CogMOR_Subtype_Data:
                case MAC_CogMOR_Subtype_ACK:
                case MAC_CogMOR_Subtype_CON:
                default:
                    duration = macmib_.getEIFS();
                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }

                    Packet::free(pktRx_);
                    pktRx_ = NULL;
                    break;
            }
            break;
        case Stage_Tb:
            if ((ch->error()) || (ch->collision())) {
                duration = macmib_.getEIFS();
                if (mhCAVList_.findCAV(chan) == 1) {
                    if ((now + duration) > mhCAVList_.getCAV(chan)) {
                        mhCAVList_.stopCAV(chan);
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }
                } else {
                    mhCAVList_.startCAV(chan, duration, now + duration);
                }

                Packet::free(pktRx_);
                pktRx_ = NULL;
                return;
            }

            switch (dh->dh_fc.fc_subtype) {
                case MAC_CogMOR_Subtype_Data:
                    duration = macmib_.getSIFS();
                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }

                    if (mhStage_.busy()) {
                        mhStage_.stop();
                    }
                    ProcessStage_ = Stage_Sx0;
                    stageStart();

                    ch->size() -= macmib_.getHdrLen();
                    ch->num_forwards() += 1;
                    ch->macLayerDuration = Scheduler::instance().clock() - ch->macLayerTimeStamp;
                    uptarget_->recv(pktRx_, (Handler *) 0);
                    pktRx_ = NULL;
                    return;
                case MAC_CogMOR_Subtype_PTS:
                case MAC_CogMOR_Subtype_PTR:
                    duration = sec(cl->dh_duration);
                    chan = (int) cl->dh_channel;
                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }

                    Packet::free(pktRx_);
                    pktRx_ = NULL;
                    break;
                case MAC_CogMOR_Subtype_Beacon_B1_U:
                case MAC_CogMOR_Subtype_Beacon_B2_U:
                case MAC_CogMOR_Subtype_Beacon_B1_B:
                case MAC_CogMOR_Subtype_Beacon_B2_B:
                case MAC_CogMOR_Subtype_ACK:
                case MAC_CogMOR_Subtype_CON:
                default:
                    duration = macmib_.getEIFS();
                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }

                    Packet::free(pktRx_);
                    pktRx_ = NULL;
                    break;
            }
            break;
        case Stage_Tx:
        case Stage_Rx:
            if ((ch->error()) || (ch->collision())) {
                duration = macmib_.getEIFS();
                if (mhCAVList_.findCAV(chan) == 1) {
                    if ((now + duration) > mhCAVList_.getCAV(chan)) {
                        mhCAVList_.stopCAV(chan);
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }
                } else {
                    mhCAVList_.startCAV(chan, duration, now + duration);
                }

                Packet::free(pktRx_);
                pktRx_ = NULL;
                return;
            }

            switch (dh->dh_fc.fc_subtype) {
                case MAC_CogMOR_Subtype_Data:
                    duration = sec(dh->dh_duration);
                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }

                    CxActive_ = Active;

                    if (ETHER_ADDR(dh->dh_ra) == index_) {
                        Rx_address = ETHER_ADDR(dh->dh_ta);
                        Tx_address = ETHER_ADDR(dh->dh_ra);

                        mhReplyState_ = Reply_ACK;
                        mhReply_.start(macmib_.getSIFS());

                        ch->size() -= macmib_.getHdrLen();
                        ch->num_forwards() += 1;
                        ch->macLayerDuration = Scheduler::instance().clock() - ch->macLayerTimeStamp;
                        uptarget_->recv(pktRx_, (Handler *) 0);
                        pktRx_ = NULL;
                    }

                    if (mhStage_.busy()) {
                        mhStage_.stop();
                    }
                    ProcessStage_ = Stage_Rx;
                    mhStage_.start(T_Rx);
                    return;
                case MAC_CogMOR_Subtype_ACK:
                    CxActive_ = Active;
                    if (ETHER_ADDR(cs->dh_ra) == index_) {
                        RetryCount_ = 0;
                        RetryShortCount_ = 0;
                        macmib_.rst_cw();
                        Packet::free(pktTx_);
                        pktTx_ = NULL;
                        SendType_ = SendType_Null;
                        mhTimeout_.stop();

                        if (callback_) {
                            Handler *h = callback_;
                            callback_ = 0;
                            h->handle((Event *) 0);
                        }

                    }

                    if (DataTransferPeriodNum_ < MaxDataTransferPeriodNum_) {
                        if (mhStage_.busy()) {
                            mhStage_.stop();
                        }
                        ProcessStage_ = Stage_Cx;
                        mhStage_.start(T_Cx);
                    } else {
                        if (mhStage_.busy()) {
                            mhStage_.stop();
                        }
                        ProcessStage_ = Stage_Sx0;
                        stageStart();
                    }
                    ch->macLayerDuration = Scheduler::instance().clock() - ch->macLayerTimeStamp;
                    mac_log(pktRx_);
                    pktRx_ = NULL;
                    break;
                case MAC_CogMOR_Subtype_PTS:
                case MAC_CogMOR_Subtype_PTR:
                    duration = sec(cl->dh_duration);
                    chan = (int) cl->dh_channel;
                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }

                    Packet::free(pktRx_);
                    pktRx_ = NULL;
                    break;
                case MAC_CogMOR_Subtype_Beacon_B1_U:
                case MAC_CogMOR_Subtype_Beacon_B2_U:
                case MAC_CogMOR_Subtype_Beacon_B1_B:
                case MAC_CogMOR_Subtype_Beacon_B2_B:
                case MAC_CogMOR_Subtype_CON:
                default:
                    duration = macmib_.getEIFS();
                    if (mhCAVList_.findCAV(chan) == 1) {
                        if ((now + duration) > mhCAVList_.getCAV(chan)) {
                            mhCAVList_.stopCAV(chan);
                            mhCAVList_.startCAV(chan, duration, now + duration);
                        }
                    } else {
                        mhCAVList_.startCAV(chan, duration, now + duration);
                    }

                    Packet::free(pktRx_);
                    pktRx_ = NULL;
                    break;
            }
            break;
        case Stage_B1_U:
        case Stage_B1_B:
        default:
            Packet::free(pktRx_);
            pktRx_ = NULL;
            fprintf(stderr,"node[%d] - rxHandler() ERROR ProcessStage_\n",index_);
            exit(1);
    }
}

void MacCogMOR::stageHandler(void) {
    struct hdr_cmn *ch;
    switch (ProcessStage_) {
        case Stage_B2_U:
            if (mhStage_.busy()) {
                mhStage_.stop();
            }
            ProcessStage_ = Stage_Sx0;
            stageStart();
            break;
        case Stage_B2_B:
            if (mhStage_.busy()) {
                mhStage_.stop();
            }
            ProcessStage_ = Stage_Sx0;
            stageStart();
            break;
        case Stage_Sx1:
            if (DxActive_ == Active) {
                if (SendType_ == SendType_Unicast) {
                    if ((Reservation_src == SendType_dst) || (Reservation_dst == SendType_dst)) {
                        RetryShortCount_++;
                        if (RetryShortCount_ >= macmib_.getRetryShortLimit()) {
                            ch = HDR_CMN(pktTx_);
                            macmib_.FailedCount++;
                            RetryCount_ = 0;
                            RetryShortCount_ = 0;
                            macmib_.rst_cw();

                            if (ch->xmit_failure_) {
                                ch->size() -= macmib_.getHdrLen();
                                ch->xmit_reason_ = XMIT_REASON_ACK;
                                ch->xmit_failure_(pktTx_->copy(), ch->xmit_failure_data_);
                            }

                            drop(pktTx_, DROP_MAC_RETRY_COUNT_EXCEEDED);
                            pktTx_ = NULL;
                            SendType_ = SendType_Null;
                            mhTimeout_.stop();

                            if (callback_) {
                                Handler *h = callback_;
                                callback_ = 0;
                                h->handle((Event *) 0);
                            }
                        }
                    }
                }
                if (mhStage_.busy()) {
                    mhStage_.stop();
                }
                ProcessStage_ = Stage_Sx0;
                stageStart();
            } else {
                if (mhStage_.busy()) {
                    mhStage_.stop();
                }
                ProcessStage_ = Stage_Tu;
                mhStage_.start(T_Tu + T_Ru);

                ReservationPeriodNum_ = 1;
            }
            break;
        case Stage_Sx2:
            if (DxActive_ == Active) {
                if(SendType_ == SendType_Unicast) {
                    if ((Reservation_src == SendType_dst) || (Reservation_dst == SendType_dst)) {

                        RetryShortCount_++;
                        if (RetryShortCount_ >= macmib_.getRetryShortLimit()) {
                            ch = HDR_CMN(pktTx_);
                            macmib_.FailedCount++;
                            RetryCount_ = 0;
                            RetryShortCount_ = 0;
                            macmib_.rst_cw();

                            if (ch->xmit_failure_) {
                                ch->size() -= macmib_.getHdrLen();
                                ch->xmit_reason_ = XMIT_REASON_ACK;
                                ch->xmit_failure_(pktTx_->copy(), ch->xmit_failure_data_);
                            }

                            drop(pktTx_, DROP_MAC_RETRY_COUNT_EXCEEDED);
                            pktTx_ = NULL;
                            SendType_ = SendType_Null;
                            mhTimeout_.stop();

                            if (callback_) {
                                Handler *h = callback_;
                                callback_ = 0;
                                h->handle((Event *) 0);
                            }
                        }
                    }
                }
                if (mhStage_.busy()) {
                    mhStage_.stop();
                }
                ProcessStage_ = Stage_Sx0;
                stageStart();
            } else {
                if (mhStage_.busy()) {
                    mhStage_.stop();
                }
                ProcessStage_ = Stage_Tu;
                mhStage_.start(T_Tu + T_Ru);
            }
            break;
        case Stage_Sx3:
            if (DxActive_ == Active) {
                if (SendType_ == SendType_Broadcast) {
                    if ((DataTransfer_src == SendType_dst) || (DataTransfer_dst == SendType_dst)) {

                        RetryCount_++;
                        if (RetryCount_ >= macmib_.getRetryLimit()) {
                            ch = HDR_CMN(pktTx_);
                            macmib_.FailedCount++;
                            RetryShortCount_ = 0;
                            RetryCount_ = 0;
                            macmib_.rst_cw();

                            if (ch->xmit_failure_) {
                                ch->size() -= macmib_.getHdrLen();
                                ch->xmit_reason_ = XMIT_REASON_ACK;
                                ch->xmit_failure_(pktTx_->copy(), ch->xmit_failure_data_);
                            }

                            drop(pktTx_, DROP_MAC_RETRY_COUNT_EXCEEDED);
                            pktTx_ = NULL;
                            SendType_ = SendType_Null;
                            mhTimeout_.stop();

                            if (callback_) {
                                Handler *h = callback_;
                                callback_ = 0;
                                h->handle((Event *) 0);
                            }
                        }
                    }
                }
                if (mhStage_.busy()) {
                    mhStage_.stop();
                }
                ProcessStage_ = Stage_Sx0;
                stageStart();
            } else {
                if (mhStage_.busy()) {
                    mhStage_.stop();
                }
                ProcessStage_ = Stage_Tx;
                mhStage_.start(T_Tx + T_Rx);
            }
            break;
        case Stage_Tb:
            if (SendType_ == SendType_Broadcast) {
                if ((Reservation_src == SendType_src) && (Reservation_dst == SendType_dst)) {
                    RetryShortCount_++;
                    if (RetryShortCount_ >= macmib_.getRetryShortLimit()) {
                        ch = HDR_CMN(pktTx_);
                        macmib_.FailedCount++;
                        RetryShortCount_ = 0;
                        RetryCount_ = 0;
                        macmib_.rst_cw();

                        if (ch->xmit_failure_) {
                            ch->size() -= macmib_.getHdrLen();
                            ch->xmit_reason_ = XMIT_REASON_ACK;
                            ch->xmit_failure_(pktTx_->copy(), ch->xmit_failure_data_);
                        }

                        drop(pktTx_, DROP_MAC_RETRY_COUNT_EXCEEDED);
                        pktTx_ = NULL;
                        SendType_ = SendType_Null;
                        mhTimeout_.stop();

                        if (callback_) {
                            Handler *h = callback_;
                            callback_ = 0;
                            h->handle((Event *) 0);
                        }
                    }
                }
            }

            if (mhStage_.busy()) {
                mhStage_.stop();
            }
            ProcessStage_ = Stage_Sx0;
            stageStart();
            break;
        case Stage_Tu:
        case Stage_Ru:
            if (CxActive_ == Active) {
                if (ReservationPeriodNum_ == MaxReservationPeriodNum_) {
                    ProcessStage_ = Stage_Sx0;
                    stageStart();
                } else {
                    ProcessStage_ = Stage_Sx2;
                    mhStage_.start(T_Sx2);
                }
                return;
            }
            if (SendType_ == SendType_Unicast) {
                if ((Reservation_src == SendType_dst) || (Reservation_dst == SendType_dst)) {
                    RetryShortCount_++;
                    if (RetryShortCount_ >= macmib_.getRetryShortLimit()) {
                        ch = HDR_CMN(pktTx_);
                        macmib_.FailedCount++;
                        RetryShortCount_ = 0;
                        RetryCount_ = 0;
                        macmib_.rst_cw();

                        if (ch->xmit_failure_) {
                            ch->size() -= macmib_.getHdrLen();
                            ch->xmit_reason_ = XMIT_REASON_ACK;
                            ch->xmit_failure_(pktTx_->copy(), ch->xmit_failure_data_);
                        }

                        drop(pktTx_, DROP_MAC_RETRY_COUNT_EXCEEDED);
                        pktTx_ = NULL;
                        SendType_ = SendType_Null;
                        mhTimeout_.stop();

                        if (callback_) {
                            Handler *h = callback_;
                            callback_ = 0;
                            h->handle((Event *) 0);
                        }
                    }
                }
            }
            if (mhStage_.busy()) {
                mhStage_.stop();
            }
            ProcessStage_ = Stage_Sx0;
            stageStart();
            break;
        case Stage_Tx:
        case Stage_Rx:
            if (CxActive_ == Active) {
                if (DataTransferPeriodNum_ == MaxDataTransferPeriodNum_) {
                    ProcessStage_ = Stage_Sx0;
                    stageStart();
                     
                } else {
                    ProcessStage_ = Stage_Sx3;
                    mhStage_.start(T_Sx3);
                }
                break;
            }

            if (SendType_ == SendType_Unicast) {
                if ((DataTransfer_src == SendType_dst) || (DataTransfer_dst == SendType_dst)) {
                    RetryCount_++;
                    if (RetryCount_ >= macmib_.getRetryLimit()) {
                        ch = HDR_CMN(pktTx_);
                        macmib_.FailedCount++;
                        RetryShortCount_ = 0;
                        RetryCount_ = 0;
                        macmib_.rst_cw();

                        if (ch->xmit_failure_) {
                            ch->size() -= macmib_.getHdrLen();
                            ch->xmit_reason_ = XMIT_REASON_ACK;
                            ch->xmit_failure_(pktTx_->copy(), ch->xmit_failure_data_);
                        }

                        drop(pktTx_, DROP_MAC_RETRY_COUNT_EXCEEDED);
                        pktTx_ = NULL;
                        SendType_ = SendType_Null;
                        mhTimeout_.stop();

                        if (callback_) {
                            Handler *h = callback_;
                            callback_ = 0;
                            h->handle((Event *) 0);
                        }
                    }
                }
            }
            if (mhStage_.busy()) {
                mhStage_.stop();
            }
            ProcessStage_ = Stage_Sx0;
            stageStart();
            break;
        case Stage_Cx:
            if ((CxActive_ == Active) && ((DataTransferPeriodNum_ + 1) <= MaxDataTransferPeriodNum_)) {
                if (mhStage_.busy()) {
                    mhStage_.stop();
                }
                ProcessStage_ = Stage_Sx3;
                mhStage_.start(T_Sx3);
                return;
            }

            if (SendType_ == SendType_Unicast) {
                if ((DataTransfer_src == SendType_dst) || (DataTransfer_dst == SendType_dst)) {
                    RetryCount_++;
                    if (RetryCount_ >= macmib_.getRetryLimit()) {
                        ch = HDR_CMN(pktTx_);
                        macmib_.FailedCount++;
                        RetryShortCount_ = 0;
                        RetryCount_ = 0;
                        macmib_.rst_cw();

                        if (ch->xmit_failure_) {
                            ch->size() -= macmib_.getHdrLen();
                            ch->xmit_reason_ = XMIT_REASON_ACK;
                            ch->xmit_failure_(pktTx_->copy(), ch->xmit_failure_data_);
                        }

                        drop(pktTx_, DROP_MAC_RETRY_COUNT_EXCEEDED);
                        pktTx_ = NULL;
                        SendType_ = SendType_Null;
                        mhTimeout_.stop();

                        if (callback_) {
                            Handler *h = callback_;
                            callback_ = 0;
                            h->handle((Event *) 0);
                        }
                    }
                }
            }
            if (mhStage_.busy()) {
                mhStage_.stop();
            }
            ProcessStage_ = Stage_Sx0;
            stageStart();
            break;
        case Stage_Sx0:
        case Stage_B1_B:
        case Stage_B1_U:
        default:
            fprintf(stderr, "node[%d] - void stageHandler() ERROR ProcessStage_\n", index_);
            exit(1);
    }
}

void MacCogMOR::stageStart() {
    switch (ProcessStage_) {
        case Stage_Sx0:

            transceiverMode_ = Sensing;
            CurrChannel_ = -2;
            ReservationChannel_ = -2;

            Reservation_src = -2;
            Reservation_dst = -2;

            init_channel_list();

            if (mhUIFS_.busy() == 0) {
                mhUIFS_.start(getUIFS());
            }

            if (mhBIFS_.busy() == 0) {
                mhBIFS_.start(getBIFS());
            }

            if (mhBackoff_.busy()) {
                mhBackoff_.stop();
            }
            break;
        case Stage_B1_U:
        case Stage_B1_B:
            transceiverMode_ = Transmitting;
            CurrChannel_ = ReservationChannel_;
            if (mhBackoff_.busy()) {
                mhBackoff_.stop();
            }
            break;
        case Stage_B2_U:
        case Stage_B2_B:
            transceiverMode_ = Receiving;
            CurrChannel_ = ReservationChannel_;
            if (mhBackoff_.busy()) {
                mhBackoff_.stop();
            }
            break;
        case Stage_Sx1:

            transceiverMode_ = Sensing;
            CurrChannel_ = ReservationChannel_;

            DataTransfer_src = -2;
            DataTransfer_dst = -2;

            DataTransferChannel_ = -2;

            ReservationPeriodNum_ = 0;
            DataTransferPeriodNum_ = 0;

            if (mhBackoff_.busy()) {
                mhBackoff_.stop();
            }
            DxActive_ = Inactive;
            CxActive_ = Inactive;
            ReservationPeriodNum_++;
            break;
        case Stage_Sx2:

            transceiverMode_ = Receiving;
            CurrChannel_ = ReservationChannel_;

            if (mhBackoff_.busy()) {
                mhBackoff_.stop();
            }

            DxActive_ = Inactive;
            CxActive_ = Inactive;
            ReservationPeriodNum_++;
            break;
        case Stage_Sx3:

            transceiverMode_ = Receiving;
            CurrChannel_ = DataTransferChannel_;

            if (mhBackoff_.busy()) {
                mhBackoff_.stop();
            }

            DxActive_ = Inactive;
            CxActive_ = Inactive;
            DataTransferPeriodNum_++;
            break;
        case Stage_Tu:
            transceiverMode_ = Receiving;
            CurrChannel_ = ReservationChannel_;
            if (mhBackoff_.busy()) {
                mhBackoff_.stop();
            }

            if (SendType_ == SendType_Unicast) {
                if ((ReservationPeriodNum_ == 1) && (ProrityActive_ == Active)) {
                    if (is_idle(CurrChannel_)) {

                        mhDeferState_ = Defer_PTS;
                        mhDefer_.start(macmib_.getSIFS());
                    }
                    ProrityActive_ = Inactive;
                } else if ((1 < ReservationPeriodNum_) && (ReservationPeriodNum_ <= MaxReservationPeriodNum_)) {
                    if (is_idle(CurrChannel_)) {
                        mhDeferState_ = Defer_PTS;
                        mhDefer_.start(getUIFS());
                    }
                }
            }

            init_channel_list();
            remove_channel_list(CurrChannel_);
            DataTransferChannel_ = selectDC();
            if (mhDefer_.busy() == 0) {
                remove_channel_list(DataTransferChannel_);
            }
            break;
        case Stage_Tb:
            transceiverMode_ = Receiving;
            CurrChannel_ = ReservationChannel_;
            if (mhBackoff_.busy()) {
                mhBackoff_.stop();
            }
            if (SendType_ == SendType_Broadcast) {
                if (ProrityActive_ == Active) {
                    if (is_idle(CurrChannel_)) {
                        mhDeferState_ = Defer_DATA_B;
                        mhDefer_.start(macmib_.getSIFS());
                    }
                }
            }
            break;
        case Stage_Tx:
            transceiverMode_ = Receiving;
            CurrChannel_ = DataTransferChannel_;
            if (mhBackoff_.busy()) {
                mhBackoff_.stop();
            }

            if (SendType_ == SendType_Unicast) {
                if ((DataTransferPeriodNum_ == 1) && (DataTransfer_src == index_)) {
                    if (is_idle(CurrChannel_)) {
                        mhDeferState_ = Defer_DATA_U;
                        mhDefer_.start(getUIFS());
                    }
                } else if ((1 < DataTransferPeriodNum_) && (DataTransferPeriodNum_ <= MaxDataTransferPeriodNum_)) {
                    if (is_idle(CurrChannel_)) {
                        mhDeferState_ = Defer_DATA_U;
                        mhDefer_.start(getUIFS());
                    }
                }
            }
            break;
        case Stage_Ru:
            transceiverMode_ = Receiving;
            CurrChannel_ = ReservationChannel_;

            if (mhBackoff_.busy()) {
                mhBackoff_.stop();
            }
            break;
        case Stage_Rx:
            transceiverMode_ = Receiving;
            CurrChannel_ = DataTransferChannel_;

            if (mhBackoff_.busy()) {
                mhBackoff_.stop();
            }
            break;
        case Stage_Cx:
            transceiverMode_ = Receiving;
            CurrChannel_ = DataTransferChannel_;

            if (mhBackoff_.busy()) {
                mhBackoff_.stop();
            }

            CxActive_ = Inactive;
            switch (SendType_) {
                case SendType_Null:
                case SendType_Broadcast:
                    break;
                case SendType_Unicast:
                    Rx_address = SendType_dst;
                    Tx_address = SendType_src;
                    mhReplyState_ = Reply_CON;
                    mhReply_.start(macmib_.getSIFS());
                    break;
                default:
                    fprintf(stderr, "node[%d] - void MacCogMOR::stageStart() ERROR SendState_\n", index_);
                    exit(1);
            }
            break;
        default:
            fprintf(stderr, "node[%d] - void MacCogMOR::stageHandler() ERROR ProcessStage_\n", index_);
            exit(1);
    }
}

void MacCogMOR::timeoutHandler(void) {
    struct hdr_cmn *ch;
    ch = HDR_CMN(pktTx_);
    macmib_.FailedCount++;
    RetryShortCount_ = 0;
    RetryCount_ = 0;
    macmib_.rst_cw();

    if (ch->xmit_failure_) {
        ch->size() -= macmib_.getHdrLen();
        ch->xmit_reason_ = XMIT_REASON_ACK;
        ch->xmit_failure_(pktTx_->copy(), ch->xmit_failure_data_);
    }

    drop(pktTx_, DROP_MAC_RETRY_COUNT_EXCEEDED);
    pktTx_ = NULL;
    SendType_ = SendType_Null;

    if (callback_) {
        Handler *h = callback_;
        callback_ = 0;
        h->handle((Event *) 0);
    }
}

void MacCogMOR::backoffHandler(void) {
    std::list<ChannelInfo>::iterator it;
    switch (ProcessStage_) {
        case Stage_Sx0:
            init_channel_list();
            ReservationChannel_ = selectRC();

            Reservation_src = SendType_src;
            Reservation_dst = SendType_dst;

            ReservationPeriodNum_ = 1;

            DataTransfer_dst = -2;
            DataTransfer_src = -2;

            DataTransferChannel_ = -2;
            DataTransferPeriodNum_ = 1;
            switch (SendType_) {
                case SendType_Unicast:
                    if (mhUIFS_.busy() == 0) {
                        if (is_idle(ReservationChannel_)) {
                            ProcessStage_ = Stage_B1_U;
                            stageStart();

                            mhTxState_ = Tx_Beacon_B1_U;
                            sendBeacon(SendType_dst,SendType_src,Beacon_B1_U);
                            transmit(pktBeacon_, CurrChannel_);
                            ProrityActive_ = Active;
                            fprintf(stderr,"node[%d] - transmit():Beacon_B1 -dst:%d -src:%d -chan:%d -time:%f\n", index_, SendType_dst , SendType_src, CurrChannel_, Scheduler::instance().clock());

                        } else {
                            //macmib_.inc_cw();
                            mhBackoff_.start(getUIFS());
                        }
                    }
                    break;
                case SendType_Broadcast:
                    if (mhBIFS_.busy() == 0) {
                        if (is_idle(ReservationChannel_)) {
                            ProcessStage_ = Stage_B1_B;
                            stageStart();

                            mhTxState_ = Tx_Beacon_B1_B;
                            sendBeacon(SendType_dst,SendType_src,Beacon_B1_B);
                            transmit(pktBeacon_, CurrChannel_);
                            ProrityActive_ = Active;
                            fprintf(stderr,"node[%d] - transmit():Beacon_B1 -dst:%d -src:%d -chan:%d -time:%f\n", index_, SendType_dst , SendType_src, CurrChannel_, Scheduler::instance().clock());

                        } else {
                            //macmib_.inc_cw();
                            mhBackoff_.start(getBIFS());
                        }
                    }
                    break;
                default:
                    fprintf(stderr, "node[%d] - void MacCogMOR::send ERROR SendType_ \n", index_);
                    exit(1);
            }
            break;
        default:
            break;
    }
}

void MacCogMOR::txHandler() {
    if (EOTtarget_) {
        if (eotPacket_ == NULL) {
            fprintf(stderr, "node[%d] - txHandler() ERROR eotPacket_ == NULL) \n", index_);
            exit(1);
        }
        EOTtarget_->recv(eotPacket_, (Handler *) 0);
        eotPacket_ = NULL;
    }
    if (nodeWorkMode_ == NodeWorkMode_Interference_Source) {
        mhTxState_ = Tx_Idle;
        TxActive_ = Inactive;

        transceiverMode_ = Receiving;

        Packet::free(pktTx_);
        pktTx_ = NULL;
        SendType_ = SendType_Null;

        if (mhPu_.busy() == 0) {
            mhPu_.start(PU_interval);
        }
        return;
    }

    if (nodeWorkMode_ == NodeWorkMode_Cognitive_Node) {
        switch (mhTxState_) {
            case Tx_Beacon_B1_U:
                mhTxState_ = Tx_Idle;
                TxActive_ = Inactive;

                if (mhStage_.busy()) {
                    mhStage_.stop();
                }
                ProcessStage_ = Stage_B2_U;
                mhStage_.start(T_B2);
                Packet::free(pktBeacon_);
                pktBeacon_ = NULL;
                break;
            case Tx_Beacon_B1_B:
                mhTxState_ = Tx_Idle;
                TxActive_ = Inactive;

                if (mhStage_.busy()) {
                    mhStage_.stop();
                }
                ProcessStage_ = Stage_B2_B;
                mhStage_.start(T_B2);
                Packet::free(pktBeacon_);
                pktBeacon_ = NULL;
                break;
            case Tx_Beacon_B2_U:
                mhTxState_ = Tx_Idle;
                TxActive_ = Inactive;

                if (mhStage_.busy()) {
                    mhStage_.stop();
                }
                ProcessStage_ = Stage_Sx1;
                mhStage_.start(T_Sx1);
                Packet::free(pktBeacon_);
                pktBeacon_ = NULL;
                break;
            case Tx_Beacon_B2_B:
                mhTxState_ = Tx_Idle;
                TxActive_ = Inactive;

                if (mhStage_.busy()) {
                    mhStage_.stop();
                }
                ProcessStage_ = Stage_Tb;
                mhStage_.start(T_Tb);
                Packet::free(pktBeacon_);
                pktBeacon_ = NULL;
                break;
            case Tx_PTS:
                mhTxState_ = Tx_Idle;
                TxActive_ = Inactive;

                if (mhStage_.busy()) {
                    mhStage_.stop();
                }
                ProcessStage_ = Stage_Ru;
                mhStage_.start(T_Ru);
                Packet::free(pktPTS_);
                pktPTS_ = NULL;
                break;
            case Tx_DATA_B:
                mhTxState_ = Tx_Idle;
                TxActive_ = Inactive;

                RetryCount_ = 0;
                RetryShortCount_ = 0;
                macmib_.rst_cw();

                Packet::free(pktTx_);
                pktTx_ = NULL;
                SendType_ = SendType_Null;
                mhTimeout_.stop();

                if (callback_) {
                    Handler *h = callback_;
                    callback_ = 0;
                    h->handle((Event *) 0);
                }

                if (mhStage_.busy()) {
                    mhStage_.stop();
                }
                ProcessStage_ = Stage_Sx0;
                stageStart();

                break;
            case Tx_DATA_U:
                mhTxState_ = Tx_Idle;
                TxActive_ = Inactive;

                if (mhStage_.busy()) {
                    mhStage_.stop();
                }
                ProcessStage_ = Stage_Rx;
                mhStage_.start(T_Rx);
                break;
            case Tx_PTR:
                mhTxState_ = Tx_Idle;
                TxActive_ = Inactive;

                if (ReservationPeriodNum_ < MaxReservationPeriodNum_) {
                    if (mhStage_.busy()) {
                        mhStage_.stop();
                    }
                    ProcessStage_ = Stage_Sx3;
                    mhStage_.start(T_Sx3);
                } else {
                    if (mhStage_.busy()) {
                        mhStage_.stop();
                    }
                    ProcessStage_ = Stage_Sx0;
                    stageStart();
                }

                Packet::free(pktPTR_);
                pktPTR_ = NULL;
                break;
            case Tx_ACK:
                mhTxState_ = Tx_Idle;
                TxActive_ = Inactive;

                if (DataTransferPeriodNum_ < MaxDataTransferPeriodNum_) {
                    if (mhStage_.busy()) {
                        mhStage_.stop();
                    }
                    ProcessStage_ = Stage_Cx;
                    mhStage_.start(T_Cx);
                } else {
                    if (mhStage_.busy()) {
                        mhStage_.stop();
                    }
                    ProcessStage_ = Stage_Sx0;
                    stageStart();
                }

                Packet::free(pktACK_);
                pktPTR_ = NULL;
                break;
            case Tx_CON:
                mhTxState_ = Tx_Idle;
                TxActive_ = Inactive;

                CxActive_ = Active;

                Packet::free(pktCx_);
                pktCx_ = NULL;
                break;
            default:
                fprintf(stderr, "node[%d] - txHandler() ERROR mhTxState_\n", index_);
                exit(1);
                break;
        }
    }
}

void MacCogMOR::replyHandler(void) {
    switch (mhReplyState_) {
        case Reply_Beacon_B2_U:
            mhReplyState_ = Reply_Idle;
            transceiverMode_ = Transmitting;
            mhTxState_ = Tx_Beacon_B2_U;
            sendBeacon(Rx_address, Tx_address, Beacon_B2_U);
            transmit(pktBeacon_, CurrChannel_);
            ProrityActive_ = Inactive;
            fprintf(stderr, "node[%d] - transmit():Beacon_B2 -dst:%d -src:%d -chan:%d -time:%f \n", index_, Rx_address,
                    Tx_address, CurrChannel_, Scheduler::instance().clock());
            break;
        case Reply_Beacon_B2_B:
            mhReplyState_ = Reply_Idle;
            transceiverMode_ = Transmitting;
            mhTxState_ = Tx_Beacon_B2_B;
            sendBeacon(Rx_address, Tx_address, Beacon_B2_B);
            transmit(pktBeacon_, CurrChannel_);
            ProrityActive_ = Inactive;
            fprintf(stderr, "node[%d] - transmit():Beacon_B2 -dst:%d -src:%d -chan:%d -time:%f \n", index_, Rx_address,
                    Tx_address, CurrChannel_, Scheduler::instance().clock());
            break;
        case Reply_PTR:
            mhReplyState_ = Reply_Idle;
            transceiverMode_ = Transmitting;
            mhTxState_ = Tx_PTR;
            sendPTR(Rx_address, Tx_address, DataTransferChannel_, Reply_PTR_timeout);
            transmit(pktPTR_, CurrChannel_);

            fprintf(stderr, "node[%d] - transmit():PTR -dst:%d -src:%d -chan:%d -datachan:%d -time:%f \n", index_, Rx_address,
                    Tx_address, CurrChannel_,DataTransferChannel_, Scheduler::instance().clock());
            break;
        case Reply_ACK:
            mhReplyState_ = Reply_Idle;
            transceiverMode_ = Transmitting;
            mhTxState_ = Tx_ACK;
            sendACK(Rx_address, Tx_address);
            transmit(pktACK_, CurrChannel_);

            fprintf(stderr, "node[%d] - transmit():ACK -dst:%d -chan:%d -time:%f \n", index_, Rx_address, CurrChannel_,
                    Scheduler::instance().clock());
            break;
        case Reply_CON:
            mhReplyState_ = Reply_Idle;
            transceiverMode_ = Transmitting;
            mhTxState_ = Tx_CON;
            sendCON(Rx_address, Tx_address);
            transmit(pktCx_, CurrChannel_);

            fprintf(stderr, "node[%d] - transmit():CON -dst:%d -chan:%d -time:%f \n", index_, Rx_address, CurrChannel_,
                    Scheduler::instance().clock());
            break;
        default:
            fprintf(stderr, "replyHandler() ERROR ReplyWorkMode\n");
            exit(1);
    }
}

void MacCogMOR::deferHandler(void) {
    switch (mhDeferState_) {
        case Defer_PTS:
            mhDeferState_ = Defer_Idle;
            if (is_idle(CurrChannel_)) {
                transceiverMode_ = Transmitting;
                mhTxState_ = Tx_PTS;
                init_channel_list();
                remove_channel_list(CurrChannel_);
                DataTransferChannel_ = selectDC();
                sendPTS(Rx_address,Tx_address, DataTransferChannel_);
                transmit(pktPTS_, CurrChannel_);
                fprintf(stderr,"node[%d] - transmit():PTS -dst:%d -src:%d -chan:%d -datachan:%d -is_idle:%d -time:%f\n", index_, SendType_dst , SendType_src, CurrChannel_, DataTransferChannel_,is_idle(DataTransferChannel_),Scheduler::instance().clock());
            }
            break;
        case Defer_DATA_U:
            mhDeferState_ = Defer_Idle;
            if (is_idle(CurrChannel_)) {
                transceiverMode_ = Transmitting;
                mhTxState_ = Tx_DATA_U;
                fprintf(stderr,"node[%d] - transmit():DATA -dst:%d -src:%d -chan:%d -time:%f\n", index_, SendType_dst , SendType_src, CurrChannel_, Scheduler::instance().clock());
                transmit(pktTx_, CurrChannel_);

            }
            break;
        case Defer_DATA_B:
            mhDeferState_ = Defer_Idle;
            if (is_idle(CurrChannel_)) {
                transceiverMode_ = Transmitting;
                mhTxState_ = Tx_DATA_B;
                fprintf(stderr,"node[%d] - transmit():DATA -dst:%d -src:%d -chan:%d -time:%f\n",
                        index_, SendType_dst , SendType_src, CurrChannel_, Scheduler::instance().clock());
                transmit(pktTx_, CurrChannel_);
            }
            break;
        default:
            fprintf(stderr,"replyHandler() ERROR ReplyWorkMode\n");
            exit(1);
    }
}

void MacCogMOR::sendPu(int channel) {
    Packet *p = Packet::alloc();
    struct hdr_cmn *ch = HDR_CMN(p);
    struct hdr_mac_cogmor *dh = HDR_MAC_COGMOR(p);
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
    ch->channelindex_ = channel;
    ch->txtime() = txtime(ch->size(), basicRate_);

    dh->dh_fc.fc_protocol_version = MAC_CogMOR_ProtocolVersion;
    dh->dh_fc.fc_type = MAC_CogMOR_Type_Data;
    dh->dh_fc.fc_subtype = MAC_CogMOR_Subtype_Data;
    dh->dh_fc.fc_more_frag = 0;
    dh->dh_fc.fc_retry = 0;
    dh->dh_fc.fc_flag = 0;
    dh->dh_fc.fc_order = 0;
    dh->dh_fc.fc_reserved = 0;
    dh->dh_duration = 0;

    pktTx_ = p;
}

void MacCogMOR::puHandler(void) {
    if (nodeWorkMode_ == NodeWorkMode_Interference_Source) {
        sendPu(NoiseChannel_);
        transmitPu(pktTx_,NoiseChannel_);
    }
}

void MacCogMOR::transmitPu(Packet *p, int channel) {
    TxActive_ = Active;
    struct hdr_cmn *ch = HDR_CMN(p);
    ch->channelindex_ = channel;

    if (RxActive_ == Active) {
        ch = HDR_CMN(pktRx_);
        ch->collision() = 1;
    }

    if (EOTtarget_) {
        if (pktTx_ != NULL) {
            fprintf(stderr, "node[%d] - void MacCogMOR::transmit() ERROR eotPacket_ != NULL \n", index_);
            exit(1);
        }
        eotPacket_ = p->copy();
    }

    mhTx_.start(txtime(p));
    downtarget_->recv(p->copy(), this);
}

void MacCogMOR::transmit(Packet *p, int channel) {
    TxActive_ = Active;
    struct hdr_cmn *ch = HDR_CMN(p);
    ch->channelindex_ = channel;

    if (RxActive_ == Active) {
        ch = HDR_CMN(pktRx_);
        ch->collision() = 1;
    }

    if (EOTtarget_) {
        if (pktTx_ != NULL) {
            fprintf(stderr, "node[%d] - void MacCogMOR::transmit() ERROR eotPacket_ != NULL \n", index_);
            exit(1);
        }
        eotPacket_ = p->copy();
    }
    mhTx_.start(txtime(p));
    downtarget_->recv(p->copy(), this);
}

int MacCogMOR::is_idle(int channel) {
    if (mhCAVList_.findCAV(channel) == 1) {
        return 0;
    }

    if (RxActive_ == Active) {
        return 0;
    }
    if (TxActive_ == Active) {
        return 0;
    }

    return 1;
}

void MacCogMOR::uifsHandler() {
    std::list<ChannelInfo>::iterator it;
    if ((RxActive_ != Active)
        && (TxActive_ != Active)
        && (ProcessStage_ == Stage_Sx0)
        && (SendType_ == SendType_Unicast)) {

        init_channel_list();
        ReservationChannel_ = selectRC();
        if (is_idle(ReservationChannel_)) {
            ProrityActive_ = Active;
            Reservation_src = SendType_src;
            Reservation_dst = SendType_dst;

            if (mhStage_.busy()) {
                mhStage_.stop();
            }
            ProcessStage_ = Stage_B1_U;
            stageStart();

            mhTxState_ = Tx_Beacon_B1_U;
            sendBeacon(SendType_dst, SendType_src, Beacon_B1_U);
            transmit(pktBeacon_, CurrChannel_);
            fprintf(stderr, "node[%d] - transmit():Beacon_B1 -dst:%d -src:%d -chan:%d -time:%f\n",
                    index_, SendType_dst, SendType_src, CurrChannel_, Scheduler::instance().clock());
        } else {
            mhBackoff_.start(getUIFS());
        }
    }
}

void MacCogMOR::bifsHandler() {
    if ((RxActive_ != Active)
        && (TxActive_ != Active)
        && (ProcessStage_ == Stage_Sx0)
        && (SendType_ == SendType_Broadcast)) {

        init_channel_list();
        ReservationChannel_ = selectRC();
        if (is_idle(ReservationChannel_)) {
            ProrityActive_ = Active;
            Reservation_src = SendType_src;
            Reservation_dst = SendType_dst;

            if (mhStage_.busy()) {
                mhStage_.stop();
            }
            ProcessStage_ = Stage_B1_B;
            stageStart();

            mhTxState_ = Tx_Beacon_B1_B;
            sendBeacon(SendType_dst, SendType_src, Beacon_B1_B);
            transmit(pktBeacon_, CurrChannel_);
            fprintf(stderr,"node[%d] - transmit():Beacon_B1 -dst:%d -src:%d -chan:%d -time:%f\n",
                    index_, SendType_dst , SendType_src, CurrChannel_, Scheduler::instance().clock());
        } else {
            mhBackoff_.start(getUIFS());
        }
    }
}

void MacCogMOR::init_channel_list() {
    int i;
    reset_channel_list();
    std::list<ChannelInfo>::iterator it;
    for (i = 0; i < MaxChannelNum_; i++) {
        ChannelInfo chan(i, mhCAVList_.getExpire(i));
        macmib_.channel_list.push_back(chan);
    }
    macmib_.channel_list.sort();
  /*  fprintf(stderr,"node[%d] ===========================================\n", index_);
    for (it = macmib_.channel_list.begin(); it != macmib_.channel_list.end();it++) {
        fprintf(stderr,"channel:%d cav:%f\n",(*it).getIndex_(),(*it).getValue_());
    }
    fprintf(stderr,"node[%d] ===========================================\n", index_);*/
}

void MacCogMOR::reset_channel_list() {
    std::list<ChannelInfo>::iterator it;
    for (it = macmib_.channel_list.begin(); it != macmib_.channel_list.end();) {
        it = macmib_.channel_list.erase(it);
    }
}

void MacCogMOR::remove_channel_list(int channel) {
    std::list<ChannelInfo>::iterator it;
    for (it = macmib_.channel_list.begin(); it != macmib_.channel_list.end();) {
        if ((*it).getIndex_() == channel) {
            it = macmib_.channel_list.erase(it);
        } else {
            it++;
        }
    }
}

int MacCogMOR::selectRC() {
    if (macmib_.channel_list.size() == 0) {
        return -2;
    } else {
        return macmib_.channel_list.front().getIndex_();
    }
}

int MacCogMOR::selectDC() {
    if (macmib_.channel_list.size() == 0) {
        return -2;
    } else {
        return macmib_.channel_list.front().getIndex_();
    }
}

double MacCogMOR::txtime(Packet *p) {
    struct hdr_cmn *ch = HDR_CMN(p);
    double t = ch->txtime();
    if (t < 0.0) {
        drop(p, "XXX");
        fprintf(stderr,"txtime:Invalid txtime()\n");
        exit(1);
    }
    return t;
}

double MacCogMOR::txtime(double psz, double drt) {
    double dsz = psz - macmib_.getPLCPhdrLen();
    int plcp_hdr = macmib_.getPLCPhdrLen() << 3;
    int datalen = (int) dsz << 3;
    double t = (((double) plcp_hdr) / macmib_.getPLCPDataRate()) + (((double) datalen) / drt);
    return t;
}