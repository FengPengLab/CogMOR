//
// Created by fengpeng on 2/20/17.
//


#ifndef ns_mac_cogmor_h
#define ns_mac_cogmor_h

#include "mac.h"
#include "mac-cogmor-timer.h"

/* ======================================================================
   Frame Formats
   ====================================================================== */
#define MAC_CogMOR_ProtocolVersion  0x00

#define MAC_CogMOR_Type_Control     0x01
#define MAC_CogMOR_Type_Data        0x02
#define MAC_CogMOR_Type_Beacon      0x03

#define MAC_CogMOR_Subtype_PTS          0x00
#define MAC_CogMOR_Subtype_PTR          0x01
#define MAC_CogMOR_Subtype_Data         0x02
#define MAC_CogMOR_Subtype_ACK          0x03
#define MAC_CogMOR_Subtype_CON          0x04
#define MAC_CogMOR_Subtype_Beacon_B1_U    0x05
#define MAC_CogMOR_Subtype_Beacon_B2_U    0x06
#define MAC_CogMOR_Subtype_Beacon_B1_B    0x07
#define MAC_CogMOR_Subtype_Beacon_B2_B    0x08

#define NodeWorkMode_Cognitive_Node         0
#define NodeWorkMode_Interference_Source    2

#define MacCogMORPropagationDelay 0.000002        // 2us   XXXX

#define Beacon_B1_U   1
#define Beacon_B2_U   2
#define Beacon_B1_B   3
#define Beacon_B2_B   4

#define Type_Beacon  1
#define Type_Control 2

class EventTrace;
class MacCogMOR;

enum MacCogMORRx {
    Rx_Idle = 0,
    Rx_Recv = 1,
    Rx_Coll = 2
};
enum MacCogMORTx {
    Tx_Idle = 0,
    Tx_Beacon_B1_U = 1,
    Tx_Beacon_B1_B = 2,
    Tx_Beacon_B2_U = 3,
    Tx_Beacon_B2_B = 4,
    Tx_PTS = 5,
    Tx_PTR = 6,
    Tx_DATA_U = 7,
    Tx_DATA_B = 8,
    Tx_ACK = 9,
    Tx_CON = 11
};
enum MacCogMORDefer {
    Defer_Idle = 0,
    Defer_PTS = 1,
    Defer_DATA_U = 2,
    Defer_DATA_B = 3
};
enum MacCogMORReply {
    Reply_Idle = 0,
    Reply_Beacon_B2_U = 1,
    Reply_Beacon_B2_B = 2,
    Reply_PTR = 3,
    Reply_ACK = 4,
    Reply_CON = 6
};

enum MacCogMORStage {
    Stage_Sx0 = 0,
    Stage_Sx1 = 1,
    Stage_Sx2 = 3,
    Stage_Sx3 = 5,
    Stage_B1_U = 6,
    Stage_B1_B = 7,
    Stage_B2_U = 8,
    Stage_B2_B = 9,
    Stage_Tu = 10,
    Stage_Ru = 11,
    Stage_Tb = 13,
    Stage_Tx = 16,
    Stage_Rx = 17,
    Stage_Cx = 18
};
enum MacCogMORSendType {
    SendType_Unicast = 0,
    SendType_Broadcast = 1,
    SendType_Null = 2
};
enum TransceiverMode {
    Receiving = 0,
    Transmitting = 1,
    Sensing = 2
};
enum MacCogMORAcitive {
    Inactive = 0,
    Active = 1
};
struct frame_control_cogmor {
    u_char fc_protocol_version : 2;
    u_char fc_type : 2;
    u_char fc_subtype : 4;
    u_char fc_more_frag : 1;
    u_char fc_retry : 1;
    u_char fc_flag : 1;
    u_char fc_order : 1;
    u_char fc_reserved : 4;
};
struct hdr_mac_cogmor_control_short {
    struct frame_control_cogmor dh_fc;
    u_char dh_ra[ETHER_ADDR_LEN];
    u_char dh_ta[ETHER_ADDR_LEN];
    u_char rf_fcs[ETHER_FCS_LEN];
};
struct hdr_mac_cogmor_control_long {
    struct frame_control_cogmor dh_fc;
    u_int16_t dh_duration;
    u_int16_t dh_channel;
    u_char dh_ra[ETHER_ADDR_LEN];
    u_char dh_ta[ETHER_ADDR_LEN];
    u_char rf_fcs[ETHER_FCS_LEN];
};

struct hdr_mac_cogmor {
    struct frame_control_cogmor dh_fc;
    u_int16_t   dh_duration;
    u_char      dh_ra[ETHER_ADDR_LEN];
    u_char      dh_ta[ETHER_ADDR_LEN];
    u_int16_t   dh_sequence;
};

/* ======================================================================
   Definitions
   ====================================================================== */
class ChannelInfo {
private:
    int index_;
    double value_;
public:
    ChannelInfo(int chan, double va) {
        index_ = chan;
        value_ = va;
    }

    bool operator < (const ChannelInfo& ch) const {
        return (value_ < ch.value_);
    }

    int getIndex_() const {
        return index_;
    }

    void setIndex_(int index_) {
        ChannelInfo::index_ = index_;
    }

    double getValue_() const {
        return value_;
    }

    void setValue_(double value_) {
        ChannelInfo::value_ = value_;
    }

};

class MAC_COGMOR_MIB {

public:
    MAC_COGMOR_MIB(MacCogMOR *parent);
    int FailedCount;      // 失败次数
    std::list<ChannelInfo> channel_list;

private:
    double SlotTime;            // 时隙时间
    double SIFSTime;            // SIFS时间
    double DIFSTime;
    double EIFSTime;
    double NIFSTime;
    double PLCPDataRate;        // PLCP定义的数据传输速率
    int CWMin;            // 最小窗口大小
    int CWMax;            // 最大窗口大小
    int RetryLimit;       // 帧重传次数 1~7
    int RetryShortLimit;
    int PreambleLength;   // 随机接入前导码长度
    int PLCPHeaderLength; // PLCP头部长度
    int cw_;         // Contention Window
public:
    double getSlotTime() { return (SlotTime); }
    double getNIFS() { return (NIFSTime); }
    double getSIFS() { return (SIFSTime); }
    double getDIFS() { return (DIFSTime); }
    double getEIFS() { return (EIFSTime); }
    double getPLCPDataRate() { return (PLCPDataRate); }
    void rst_cw() {
        cw_ = getCWMin();
    }

    void inc_cw() {
        cw_ = (cw_ << 1) + 1;
        if (cw_ > getCWMax())
            cw_ = getCWMax();
    }

    int getCW() { return (cw_); }
    int getCWMin() { return (CWMin); }
    int getCWMax() { return (CWMax); }
    int getPreambleLength() { return (PreambleLength); }
    int getPLCPhdrLen() { return ((getPreambleLength() + PLCPHeaderLength) >> 3); }
    int getHdrLen() { return (getPLCPhdrLen() + sizeof(struct hdr_mac_cogmor)) + ETHER_FCS_LEN; }
    int getPTSlen() { return (getPLCPhdrLen() + sizeof(struct hdr_mac_cogmor_control_long)); }
    int getPTRlen() { return (getPLCPhdrLen() + sizeof(struct hdr_mac_cogmor_control_long)); }
    int getACKlen() { return (getPLCPhdrLen() + sizeof(struct hdr_mac_cogmor_control_short)); }
    int getCONlen() { return (getPLCPhdrLen() + sizeof(struct hdr_mac_cogmor_control_short)); }
    int getBEACONlen() { return (getPLCPhdrLen() + sizeof(struct hdr_mac_cogmor_control_short)); }
    int getRetryLimit() { return (RetryLimit); }
    int getRetryShortLimit() { return (RetryShortLimit); }
};

class CogMORHost {
public:
    LIST_ENTRY(CogMORHost) link;
    u_int32_t	index;
    u_int32_t	seqno;
};

/* ======================================================================
   The actual CogMOR MAC class.
   ====================================================================== */
class MacCogMOR : public Mac {
    friend class MacCogMORSenseTimer;
    friend class MacCogMORDeferTimer;
    friend class MacCogMORBackoffTimer;
    friend class MacCogMORCavTimer;
    friend class MacCogMORReplyTimer;
    friend class MacCogMORRxTimer;
    friend class MacCogMORTxTimer;
    friend class MacCogMORPuTimer;
    friend class MacCogMORStageTimer;
    friend class MacCogMORUIFSTimer;
    friend class MacCogMORBIFSTimer;
    friend class MacCogMORTimeoutTimer;
public:
    MacCogMOR();
    EventTrace *et_;

    void recv(Packet *p, Handler *h);
    void trace_event(char *, Packet *);
    int getMacAddress();
    int getProcessStage();
protected:
    void rxHandler(void);
    void txHandler(void);
    void backoffHandler(void);
    void replyHandler(void);
    void deferHandler(void);
    void puHandler(void);
    void stageHandler(void);
    void stageStart(void);
    void timeoutHandler(void);

    void uifsHandler();
    void bifsHandler();

    double getBIFS() {
        double f = (macmib_.getSIFS() + integer(macmib_.getCW() - 1) * macmib_.getSlotTime());
        return f;
    }

    double getUIFS() {
        double f = (macmib_.getSIFS() + integer(macmib_.getCW() - 1) * macmib_.getSlotTime());
        return f;
    }

    double getMaxBIFS() {
        double f = (macmib_.getSIFS() + (macmib_.getCWMax() - 1) * macmib_.getSlotTime());
        return f;
    }

    double getMaxUIFS() {
        double f = (macmib_.getSIFS() + (macmib_.getCWMax() - 1) * macmib_.getSlotTime());
        return f;
    }

    MAC_COGMOR_MIB macmib_;
    int is_idle(int channel);
private:
    int command(int argc, const char *const *argv);
    void transmit(Packet *p, int channel);
    void sendDATA(Packet *p);
    void sendPTS(int dst, int src, int channel);
    void sendPTR(int dst, int src, int channel, double timeout);
    void sendACK(int dst, int src);
    void sendCON(int dst, int src);
    void sendBeacon(int dst, int src, int type);
    void sendPu(int channel);
    void transmitPu(Packet *p, int channel);
    double txtime(Packet *p);
    double txtime(double psz, double drt);

    int initialized() { return (logtarget_ && Mac::initialized()); }
    void mac_log(Packet *p) { logtarget_->recv(p, (Handler *) 0); }

    double sec(double t) { return (t *= 1.0e-6); }
    u_int16_t usec(double t) { u_int16_t us = (u_int16_t) floor((t *= 1e6) + 0.5);return us; }

    int integer(int cw);

    void send(Packet *p, Handler *h);

    int selectRC();
    int selectDC();
    void init_channel_list();
    void reset_channel_list();
    void remove_channel_list(int channel);
private:
    MacCogMORTxTimer mhTx_;        // outgoing packets
    MacCogMORRxTimer mhRx_;        // incoming packets
    MacCogMORTimeoutTimer mhTimeout_;
    MacCogMORReplyTimer mhReply_;
    MacCogMORDeferTimer mhDefer_; //
    MacCogMORStageTimer mhStage_;
    MacCogMORBackoffTimer mhBackoff_;    // backoff timer
    MacCogMORCAVList mhCAVList_;
    MacCogMORPuTimer mhPu_;

    MacCogMORUIFSTimer mhUIFS_;
    MacCogMORBIFSTimer mhBIFS_;

    TransceiverMode transceiverMode_;

    MacCogMORAcitive RxActive_;
    MacCogMORAcitive TxActive_;
    MacCogMORAcitive DxActive_;
    MacCogMORAcitive CxActive_;
    MacCogMORAcitive ProrityActive_;

    MacCogMORRx mhRxState_;
    MacCogMORTx mhTxState_;
    MacCogMORReply mhReplyState_;
    MacCogMORDefer mhDeferState_;
    MacCogMORStage ProcessStage_;
    MacCogMORSendType SendType_;

    int SendType_src;
    int SendType_dst;

    int Tx_address;
    int Rx_address;

    int DataTransfer_src;
    int DataTransfer_dst;

    int Reservation_src;
    int Reservation_dst;

    int ReservationPeriodNum_;
    int MaxReservationPeriodNum_;

    int DataTransferPeriodNum_;
    int MaxDataTransferPeriodNum_;

    int ReservationChannel_;
    int DataTransferChannel_;
    int CurrChannel_;
    int NoiseChannel_;

    int MaxDate_Sizeof_;

    Packet *eotPacket_;     // copy for eot callback
    Packet *pktPTS_;        // outgoing PTS packet
    Packet *pktPTR_;
    Packet *pktACK_;
    Packet *pktCx_;
    Packet *pktTx_;
    Packet *pktRx_;
    Packet *pktBeacon_;
    Packet *pktData_;

    int MaxChannelNum_;

    int RetryCount_;          // Retry Count
    int RetryShortCount_;
    int nodeWorkMode_;
    
    double basicRate_;          // 广播发送速率
    double dataRate_;           // 数据发送速率
    double noiseStartTime_;

    double T_B2;
    double T_Tu;
    double T_Tb;
    double T_Tx;
    double T_Ru;
    double T_Rb;
    double T_Rx;
    double T_Sx1;
    double T_Sx2;
    double T_Sx3;
    double T_Cx;
    double T_broadcast_timeout;
    double T_unicast_timeout;
    double PU_interval;
    double Reply_PTR_timeout;
    int cache_node_count_;
    
    NsObject *logtarget_;
    NsObject *EOTtarget_;   // given a copy of packet at TX end

    u_int16_t sta_seqno_;	// next seqno that I'll use
    
    CogMORHost *cache_;

    int output;
};

#endif
