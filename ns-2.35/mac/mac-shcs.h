//
// Created by fengpeng on 6/24/17.
//

#ifndef NS_2_35_MAC_SHCS_H
#define NS_2_35_MAC_SHCS_H
#include "mac.h"
#include "mac-shcs-timer.h"
/* ======================================================================
   Frame Formats
   ====================================================================== */


#define MAC_SHCS_Type_Beacon 0x00
#define MAC_SHCS_Type_Control 0x01
#define MAC_SHCS_Type_Data 0x02
#define MAC_SHCS_Type_Signal 0x03

#define MAC_SHCS_Subtype_Reserved 0x00
#define MAC_SHCS_Subtype_RTS 0x01
#define MAC_SHCS_Subtype_CTS 0x02
#define MAC_SHCS_Subtype_DATA 0x03
#define MAC_SHCS_Subtype_ACK 0x04
#define MAC_SHCS_Subtype_Active_Beacon 0x05
#define MAC_SHCS_Subtype_Busy_Signal 0x06

#define MAC_SHCS_Operation_Reserved 0x00
#define MAC_SHCS_Operation_Basic 0x01
#define MAC_SHCS_Operation_Extended 0x02

#define MAC_SHCS_ProtocolVersion 0x00

#define MacSHCS_Default_Channel 0

#define MacSHCSPropagationDelay 0.000002        // 2us   XXXX

class EventTrace;
class MacSHCS;
enum MacSHCSState {
    MacSHCS_Idle = 0,
    MacSHCS_Recv = 1,
    MacSHCS_Coll = 2,
    MacSHCS_Send_Beacon = 3,
    MacSHCS_Send_Signal = 4,
    MacSHCS_Send_RTS = 5,
    MacSHCS_Send_CTS = 6,
    MacSHCS_Send_DATA = 7,
    MacSHCS_Send_ACK = 8,
};
enum MacSHCSSendType {
    MacSHCS_Null = 0,
    MacSHCS_Unicast = 1,
    MacSHCS_Broadcast = 2
};
enum MacSHCSPhaseState {
    MacSHCS_Init = 0,
    MacSHCS_Sensing = 1,
    MacSHCS_Beacon = 2,
    MacSHCS_Reporting = 3,
    MacSHCS_Transfer = 4,
    MacSHCS_Switching = 5
};
enum MacSHCSTransfer {
    MacSHCS_Silence_Active = 0,
    MacSHCS_Basic_Active = 1,
    MacSHCS_Extended_Active = 2
};
struct frame_control_shcs {
    u_char fc_subtype		    : 4;
    u_char fc_type			    : 2;
    u_char fc_operation         : 2;
    u_char fc_protocol_version	: 2;
    u_char fc_order		        : 1;
    u_char fc_wep			    : 1;
    u_char fc_more_data		    : 1;
    u_char fc_pwr_mgt		    : 1;
    u_char fc_retry		        : 1;
    u_char fc_more_frag		    : 1;
    u_char fc_from_ds           : 1;
    u_char fc_to_ds		        : 1;
};
struct hdr_mac_shcs {
    struct frame_control_shcs dh_fc;
    u_int16_t   dh_duration;
    u_char      dh_ra[ETHER_ADDR_LEN];
    u_char      dh_ta[ETHER_ADDR_LEN];
    u_char      dh_3a[ETHER_ADDR_LEN];
    u_char      dh_4a[ETHER_ADDR_LEN];
    u_char		dh_scontrol;
    u_char      dh_body[1]; // size of 1 for ANSI compatibility
};
struct hdr_mac_shcs_beacon {
    struct frame_control_shcs dh_fc;
    int suc_id;
};
/*
 * 该数据包用于模拟busy signal
 * 接收该数据包时忽略接收时间，默认该信号不会产生碰撞
 */
struct hdr_mac_shcs_signal {
    struct frame_control_shcs dh_fc;
};
/* ======================================================================
   Definitions
   ====================================================================== */

class MAC_SHCS_MIB {

public:
    MAC_SHCS_MIB(MacSHCS *parent);
    int FailedCount;      // 失败次数
private:

    double SlotTime;            // 时隙时间
    double SIFSTime;            // SIFS时间
    double PLCPDataRate;        // PLCP定义的数据传输速率
    int CWMin;            // 最小窗口大小
    int CWMax;            // 最大窗口大小
    int RetryLimit;       // 帧重传次数 1~7
    int PreambleLength;   // 随机接入前导码长度
    int PLCPHeaderLength; // PLCP头部长度
public:
    double getSlotTime() { return (SlotTime); }
    double getSIFS() { return (SIFSTime); }
    double getDIFS() { return (SIFSTime + 2 * SlotTime); }
    double getEIFS() { return(SIFSTime + getDIFS() + (8 *  getACKlen())/PLCPDataRate); }
    double getPLCPDataRate() { return (PLCPDataRate); }
    int getCWMin() { return (CWMin); }
    int getCWMax() { return (CWMax); }
    int getPreambleLength() { return (PreambleLength); }
    int getPLCPhdrLen() { return ((PreambleLength + PLCPHeaderLength) >> 3); }
    int getHdrLen() { return (getPLCPhdrLen() + sizeof(struct hdr_mac_shcs)) + ETHER_FCS_LEN; }
    int getBeaconlen() { return (getPLCPhdrLen() + sizeof(struct hdr_mac_shcs_beacon)); }
    int getSignallen() { return (getPLCPhdrLen() + sizeof(struct hdr_mac_shcs_signal)); }
    int getRTSlen() { return (getPLCPhdrLen() + sizeof(struct hdr_mac_shcs)); }
    int getCTSlen() { return (getPLCPhdrLen() + sizeof(struct hdr_mac_shcs)); }
    int getACKlen() { return (getPLCPhdrLen() + sizeof(struct hdr_mac_shcs)); }
    int getRetryLimit() { return (RetryLimit); }
};

class SHCSHost {
public:
    LIST_ENTRY(SHCSHost) link;
    u_int32_t	index;
    u_int32_t	seqno;
};

/* ======================================================================
   The actual SHCS MAC class.
   ====================================================================== */
class MacSHCS : public Mac {
    friend class MacSHCSCCATimer;
    friend class MacSHCSDeferTimer;
    friend class MacSHCSBackoffTimer;
    friend class MacSHCSIFTimer;
    friend class MacSHCSCavTimer;
    friend class MacSHCSReplyTimer;
    friend class MacSHCSRxTimer;
    friend class MacSHCSTxTimer;
    friend class MacSHCSNodeTimer;
    friend class MacSHCSPhaseTimer;
    friend class MacSHCSPuTimer;
public:
    MacSHCS();
    EventTrace *et_;

    void recv(Packet *p, Handler *h);
    void trace_event(char *, Packet *);


    MacSHCSSendType getSendType() { return SendType_; }
    MacSHCSState getReplyState() { return ReplyState_; }
    MacSHCSState getDeferState() { return DeferState_; }
    MacSHCSState getSendState() { return SendState_; }
    MacSHCSState getBackoffState() { return BackoffState_; }
    MacSHCSState getCCAState() { return CCAState_; }
    MacSHCSState getRxState() { return RxState_; }
    MacSHCSState getTxState() { return TxState_; }
    MacSHCSPhaseState getPhaseState() { return PhaseState_; }

    void checkBackoffTimer();

    void setSendType(MacSHCSSendType s) { SendType_ = s; }
    void setBackoffState(MacSHCSState k) { BackoffState_ = k; }
    void setSendState(MacSHCSState k) { SendState_ = k; }
    void setReplyState(MacSHCSState k) { ReplyState_ = k; }
    void setDeferState(MacSHCSState k) { DeferState_ = k; }
    void setCCAState(MacSHCSState k) { CCAState_ = k; }
    void setRxState(MacSHCSState s) { RxState_ = s; }
    void setTxState(MacSHCSState s) { TxState_ = s; }

    void setPhaseState(MacSHCSPhaseState p);

    void setTxActive(int i);
    void setRxActive(int i);
    void setSwitchActive(int k);
    void setExtendedActive(int p);


    int getTxActive() { return TxActive_; }
    int getRxActive() { return RxActive_; }
    int getSwitchActive() { return SwitchActive_; }
    int getCurrChannel() { return CurrChannel_; }
    int getMaxChannel() { return MaxChannelNum_;}

protected:

    void recvHandler(void);
    void txHandler(double timeout);
    void backoffHandler(void);
    void deferHandler(void);
    void sendHandler(void);
    void replyHandler(void);
    void phaseHandler(void);
    void puHandler(void);
    MAC_SHCS_MIB macmib_;
    int is_idle();
private:
    int command(int argc, const char *const *argv);

    void transmit(Packet *p, double timeout);

    void send(Packet *p, Handler *h);

    void sendDATA(Packet *p);
    void sendRTS(int dst, int src);
    void sendCTS(int dst, int src, double time);
    void sendACK(int dst, int src);
    void sendBeacon();
    void sendSignal();
    void sendPu();
    void recvRTS(Packet *p);
    void recvCTS(Packet *p);
    void recvDATA(Packet *p);
    void recvACK(Packet *p);
    void recvBeacon(Packet *p);
    void recvSignal(Packet *p);

    void switchChannel();

    double txtime(Packet *p);
    double txtime(double psz, double drt);

    int initialized() {
        return (logtarget_ && Mac::initialized());
    }
    void mac_log(Packet *p) {
        logtarget_->recv(p, (Handler *) 0);
    }
    void rst_cw() {
        cw_ = macmib_.getCWMin();
    }
    void inc_cw() {
        cw_ = (cw_ << 1) + 1;
        if (cw_ > macmib_.getCWMax())
            cw_ = macmib_.getCWMax();
    }
    double sec(double t) {
        return (t *= 1.0e-6);
    }
    u_int16_t usec(double t) {
        u_int16_t us = (u_int16_t) floor((t *= 1e6) + 0.5);
        return us;
    }


private:

    MacSHCSIFTimer mhIF_;
    MacSHCSTxTimer mhSend_;
    MacSHCSRxTimer mhRecv_;
    MacSHCSDeferTimer mhDefer_;
    MacSHCSReplyTimer mhReply_;
    MacSHCSBackoffTimer mhBackoff_;
    MacSHCSPhaseTimer mhPhase_;
    MacSHCSPuTimer mhPu_;
    MacSHCSCAVList mhCAVList_;

    MacSHCSSendType SendType_;
    MacSHCSPhaseState PhaseState_;
    MacSHCSTransfer TransferType_;

    MacSHCSState CCAState_;
    MacSHCSState DeferState_;
    MacSHCSState BackoffState_;
    MacSHCSState ReplyState_;
    MacSHCSState SendState_;
    MacSHCSState RxState_;     // incoming state
    MacSHCSState TxState_;     // outgoint state

    Packet *eotPacket_;     // copy for eot callback
    Packet *pktRTS_;
    Packet *pktCTS_;
    Packet *pktACK_;
    Packet *pktTx_;
    Packet *pktRx_;
    Packet *pktBeacon_;
    Packet *pktSignal_;

    int findSUC_;
    int interface_;                 // 接口index
    int InterfaceNum_;
    int CurrChannel_;              // 接收信道
    int hoppingChannel_;
    int MaxChannelNum_;
    int noiseThreshold_;
    int noiseEnabled_;    // 干扰模拟器标识
    int noiseChannel_;
    int nodeWorkMode_;
    int TxActive_;         // transmitter is ACTIVE
    int RxActive_;
    int cw_;          // Contention Window
    int RetryCount_;          // Retry Count
    int transmitDst_;
    int NodeID;
    double basicRate_;          // 广播发送速率
    double dataRate_;           // 数据发送速率
    double noiseStartTime_;
    double T_switch_channel;
    double T_sensing;
    double T_cca;
    double T_beacon;
    double T_reporting;
    double T_transfer;
    double T_extended_transfer;
    double T_hop_duration;
    double T_frame_duration;
    double T_init;
    double PU_interval;
    int SUC_ID_;
    int SucActive_;
    int NoiseActive_;
    int BeaconActive_;
    int SwitchActive_;
    int ExtendedActive_;
    int lostBeacon_;
    NsObject *logtarget_;
    NsObject *EOTtarget_;   // given a copy of packet at TX end

    u_int16_t sta_seqno_;	// next seqno that I'll use
    int cache_node_count_;
    SHCSHost *cache_;
};
#endif //NS_2_35_MAC_SHCS_H
