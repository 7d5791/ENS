//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#ifndef EnsRSU11p_H
#define EnsRSU11p_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/world/annotations/AnnotationManager.h"

#include "veins/modules/messages/VideoStreamWithTrace.h"
#include "veins/modules/messages/VideoStreamMessage_m.h"

#include "veins/modules/messages/WaveShortMessage_m.h"
#include <list>
#include "beacon.h"

////////////////////////////////////////////////////////////////////

#include "veins/modules/mac/ieee80211p/Mac1609_4.h"
#include "veins/modules/mac/ieee80211p/Mac80211pToPhy11pInterface.h"
#include "veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h"


#include "veins/modules/phy/Decider80211p.h"
#include "veins/modules/phy/DeciderResult80211.h"
#include "veins/modules/phy/Decider80211pToPhy80211pInterface.h"
#include "veins/modules/phy/PhyLayer80211p.h"
#include "veins/modules/phy/NistErrorRate.h"
#include "veins/base/phyLayer/ChannelState.h"
#include "veins/modules/phy/SNRThresholdDecider.h"
#include "veins/base/utils/MacToNetwControlInfo.h"
#include "veins/base/phyLayer/BasePhyLayer.h"

#include "veins/base/phyLayer/PhyToMacControlInfo.h"
#include "veins/modules/phy/DeciderResult80211.h"
#include "veins/modules/utility/ConstsPhy.h"
////////////////////////////////////////////////////////////////////


using Veins::AnnotationManager;

// action mode macros
#define SERVER_MODE     1
#define CLIENT_MODE     2

using namespace std;

typedef std::vector<WaveShortMessage*> WaveShortMessages;
typedef std::vector<VideoStreamMessage*> VideoStreamMessages;



enum MessageKind {
    FRAME_START = 100, PACKET_TX = 200
};



/**
 * Small RSU Demo using 11p
 */
class EnsRSU11p : public BaseWaveApplLayer {
	public:
    struct VideoStreamData {
        // packet generation
        uint16_t currentSequenceNumber; ///< current (16-bit RTP) sequence number

        // variable for a video trace
        TraceFormat traceFormat;    ///< file format of trace file
        long numFrames;         ///< total number of frames for a video trace
        long numFramesSent; ///< counter for the number of frames sent; reset to zero once the whole frames in the trace file have been sent
        double framePeriod;     ///< frame period for a video trace
        long currentFrame; ///< frame index (starting from 0) to read from the trace (will be wrapped around)
        long frameNumber; ///< frame number (display order) of the current frame
        double frameTime; ///< cumulative display time of the current frame in millisecond
        FrameType frameType;    ///< type of the current frame
        long frameSize;         ///< size of the current frame in byte
        long bytesLeft;         ///< bytes left to transmit in the current frame
        double pktInterval; ///< interval between consecutive packet transmissions in a given frame
        int destinationId;

        // statistics
        long numPktSent;           ///< number of packets sent

        // self messages for timers
        cMessage *frameStartMsg;  ///< start of each frame
        cMessage *packetTxMsg;    ///< start of each packet transmission
    };

    struct statusNode {double distaneToRsu; std::string status;};

    typedef std::vector<VideoStreamData *> VideoStreamVector;

    virtual int numInitStages() const {
        return 4;
    }

	virtual void initialize(int stage);
	virtual void handleMessage(cMessage* msg);
	virtual void handleLowerMsg(cMessage* msg);

	protected:
	    /*
	     * These are variables for the client mode
	     */
	    // module parameters
	    double startupDelay;
	    long numTraceFrames; ///< number of frames in the trace file (needed to handle wrap around of frames by the server)
	    int gopSize;    ///< GOP pattern: I-to-I frame distance (N)
	    int numBFrames; ///< GOP pattern: I-to-P frame distance (M)

	    // statistics
	    bool warmupFinished;    ///< if true, start statistics gathering
	    long numPacketsReceived;    ///< number of packets received
	    long numPacketsLost;    ///< number of packets lost
	    long numFramesReceived; ///< number of frames received and correctly decoded
	    long numFramesDiscarded; ///< number of frames discarded due to packet loss or failed dependency

	    long numSentPacket;
	    long numSentFrame;
	    long ReceivedPacketCount;

	    // variables for packet and frame loss handling
	    uint16_t prevSequenceNumber; ///< (16-bit RTP) sequence number of the most recently received packet
	    long prevIFrameNumber; ///< frame number of the most recently received I frame
	    long prevPFrameNumber; ///< frame number of the most recently received P frame
	    long currentFrameNumber; ///< frame number (display order) of the frame under processing
	    long currentEncodingNumber; ///< encoding number (transmission order) of the frame under processing
	    FrameType currentFrameType; /// type (I, IDR, P, or B) of the frame under processing
	    bool currentFrameDiscard; ///< if true, all the remaining packets of the current frame are to be discarded
	    bool currentFrameFinished; ///< if true, the frame has been successfully received and decoded

	    VideoStreamVector streamVector;

	    int m_count;
	    int minCount;
	    double minDistance;
	    double maxTime;
	    string traceFile;

	    int m_id;
	    string mode;
	    double m_dist;
	    char network[127];

	    unsigned int numStreams;
	    list<int> m_pktNameSet;
	    bool startFlag;
	    simtime_t curTime;

	    // variables for a trace file
	    TraceFormat traceFormat;    ///< file format of trace file
	    long numFrames; ///< total number of frames in a trace file
	    double framePeriod;

	    int appOverhead;
	    int maxPayloadSize;
	    bool frameSpreading;
	    int actionMode;

	    //Last Frame received
	    int lastFrameReceived;

	    Coord sourcePos;
	    /*
	     * These are vector structures for the frame information
	     */
	    LongVector frameNumberVector; ///< vector of frame numbers (display order) (only for verbose trace)
	    DoubleVector frameTimeVector; ///< vector of cumulative frame display times (only for verbose trace)
	    FrameTypeVector frameTypeVector; ///< vector of frame types (I, P, B, and IDR (H.264); only for verbose trace)
	    LongVector frameSizeVector;


	    /* Beacon parameters*/
	    int beaconLengthBits;
	    int beaconPriority;
	    int beaconInterval;
	    bool sendBeacons;
	    simtime_t individualOffset;
	    cMessage* sendBeaconEvt;
	    double neighborValidityInterval;

	    /*Video Server*/
	    int idVideoRequest;
	    int idVehicleRequest;


	    ///*Neighbors Table*/
	    BeaconList ListBeacon;
	    map<int, double> maxDistanceRsu;
	    int counterBeaconsReceived;

	    map<int, statusNode> statusNodes;

	    WaveShortMessages neighbors;

	    VideoStreamMessages dataVideo;


	public:


	// process stream request from client
	virtual void processStreamRequest(cMessage *msg, int lfr, int idD);

	// read new frame data and update relevant variables
	virtual void readFrameData(cMessage *frameTimer);

	 // send a packet of the given video stream
	 virtual void sendStreamData(cMessage *pktTimer);

	 enum WaveApplMessageKinds {
	                SEND_BEACON_EVT=1000,
	                SEND_VIDEO_EVT=2000
	  };



	protected:
		AnnotationManager* annotations;
		BaseMobility* mobi;
		bool sentMessage;
	protected:
		virtual void onBeacon(WaveShortMessage* wsm);
		virtual void onData(WaveShortMessage* wsm);
		void sendMessage(std::string blockedRoadId);
		virtual void sendWSM(WaveShortMessage* wsm);

	protected:
	    void setupLowerLayer();
	    virtual void handleSelfMsg(cMessage* apMsg);
	    virtual void sendMessage(VideoStreamMessage* sendMsg);
	    virtual void handlePositionUpdate();


	    void sendVideoData();
	    void loadVideoData(VideoStreamMessage* Vmsg);

	    void LogToFile(VideoStreamMessage* msg, bool flag);
	    void recvStream(VideoStreamMessage *pkt);
	    long frameEncodingNumber(long frameNumber, int numBFrames,
	            FrameType frameType);
	    struct ReceivedMessage {
	        long number;
	        simtime_t time;
	        long size;
	        int type;
	        bool endFlag;

	        ReceivedMessage(long m_number, simtime_t m_time, int m_type,
	                long m_size, bool m_endFlag) {
	            number = m_number;
	            time = m_time;
	            size = m_size;
	            type = m_type;
	            endFlag = m_endFlag;
	        }
	    };

	    list<ReceivedMessage> m_recvMessageList;


	    int frameCount;
	    int packetCount;

	    double accidentDuration;
	    double accidentStart;
	    int frameLost;

	    int m_logIndex;
	    int m_frameIndex;
	    int m_lastFrameCount;
	    int prevFrameNumber;

};

#endif
