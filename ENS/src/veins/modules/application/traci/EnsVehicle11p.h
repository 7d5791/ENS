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

#ifndef TraCIDemo11p_H
#define TraCIDemo11p_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
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
#include <algorithm>
#include <omnetpp.h>
#include "beacon.h"
#include "rsuData.h"

#include "veins/modules/messages/VideoStreamWithTrace.h"
#include "veins/modules/messages/VideoStreamMessage_m.h"

#include <list>

// action mode macros
#define SERVER_MODE     1
#define CLIENT_MODE     2

using namespace std;

using Veins::TraCIMobility;
using Veins::TraCICommandInterface;
using Veins::AnnotationManager;

typedef std::vector<WaveShortMessage*> WaveShortMessages;
/**
 * Small IVC Demo using 11p
 */
class EnsVehicle11p : public BaseWaveApplLayer {
	public:
		virtual void initialize(int stage);
		virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details);
		virtual void handleMessage(cMessage* msg);

		enum WaveApplMessageKinds {
		                SEND_BEACON_EVT=1000,
		                SEND_VIDEO_EVT=2000
		            };
	    struct statusNode {double distaneToRsu; std::string status;};

	    EnsVehicle11p();
		virtual ~EnsVehicle11p();

	protected:
	    virtual int numInitStages() const {
	        return 4;
	    }

		TraCIMobility* mobility;
		TraCICommandInterface* traci;
		TraCICommandInterface::Vehicle* traciVehicle;
		AnnotationManager* annotations;
		simtime_t lastDroveAt;
		bool sentMessage;
		bool isParking;
		bool sendWhileParking;
		static const simsignalwrap_t parkingStateChangedSignal;
	protected:
		virtual void onBeacon(WaveShortMessage* wsm);
		virtual void onData(WaveShortMessage* wsm);
		//void sendMessage(std::string blockedRoadId);
		void sendMessage(WaveShortMessage* sendMsg);
		virtual void handlePositionUpdate(cObject* obj);
		virtual void handleParkingUpdate(cObject* obj);
		virtual void sendWSM(WaveShortMessage* wsm);
		virtual void handleSelfMsg(cMessage* msg);
		virtual void finish() override;

		virtual void handleLowerMsg(cMessage* apMsg);

	    void requestServiceToRSU();
	    void statisticsRSUIn();
	    void statisticsRSUOut();

	    /*Methods to simulate a Queue*/
        void queueBuffer(cMessage *msg);
	    void sinkBuffer(cMessage *msg);
	    void sourceBuffer(VideoStreamMessage *job);



	    /*Method to write results by vehicle*/
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

	    int frameCount;
	    int packetCount;

	    //Positions RSU
	    Coord rsu1;
	    Coord rsu2;
	    Coord rsu3;


	    /* Beacon parameters*/
	     int beaconLengthBits;
	     int beaconPriority;
	     int beaconInterval;
	     bool sendBeacons;
	     simtime_t individualOffset;
	     cMessage* sendBeaconEvt;
	     double neighborValidityInterval;


	     ///*Neighbors Table*/
	     BeaconList ListBeacon;
	     map<int, double> maxDistanceRsu;
	     int counterBeaconsReceived;
	     map<int, statusNode> statusNodes;
	     WaveShortMessages neighbors;

	     //*Service from RSU*/
	     bool inCoverage;
	     cMessage *endRsuCoverageMsg;
	     int idRsuService;

	     //*Statistics*/
         double vehicleCurrentSpeed;
         double myBitrate;

         std::list<simtime_t> timeInOutRsu;
         std::list<double> distanceTravelled;
         map<simtime_t, rsuData> statisticsRsuIn;
         map<simtime_t, rsuData> statisticsRsuOut;

         double distanceNextRsu;





	     // Queue//
	 protected:
	  virtual simtime_t startService(cMessage *msg);
	  virtual void endService(cMessage *msg);
	   virtual void arrival(cMessage *msg) {}


	 protected:
	   cMessage *msgServiced;
	   cMessage *endServiceMsg;

	   cQueue queue;
	   int capacity;
	   bool fifo;

	   simsignal_t qlenSignal;
	   simsignal_t busySignal;
	   simsignal_t queueingTimeSignal;

	 public:
	   double totalDownload,totalDownloadMB;
	   double totalQueue,totalQueueMB;
	   double totalPlayed,totalPlayedMB;

	 private:
	   simsignal_t lifetimeSignal;


};

#endif
