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

#include "veins/modules/application/traci/EnsVehicle11p.h"

using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

const simsignalwrap_t EnsVehicle11p::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

Define_Module(EnsVehicle11p);

void EnsVehicle11p::initialize(int stage) {
	BaseWaveApplLayer::initialize(stage);
	if (stage == 0) {
		mobility = TraCIMobilityAccess().get(getParentModule());
		traci = mobility->getCommandInterface();
		traciVehicle = mobility->getVehicleCommandInterface();
		annotations = AnnotationManagerAccess().getIfExists();
		ASSERT(annotations);

		sentMessage = false;
		lastDroveAt = simTime();
		findHost()->subscribe(parkingStateChangedSignal, this);
		isParking = false;
		sendWhileParking = par("sendWhileParking").boolValue();

		counterBeaconsReceived=0;

		statusNodes[0] = {-1,"off"};

	} else if (stage == 3) {
        //setupLowerLayer();
        neighborValidityInterval = par("neighborValidityInterval").doubleValue(); ///< unit is second

        startupDelay = par("startupDelay").doubleValue(); ///< unit is second
        numTraceFrames = par("numTraceFrames").longValue();
        gopSize = par("gopSize").longValue();
        numBFrames = par("numBFrames").longValue();

        //Locate Position of RSUs
        rsu1.x = par("rsu1X").doubleValue(); ///< unit is second
        rsu1.y = par("rsu1Y").doubleValue();
        rsu2.x = par("rsu2X").doubleValue();
        rsu2.y = par("rsu2Y").doubleValue();
        rsu3.x = par("rsu3X").doubleValue();
        rsu3.y = par("rsu3Y").doubleValue();



        // initialize statistics
        numPacketsReceived = 0;
        numPacketsLost = 0;
        numFramesReceived = 0;
        numFramesDiscarded = 0;
        frameCount = 0;
        packetCount = 0;

        //traci control vehicle
        //traci = TraCIMobilityAccess().get();
        //traci->subscribe(mobilityStateChangedSignal, this);


        //initialize statistics queue
        endServiceMsg = new cMessage("end-service",END_QUEUE_EVT);
        queue.setName("queue");

        qlenSignal = registerSignal("qlen");
        busySignal = registerSignal("busy");
        queueingTimeSignal = registerSignal("queueingTime");
        emit(qlenSignal, queue.length());
        emit(busySignal, false);

        //initialize statistics sink
        lifetimeSignal = registerSignal("lifetime");

        totalDownload=totalDownloadMB=0;
        totalQueue= totalQueueMB=0;
        totalPlayed=totalPlayedMB=0;

        vehicleCurrentSpeed=0;

        WATCH(totalPlayedMB);
        WATCH(totalQueueMB);
        WATCH(totalDownloadMB);

        WATCH(vehicleCurrentSpeed);
        WATCH(inCoverage);

        //initialize detect out of service rsu
        endRsuCoverageMsg = new cMessage("end-service-rsu", END_RSUSERVICE_EVT);
        inCoverage=false;

        timeInOutRsu.push_back(simTime());
        distanceTravelled.push_back(mobility->getCurrentPosition().x);

        distanceNextRsu=0;
    }
}


EnsVehicle11p::EnsVehicle11p()
{
    msgServiced = endServiceMsg = endRsuCoverageMsg = NULL;
}

EnsVehicle11p::~EnsVehicle11p()
{

}



/*Methods to simulate a Queue*/

void EnsVehicle11p:: queueBuffer(cMessage *msg)
{

if (msg==endServiceMsg)
{
    endService( msgServiced );
    if (queue.empty())
    {
        msgServiced = NULL;
        emit(busySignal, false);
    }
    else
    {
        msgServiced = (cMessage *) queue.pop();
        VideoStreamMessage *msgServiced1 = dynamic_cast<VideoStreamMessage*>(msgServiced);

        // EV <<"Paquete msgServiced1->getByteLength() sacado de la cola: "<<msgServiced1->getByteLength()<<endl;
        // EV <<"Paquete msgServiced1->getByteLength() sacado de la cola: "<<msgServiced1->getFrameType()<<endl;
        // EV <<"Paquete msgServiced1->getByteLength() sacado de la cola: "<<msgServiced1->getFrameNumber()<<endl;

         totalPlayed=totalPlayed+msgServiced1->getByteLength();
         totalPlayedMB=(totalPlayed/1024)/1024;

         totalQueue=totalDownload-totalPlayed;
         totalQueueMB=(totalQueue/1024)/1024;

        emit(qlenSignal, queue.length());
        emit(queueingTimeSignal, simTime() - msgServiced->getTimestamp());
        simtime_t serviceTime = startService( msgServiced );
        scheduleAt( simTime()+serviceTime, endServiceMsg );
    }
}
else if (!msgServiced)
{
    arrival( msg );
    msgServiced = msg;
    emit(queueingTimeSignal, SIMTIME_ZERO);
    simtime_t serviceTime = startService( msgServiced );
    scheduleAt( simTime()+serviceTime, endServiceMsg );
    emit(busySignal, true);
}
else
{
    arrival( msg );
    queue.insert( msg );
    VideoStreamMessage *vpack = dynamic_cast<VideoStreamMessage*>(msg);
    totalDownload=totalDownload+vpack->getByteLength();
    totalDownloadMB=(totalDownload/1024)/1024;

    msg->setTimestamp();
    emit(qlenSignal, queue.length());
}
}



void EnsVehicle11p::sourceBuffer(VideoStreamMessage *job){

    queueBuffer(job);

}

void EnsVehicle11p::sinkBuffer(cMessage *msg)
{
    simtime_t lifetime = simTime() - msg->getCreationTime();
    EV << "Received " << msg->getName() << ", lifetime: " << lifetime << "s" << endl;
    emit(lifetimeSignal, lifetime);
    delete msg;

}


simtime_t EnsVehicle11p::startService(cMessage *msg)
{
    EV << "Starting service of " << msg->getName() << endl;
    return par("serviceTime");
}

void EnsVehicle11p::endService(cMessage *msg)
{
    EV << "Completed service of " << msg->getName() << endl;
    sinkBuffer( msg);
}


void EnsVehicle11p::onData(WaveShortMessage* wsm) {
	findHost()->getDisplayString().updateWith("r=16,green");
	annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobility->getPositionAt(simTime()), "blue"));

}



/*
 * this is the function for send the video request message
 */
void EnsVehicle11p::sendMessage(WaveShortMessage* sendMsg) {
    sentMessage = true;
    sendWSM(sendMsg);
   // socket.sendTo(sendMsg, IPv4Address::ALL_HOSTS_MCAST, 4000);
}

/*
 * receive the packet handler
 */
void EnsVehicle11p::handleMessage(cMessage* msg) {
    if (msg->isSelfMessage()) {
        handleSelfMsg(msg);
    } else {
        handleLowerMsg(msg);
    }
}




void EnsVehicle11p::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
	Enter_Method_Silent();
	if (signalID == mobilityStateChangedSignal) {
		handlePositionUpdate(obj);
	}

}
void EnsVehicle11p::handleParkingUpdate(cObject* obj) {
	isParking = mobility->getParkingState();
	if (sendWhileParking == false) {
		if (isParking == true) {
			(FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(this->getParentModule()->getSubmodule("nic"));
		}
		else {
			Coord pos = mobility->getCurrentPosition();
			(FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(this->getParentModule()->getSubmodule("nic"), (ChannelAccess*) this->getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);
		}
	}
}
void EnsVehicle11p::handlePositionUpdate(cObject* obj) {
	BaseWaveApplLayer::handlePositionUpdate(obj);

	   EV << ">> Neighbors Table <<"<<std::endl;
	    ListBeacon.PurgeBeacons(neighborValidityInterval);
	    ListBeacon.PrintBeacons();


	    requestServiceToRSU();

	    EV<<"Inside RSU ?"<<inCoverage<<endl;
	    EV<<"Speed:: "<<mobility->getSpeed()<<"m/s"<<endl;
	    EV<<"Number of received frames: "<<frameCount<<" frames"<<endl;

	    vehicleCurrentSpeed=mobility->getSpeed();


}
void EnsVehicle11p::sendWSM(WaveShortMessage* wsm) {
	if (isParking && !sendWhileParking) return;
	sendDelayedDown(wsm,individualOffset);
}


void EnsVehicle11p::handleSelfMsg(cMessage* msg) {
    switch (msg->getKind()) {

            case  END_QUEUE_EVT: {
                queueBuffer(msg);
                break;
            }

            case  END_RSUSERVICE_EVT: {
                           inCoverage=false;
                           statisticsRSUOut();
                           break;

                      }
            default: {
                    BaseWaveApplLayer::handleSelfMsg(msg);
                   }
        }
}



void EnsVehicle11p::onBeacon(WaveShortMessage* wsm) {

    counterBeaconsReceived++;

    double dsr=mobility->getCurrentPosition().distance(wsm->getSenderPos());


    if (((strcmp(wsm->getNodeType(),"EnsRSU11p"))==0) && endRsuCoverageMsg->isScheduled() )
      cancelEvent(endRsuCoverageMsg);

    if(((strcmp(wsm->getNodeType(),"EnsRSU11p"))==0)){
        distanceNextRsu=dsr;
         inCoverage=true;
         idRsuService=wsm->getSenderAddress();
         scheduleAt(simTime()+2, endRsuCoverageMsg);
    }


    maxDistanceRsu[counterBeaconsReceived] = dsr;


    double pendiente=(mobility->getCurrentPosition().y-wsm->getSenderPos().y)/(mobility->getCurrentPosition().x-wsm->getSenderPos().x);

    string status=pendiente>0 ? "out":"in";

    EV<<"pendiente   = "<<pendiente<<"-"<<status<<endl;

    statusNodes[counterBeaconsReceived] = {dsr,status};


    if (ListBeacon.SearchBeacon(int(wsm->getSenderAddress()))){
        ListBeacon.UpdateBeacon(wsm->getSenderAddress(),
        wsm->getNodeType(),status,wsm->getArrivalTime(), wsm->getCreationTime(), wsm->getSenderAddress(),wsm->getSpeed(),
        wsm->getSenderPos().x, wsm->getSenderPos().y, wsm->getSenderPos().z, 0,dsr,0, 0,0, 0);
        ListBeacon.PurgeBeacons(neighborValidityInterval);
    }else{
        ListBeacon.AddBeacon(wsm->getSenderAddress(),
        wsm->getNodeType(),status,wsm->getArrivalTime(),  wsm->getCreationTime(),wsm->getSenderAddress(),wsm->getSpeed(),
        wsm->getSenderPos().x, wsm->getSenderPos().y, wsm->getSenderPos().z,0, dsr,0,0,0,0);
        ListBeacon.PurgeBeacons(neighborValidityInterval);
         }


    ListBeacon.SortBeacons();
    // ListBeacon.PrintBeacons();
    //read BitRate
    myBitrate=((DeciderResult80211*)((PhyToMacControlInfo*)wsm->getControlInfo())->getDeciderResult())->getBitrate();




    ///////////////////////////////////////////////////////////////
     // is it a new neighbor?
     bool isNewNeighbor = true;
     vector<uint> indices;





     for (uint i = 0; i < neighbors.size(); ++i) {
         WaveShortMessage* neighbor = neighbors[i];

         //if (neighbor->getCarId() == wsm->getCarId()) {
         if (neighbor->getSenderAddress() == wsm->getSenderAddress()) {
          // isNewNeighbor = false;
             neighbors.erase(neighbors.begin()+i);
         }
         else {
             // check for removal
             if (simTime() - neighbor->getArrivalTime() > neighborValidityInterval)
                 indices.push_back(i);
         }
     }


   // add new neighbor to neighbors list
   neighbors.push_back(wsm->dup());




  // remove the old neighbors
  WaveShortMessages newNeighborList;
  for (uint i = 0; i < neighbors.size(); ++i) {
      bool keepNeighbor = true;
      for (uint j = 0; j < indices.size(); ++j) {
          if (i == indices[j])
              keepNeighbor = false;
      }
      if (keepNeighbor)
          newNeighborList.push_back(neighbors[i]);
  }
  neighbors = newNeighborList;


  //Print Neigbors Table
  EV<<"_____________________________________________________________________________________ "<<endl;
  EV<<"                             Vehicle Neighbors Table "<<endl;
  EV<<"_____________________________________________________________________________________ "<<endl;
  EV<<'|' << setw(20) << "ArrivalTime"<<'|'<< setw(20) << "NodeType"<<'|'<< setw(10)<<" Sender"<<'|'<< setw(20)<<"Distance To RSU"<<'|'<<endl;
  for (std::vector<WaveShortMessage*>::const_iterator i = neighbors.begin(); i != neighbors.end(); ++i){
      WaveShortMessage* msg = *i;
      EV<<'|' << setw(20) << msg->getArrivalTime()<<'|' << setw(20)<<msg->getNodeType()<<'|'<< setw(10)<<msg->getSenderAddress()<<'|'<< setw(20)<<mobility->getCurrentPosition().distance(msg->getSenderPos())<<'|'<<endl;
  }
  EV<<"_____________________________________________________________________________________ "<<endl;


     ///////////////////////////////////////////////////////////////
     ///////////////////////////////////////////////////////////////

}



void EnsVehicle11p::handleLowerMsg(cMessage* msg) {

    if (!strcmp(msg->getName(), "beacon")) {

        onBeacon((WaveShortMessage*) msg);


    }
    else if(!strcmp(msg->getName(), "VideoStreamPacket")){
        EV<<"We receive a VideoStreamPacket   "<<endl;

        VideoStreamMessage* wsm = dynamic_cast<VideoStreamMessage*>(msg);
        EV<<"wsm->getSenderAddress()= "<<wsm->getSenderAddress()<<endl;
        EV<<"traci->getId()= "<<mobility->getId()<<endl;
        EV<<"wsm->getRecipientAddress()= "<<wsm->getRecipientAddress()<<endl;



        if (wsm->getRecipientAddress()==mobility->getId() ) {
            recvStream((VideoStreamMessage*) msg);
            LogToFile((VideoStreamMessage*) msg, false);
           }

    }
    else {
            ev << "unknown message (" << msg->getName() << ")  received\n";
        }
        delete(msg);


}

void EnsVehicle11p::recvStream(VideoStreamMessage *pkt) {
    // get packet fields
    uint16_t seqNumber = pkt->getSequenceNumber();

    bool isFragmentStart = pkt->getFragmentStart();
    bool isFragmentEnd = pkt->getFragmentEnd();
    long frameNumber = pkt->getFrameNumber();
    FrameType frameType = FrameType(pkt->getFrameType());
    long encodingNumber = frameEncodingNumber(frameNumber, numBFrames,
            frameType);

    // in the following, the frame statistics will be updated only when
    // - the end fragment has been received or
    // - the previously handled frame hasn't been properly processed
    int currentNumPacketsLost = 0;
    int currentNumFramesLost = 0;
    if (warmupFinished == false) {
        // handle warm-up flag based on both simulation time and GoP beginning
        if (simTime() >= simulation.getWarmupPeriod()) {
            if (frameType == I || frameType == IDR) {
                if (isFragmentStart == true) {
                    // initialize variables for handling sequence number and frame number
                    prevSequenceNumber = seqNumber;
                    prevPFrameNumber =
                            (frameNumber > 0) ?
                                    frameNumber - numBFrames - 1 : -1;
                    ///< to avoid loss of B frames after this frame
                    currentFrameNumber = frameNumber;
                    currentEncodingNumber = encodingNumber;
                    currentFrameType = frameType;

                    if (isFragmentEnd == true) {
                        // this frame consists of this packet only
                        currentFrameFinished = true;
                        prevIFrameNumber = frameNumber;
                        numFramesReceived++; ///< count the current frame as well
                    } else {
                        // more fragments to come!
                        currentFrameDiscard = false;
                        currentFrameFinished = false;
                        prevIFrameNumber = -1;
                    }

                    numPacketsReceived++;   ///< update packet statistics
                    warmupFinished = true;  ///< set the flag
                }   // end of fragmentStart check
            }   // end of frameType check
        }   // end of warm-up period check
    }   // end of warm-up flag check and related processing
    else {
        if (seqNumber != (prevSequenceNumber + 1) % 65536) {
            currentNumPacketsLost = (
                    seqNumber > prevSequenceNumber ?
                            seqNumber : prevSequenceNumber + 1)
                    - prevSequenceNumber;

            // detect loss of frame(s) between the one previously handled and the current one.
            // note that the previously handled frame could be the current one as well.
            if (encodingNumber > (currentEncodingNumber + 1) % numTraceFrames) {
                currentNumFramesLost = (
                        encodingNumber > currentEncodingNumber ?
                                encodingNumber : currentEncodingNumber)
                        - currentEncodingNumber - 1;
            }

            // check whether the previously handled frame should be discarded or not
            // as a result of current packet loss
            // TODO: implement the case for decoding threshold (DT) < 1
            if (currentFrameFinished == false) {
                currentNumFramesLost++;
            }

            // set frame discard flag for non-first packet of frame
            // TODO: implement the case for decoding threshold (DT) < 1
            if (isFragmentStart == false) {
                currentFrameDiscard = true;
            }
        }   // end of packet and frame loss detection and related-processing
        prevSequenceNumber = seqNumber; ///< update the sequence number

        switch (frameType) {
        case IDR:
        case I:
            if (isFragmentStart == true) {
                if (isFragmentEnd == true) {
                    // this frame consists of this packet only
                    currentFrameFinished = true;
                    prevIFrameNumber = frameNumber;
                    numFramesReceived++;    ///< count the current frame as well
                } else {
                    // more fragments to come!
                    currentFrameDiscard = false;
                    currentFrameFinished = false;
                }

                // update frame-related flags and variables
                currentFrameNumber = frameNumber;
                currentEncodingNumber = encodingNumber;
                currentFrameType = frameType;
            }   // end of processing of the first packet of I/IDR frame
            else {
                if (isFragmentEnd == true) {
                    if (currentFrameDiscard == false) {
                        // the frame has been received and decoded successfully
                        prevIFrameNumber = currentFrameNumber;
                        numFramesReceived++;
                    } else {
                        currentNumFramesLost++;
                    }
                    currentFrameFinished = true;
                }
            }   // end of processing of the non-first packet of I/IDR frame
            break;

        case P:
            if (isFragmentStart == true) {
                if (prevIFrameNumber == frameNumber - numBFrames - 1
                        || prevPFrameNumber == frameNumber - numBFrames - 1) {
                    // I or P frame that the current frame depends on was successfully decoded
                    currentFrameDiscard = false;

                    if (isFragmentEnd == true) {
                        currentFrameFinished = true; /// no more packet in this frame
                        prevPFrameNumber = frameNumber;
                        numFramesReceived++; ///< count the current frame as well
                    } else {
                        currentFrameFinished = false; ///< more fragments to come
                    }
                } else {
                    // the dependency check failed, so the current frame will be discarded
                    currentFrameDiscard = true;

                    if (isFragmentEnd == true) {
                        currentFrameFinished = true; /// no more packet in this frame
                        currentNumFramesLost++; ///< count the current frame as well
                    } else {
                        currentFrameFinished = false; ///< more fragments to come
                    }
                }

                // update frame-related flags and variables
                currentFrameNumber = frameNumber;
                currentEncodingNumber = encodingNumber;
                currentFrameType = frameType;
            }   // end of processing of the first packet of P frame
            else {
                if (isFragmentEnd == true) {
                    if (currentFrameDiscard == false) {
                        // the frame has been received and decoded successfully
                        prevPFrameNumber = currentFrameNumber;
                        numFramesReceived++;
                    } else {
                        currentNumFramesLost++;
                    }
                    currentFrameFinished = true;
                }
            }   // end of processing of the non-first packet of P frame
            break;

        case B:
            if (isFragmentStart == true) {
                // check frame dependency
                long lastDependonFrameNumber = (frameNumber / (numBFrames + 1))
                        * (numBFrames + 1);
                ///< frame number of the last I or P frame it depends on
                long nextDependonFrameNumber = lastDependonFrameNumber
                        + numBFrames + 1;
                ///< frame number of the next I or P frame it depends on
                bool passedDependency = false;
                if (nextDependonFrameNumber % gopSize == 0) {

                    // next dependent frame is I frame, so we need to check
                    // both next (I) and last frames.
                    if (prevPFrameNumber == lastDependonFrameNumber
                            && prevIFrameNumber == nextDependonFrameNumber) {
                        passedDependency = true;
                    }
                } else {
                    // next dependent frame is P frame, so we need to check
                    // only next (P) frame.
                    if (prevPFrameNumber == nextDependonFrameNumber) {
                        passedDependency = true;
                    }
                }

                if (passedDependency == true) {
                    if (isFragmentEnd == true) {
                        // this frame consists of this packet only
                        currentFrameFinished = true;
                        numFramesReceived++; ///< count the current frame as well
                    } else {
                        // more fragments to come!
                        currentFrameDiscard = false;
                        currentFrameFinished = false;
                    }
                } else {
                    // the dependency check failed, so the current frame will be discarded
                    currentFrameDiscard = true;

                    if (isFragmentEnd == true) {
                        // this frame consists of this packet only
                        currentFrameFinished = true;
                        currentNumFramesLost++; ///< count the current frame as well
                    } else {
                        // more fragments to come!
                        currentFrameFinished = false;
                    }
                }

                // update frame-related flags and variables
                currentFrameNumber = frameNumber;
                currentEncodingNumber = encodingNumber;
                currentFrameType = frameType;
            }   // end of processing of the first packet of B frame
            else {
                if (isFragmentEnd == true) {
                    if (currentFrameDiscard == false) {
                        // the frame has been received and decoded successfully
                        numFramesReceived++;
                    } else {
                        currentNumFramesLost++;
                    }
                    currentFrameFinished = true;
                }
            }   // end of processing of the non-first packet of B frame
            break;

        default:
            error("%s: Unexpected frame type: %d", getFullPath().c_str(),
                    frameType);
        }   // end of switch ()

        // update packet statistics
        numPacketsReceived++;
        numPacketsLost += currentNumPacketsLost;

        // update frame statistics
        numFramesDiscarded += currentNumFramesLost;

    }   // end of 'if (warmupFinshed == false)'

}



/*
 * Output the results to log files
 */
void EnsVehicle11p::LogToFile(VideoStreamMessage* msg, bool flag) {
    ofstream of;
    bool updateFlag = false;
    ReceivedMessage recvMsg(msg->getFrameNumber(), simTime(),
            msg->getFrameType(), msg->getByteLength(), msg->getFragmentEnd());

    list<ReceivedMessage>::iterator it;
    for (it = m_recvMessageList.begin(); it != m_recvMessageList.end(); it++) {
        ReceivedMessage tmpMsg = *it;
        if ((tmpMsg.number == msg->getFrameNumber())
                && (tmpMsg.type == msg->getFrameType()) && !tmpMsg.endFlag) {

            ReceivedMessage newMsg(0, 0, 0, 0, 0);
            newMsg.number = msg->getFrameNumber();
            newMsg.time = simTime();
            newMsg.endFlag = msg->getFragmentEnd();
            newMsg.size = tmpMsg.size + msg->getByteLength();
            newMsg.type = msg->getFrameType();

            m_recvMessageList.push_back(newMsg);
            m_recvMessageList.erase(it);
            updateFlag = true;
            break;
        } else if ((tmpMsg.number == msg->getFrameNumber())
                && (tmpMsg.type == msg->getFrameType()) && tmpMsg.endFlag) {
            return;
        }
    }

    if (!updateFlag)
        m_recvMessageList.push_back(recvMsg);

    if (msg->getFragmentEnd() == false)
        return;

    ReceivedMessage resultMsg(0, 0, 0, 0, 0);

    for (it = m_recvMessageList.begin(); it != m_recvMessageList.end(); it++) {
        resultMsg = *it;
        if (resultMsg.number == msg->getFrameNumber()) {
            break;
        }
    }

    VideoStreamMessage *job = new VideoStreamMessage("job");
    job->setByteLength(resultMsg.size);
    job->setFrameType(resultMsg.type);
    job->setArrivalTime(resultMsg.time);
    job->setFrameNumber(resultMsg.number);

    of.open(
            string("./results/")
                    + getParentModule()->getParentModule()->getFullName()
                    + string("-vehicle-") + getParentModule()->getFullName()
                    + string(".txt"), ios_base::app);
    if (of.is_open()) {
        switch (resultMsg.type) {
        case I:
            of << resultMsg.number << " " << resultMsg.time << " " << "I "
                    << resultMsg.size << " bytes " << resultMsg.endFlag << " "
                    << msg->getName() << endl;
            sourceBuffer(job);
            break;
        case IDR:
            of << resultMsg.number << " " << resultMsg.time << " " << "IDR "
                    << resultMsg.size << " bytes " << resultMsg.endFlag << " "
                    << msg->getName() << endl;
            sourceBuffer(job);
            break;
        case B:
            of << resultMsg.number << " " << resultMsg.time << " " << "B "
                    << resultMsg.size << " bytes " << resultMsg.endFlag << " "
                    << msg->getName() << endl;
            sourceBuffer(job);
            break;
        case P:
            of << resultMsg.number << " " << resultMsg.time << " " << "P "
                    << resultMsg.size << " bytes " << resultMsg.endFlag << " "
                    << msg->getName() << endl;
            sourceBuffer(job);
            break;
        default:
            break;
        }
    }

    of.close();

    frameCount ++;

    ofstream fc( string("./results/")
            + getParentModule()->getParentModule()->getFullName()
            +string("-vehicle")
            + string("-") + getParentModule()->getFullName()
            + string("-frame-packet.txt"), ios_base::app);

    fc << simTime() << " : " << frameCount << " : " << m_recvMessageList.size() << endl;
    fc.close();
}

// get the frame encoding number
long EnsVehicle11p::frameEncodingNumber(long frameNumber, int numBFrames,
        FrameType frameType) {
    long encodingNumber = 0;

    switch (frameType) {
    case IDR:
    case I:
        encodingNumber = frameNumber == 0 ? 0 : frameNumber - numBFrames;
        break;
    case P:
        encodingNumber = frameNumber - numBFrames;
        break;
    case B:
        encodingNumber = frameNumber + 1;
        break;
    default:
        error("%s: Unexpected frame type: %d", getFullPath().c_str(),
                frameType);
    } // end of switch ()

    return encodingNumber;
}



void EnsVehicle11p::statisticsRSUIn(){

    double d=mobility->getCurrentPosition().x;
    double d2=d-distanceTravelled.back();
    simtime_t t=simTime();
    simtime_t timeWithoutConnection=t-timeInOutRsu.back();
    rsuData r;
    r.setIdRsu(idRsuService);
    r.setTimeStartService(t);
    r.setSpeedIn(mobility->getSpeed());
    r.setTotalDownloadedIn(totalDownloadMB);
    r.setTotalQueueIn(totalQueueMB);
    r.setTotalPlayedIn(totalPlayedMB);
    r.setTimeWithOutService(timeWithoutConnection);
    r.setBitRate(myBitrate);
    r.setDistanceWithOutService(d2);
    statisticsRsuIn[simTime()]=r;
    distanceTravelled.push_back(d);
    timeInOutRsu.push_back(t);
}

void EnsVehicle11p::statisticsRSUOut(){
    double d=mobility->getCurrentPosition().x;
    double d2=d-distanceTravelled.back();
    simtime_t t=simTime();
    simtime_t timeWithConnection=t-timeInOutRsu.back();
    rsuData r;
    r.setIdRsu(idRsuService);
    r.setTimeEndService(t);

    r.setSpeedOut(mobility->getSpeed());

    r.setTotalDownloadedOut(totalDownloadMB);
    r.setTotalQueueOut(totalQueueMB);
    r.setTotalPlayedOut(totalPlayedMB);

    r.setTimeSpentInRsu(timeWithConnection);//t.dbl()-timeInOutRsu.back().dbl());

    r.setBitRate(myBitrate);
    r.setDistanceWithService(d2);

    statisticsRsuOut[simTime()]=r;
    distanceTravelled.push_back(d);
    timeInOutRsu.push_back(simTime());
}

void EnsVehicle11p::finish()
{
    // This function is called by OMNeT++ at the end of the simulation.
    ofstream fc( string("./results/")
           //  + getParentModule()->getParentModule()->getFullName()
             +string("Vehicle")
             + string("-") + getParentModule()->getFullName()
             + string("-Downloaded.txt"), ios_base::app);

    //Print Statistics from RSU

    fc <<"RSU Statistics"<< " in vehicle: " << getParentModule()->getIndex() <<" with Bitrate:"<<myBitrate<< endl;

    for(auto it_m1 = statisticsRsuIn.cbegin(), end_m1 = statisticsRsuIn.cend(),
         it_m2 = statisticsRsuOut.cbegin(), end_m2 = statisticsRsuOut.cend();
         it_m1 != end_m1 || it_m2 != end_m2;)
     {
         if(it_m1 != end_m1) {
             rsuData d=it_m1->second;
             fc <<"__________________________________________________________________________________________"<<endl;
             fc <<' ' << setw(25) << "Video downloaded of RSU"<<' '<< setw(20) << d.getIdRsu() <<' '<< endl;
             fc <<'|' << setw(25) << "Initial Speed"<<'|'<< setw(20) << d.getSpeedIn()<< "[m/s.]"<<'|'<< endl;
             fc <<'|' << setw(25) << "Time without service"<<'|'<< setw(20) << d.getTimeWithOutService()<< "[sec.]"<<'|'<< setw(30) << "Distance without service" <<'|'<< setw(15) << d.getDistanceWithOutService()<<"[  m.]" <<'|'<< endl;
             fc <<'|' << setw(25) << "Start Total Downloaded"<<'|'<< setw(20) << d.getTotalDownloadedIn() <<"[ MB.]"<<'|'<< setw(30) << "Start TotalQueue" <<'|'<< setw(15) << d.getTotalQueueIn() <<"[ MB.]"<<'|'<< setw(20) << "Start TotalPlayed" <<'|'<< setw(15) << d.getTotalPlayedIn() <<"[ MB.]"<<'|'<< endl;


             ++it_m1;
         }


         if(it_m2 != end_m2) {
             rsuData d=it_m2->second;
             fc <<'|' << setw(25) << "End Total Downloaded" <<'|'<< setw(20) << d.getTotalDownloadedOut() <<"[ MB.]"<<'|'<< setw(30) <<  "End TotalQueue" <<'|'<< setw(15) <<  d.getTotalQueueOut() <<"[ MB.]"<<'|'<< setw(20) <<  "End TotalPlayed"<<'|'<< setw(15) <<  d.getTotalPlayedOut() <<"[ MB.]"<<'|'<< endl;
             fc <<'|' << setw(25) << "Time with service" <<'|'<< setw(20) <<d.getTimeSpentInRsu()<<"[sec.]"<<'|'<< setw(30) << "Distance with service" <<'|'<< setw(15)<< d.getDistanceWithService() << "[  m.]"<<'|'<< endl;
             fc <<'|' << setw(25) << "Final Speed" <<'|'<< setw(20) << d.getSpeedOut()<< "[m/s.]"<<'|'<< endl;
             fc <<"__________________________________________________________________________________________"<<endl;

             ++it_m2;
         }


         fc << " "<< endl;
         fc << " "<< endl;
     }





     fc.close();

}

void EnsVehicle11p::requestServiceToRSU()
{

    if(inCoverage){
        traciVehicle->setSpeed(1);
           if (!sentMessage){
           // generate and send a reqVod packet
           WaveShortMessage *reqVod = new WaveShortMessage();
           reqVod->setName("ReqVod");

           reqVod->setIdVideo(100);
           reqVod->setLastFrameReceived(frameCount-1);
           reqVod->setSenderAddress(mobility->getId());
           reqVod->setSenderPos(mobility->getCurrentPosition());

           sendMessage(reqVod);
           statisticsRSUIn();


           }

       }else{

           sentMessage = false;
           traciVehicle->setSpeed(25);
       }


    EV<<"In RSU transmission coverage: "<<inCoverage<<endl;
    EV<<"Speed: "<<mobility->getSpeed()<<"m/s"<<endl;
    EV<<"Number of frames received: "<<frameCount<<" frames"<<endl;

}




