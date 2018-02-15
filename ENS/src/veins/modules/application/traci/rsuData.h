//
// Copyright (C) 2013-2017 Cristhian Iza <7d5791@gmail.com>
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

#ifndef rsuData_H_
#define rsuData_H_

#include <fstream>

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include <cstdlib>
#include <iostream>
#include <vector>




using namespace std;
//
class rsuData
{
private:
    int idRsu;
    simtime_t timeStartService;
    simtime_t timeEndService;
    double totalDownloadedIn;
    double totalDownloadedOut;
    double totalQueueIn;
    double totalQueueOut;
    double totalPlayedIn;
    double totalPlayedOut;
    double bitRate;
    double speedIn;
    double speedOut;
    simtime_t  timeWithOutService;
    simtime_t timeSpentInRsu;
    double distanceWithOutService;
    double distanceWithService;

public:
    rsuData(){};

    rsuData(int id, simtime_t tSS, simtime_t tES, double tDI,double tDO, double tQI, double tQO,double tPI, double tPO, double bR, double sI,double sO, simtime_t tWOS,simtime_t tSIR,double dWOS,double dWS) :
        idRsu(id), timeStartService(tSS), timeEndService(tES),totalDownloadedIn(tDI),totalDownloadedOut(tDO), totalQueueIn(tQI),totalQueueOut(tQO), totalPlayedIn(tPI),totalPlayedOut(tPO),
        bitRate(bR), speedIn(sI),speedOut(sO), timeWithOutService(tWOS), timeSpentInRsu(tSIR),distanceWithOutService(dWOS),distanceWithService(dWS)
    {}
    int getIdRsu(){return idRsu;}
    void setIdRsu(int id){idRsu=id;}

    simtime_t getTimeStartService(){return timeStartService;}
    void setTimeStartService(simtime_t tSS){timeStartService=tSS;}

    simtime_t getTimeEndService(){return timeEndService;}
    void setTimeEndService(simtime_t tES){timeEndService=tES;}

    simtime_t getTimeSpentInRsu(){return timeSpentInRsu;}
    void setTimeSpentInRsu(simtime_t tSIR){timeSpentInRsu=tSIR;}

    double getTotalDownloadedIn(){return totalDownloadedIn;}
    void setTotalDownloadedIn(double tDI){totalDownloadedIn=tDI;}

    double getTotalDownloadedOut(){return totalDownloadedOut;}
       void setTotalDownloadedOut(double tDO){totalDownloadedOut=tDO;}

    double getTotalQueueIn(){return totalQueueIn;}
    void setTotalQueueIn(double tQI){totalQueueIn=tQI;}

    double getTotalQueueOut(){return totalQueueOut;}
    void setTotalQueueOut(double tQO){totalQueueOut=tQO;}

    double getTotalPlayedIn(){return totalPlayedIn;}
    void setTotalPlayedIn(double tPI){totalPlayedIn=tPI;}

    double getTotalPlayedOut(){return totalPlayedOut;}
    void setTotalPlayedOut(double tPO){totalPlayedOut=tPO;}

    double getBitRate(){return bitRate;}
    void setBitRate(double bR){bitRate=bR;}

    double getSpeedIn(){return speedIn;}
    void setSpeedIn(double sI){speedIn=sI;}

    double getSpeedOut(){return speedOut;}
    void setSpeedOut(double sO){speedOut=sO;}

    simtime_t getTimeWithOutService(){return timeWithOutService;}
    void setTimeWithOutService(simtime_t tWOS){timeWithOutService=tWOS;}

    double getDistanceWithOutService(){return distanceWithOutService;}
    void setDistanceWithOutService(double dWOS){distanceWithOutService=dWOS;}

    double getDistanceWithService(){return distanceWithService;}
    void setDistanceWithService(double dWS){distanceWithService=dWS;}
};


#endif /* rsuData_H_ */
