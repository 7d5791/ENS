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

package org.car2x.veins.modules.application.traci;
import org.car2x.veins.modules.application.ieee80211p.BaseWaveApplLayer;

simple EnsVehicle11p extends BaseWaveApplLayer
{
    @class(EnsVehicle11p);
    @display("i=block/app2");
    bool sendWhileParking  = default(false); //send messages when the car is parked
    double maxTime = default(50ms) @unit(ms); //header length of the application
    int minCount = default(1); //header length of the application
    double minDistance = default(50m) @unit(m); //header length of the application
    string traceFile = default("../videoTraces/test_verbose2");
    string mode = default("client");
		
    double accidentStart = default(500s) @unit(s);
	double accidentDuration = default(100s) @unit(s);
		
    double startupDelay = default(30ms) @unit(ms);
    int numTraceFrames = default(125);
    int gopSize = default(16);
    int numBFrames = default(3);
    int fps = default(30);
  
        
    double rsu1X = default(2500m)@unit(m);
    double rsu1Y = default(20m) @unit(m);
        
    double rsu2X = default(3500m)@unit(m);
    double rsu2Y = default(20m)@unit(m);
        
    double rsu3X = default(4500m)@unit(m);
    double rsu3Y = default(20m)@unit(m);
        
         
    double neighborValidityInterval @unit("s") = default(30s);
    double maxJitter @unit("s") = default(1s);
        
        
    //@group(Queueing);
    volatile double serviceTime @unit(s);
    @signal[qlen](type="long");
    @signal[busy](type="bool");
    @signal[queueingTime](type="simtime_t");
    @statistic[qlen](title="queue length";record=vector,timeavg,max;interpolationmode=sample-hold);
    @statistic[busy](title="server busy state";record=vector?,timeavg;interpolationmode=sample-hold);
    @statistic[queueingTime](title="queueing time at dequeue";unit=s;record=vector,mean,max;interpolationmode=none);

    int capacity = default(-1);    // negative capacity means unlimited queue
    bool fifo = default(true);     // whether the module works as a queue (fifo=true) or a stack (fifo=false)
        
    //group(sink);
    @signal[lifetime](type="simtime_t");
    @statistic[lifetime](title="lifetime of arrived jobs"; unit=s; record=vector,mean,max; interpolationmode=none);
    
}
