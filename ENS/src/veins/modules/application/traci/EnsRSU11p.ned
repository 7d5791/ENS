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

simple EnsRSU11p extends BaseWaveApplLayer
{
    @class(EnsRSU11p);
    @display("i=block/app2");
    
     double startupDelay = default(30ms) @unit(ms);
        int numTraceFrames = default(125);
        int gopSize = default(16);
        int numBFrames = default(3);
        int fps = default(30);
        
 		double maxTime = default(50ms) @unit(ms); //header length of the application
        int minCount = default(1); //header length of the application
        double minDistance = default(50m) @unit(m); //header length of the application
		string traceFile = default("../videoTraces/test_verbose");
		string mode = default("server");
		
		double accidentStart = default(500s) @unit(s);
		double accidentDuration = default(100s) @unit(s);
		
	
        double neighborValidityInterval @unit("s") = default(30s);
        double maxJitter @unit("s") = default(1s);   
        
        // bit rate 
        double bitrate @unit(bps) = default(18 Mbps); 
        
        
}
