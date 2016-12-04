package se.oru.aass.lucia2016.utility;

import org.metacsp.multi.spatioTemporal.paths.Pose;

public class ParkingPoseLib {
	
	public static Pose getMoveAwayParkingPose(int robotID) {
		if(robotID == 1) return new Pose(4.18, 4.16, 0.0);
			//return new Pose(0.42, 4.30, 0.0);
		else if(robotID == 2)
			return new Pose(5.24, 3.68, 0.0);
		else
			return new Pose(6.35, 4.14, 0.0);
	}

}
