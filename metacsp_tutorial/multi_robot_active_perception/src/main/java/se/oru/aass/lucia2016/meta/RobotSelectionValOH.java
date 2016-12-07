package se.oru.aass.lucia2016.meta;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ValueOrderingH;


import se.oru.aass.lucia2016.multi.RobotConstraint;
import se.oru.aass.lucia2016.multi.ViewVariable;

public class RobotSelectionValOH extends ValueOrderingH{

	
	
	ViewCoordinator metaSolver = null;
	
	@Override
	public int compare(ConstraintNetwork arg0, ConstraintNetwork arg1) {
		Constraint[] cons0 = arg0.getConstraints();
		Constraint[] cons1 = arg1.getConstraints();		
		
		double sum0 = getSumDist(cons0);
		double sum1 = getSumDist(cons1);
		
		if(sum0 < sum1) return -1;
		else if(sum0 > sum1) return 1;		
		return 0;
	}

	private double getSumDist(Constraint[] cons) {
		double dist = 0;
		for (int i = 0; i < cons.length; i++) {
			RobotConstraint rc =(RobotConstraint)cons[i];
			ViewVariable vv = (ViewVariable)rc.getFrom();
			org.metacsp.multi.spatioTemporal.paths.Pose vvPose = vv.getTrajectoryEnvelope().getTrajectory().getPose()[0];
			geometry_msgs.Pose robotPose = metaSolver.getRobotsCurrentPose().get(rc.getRobotId());			
			dist += Math.sqrt(Math.pow(vvPose.getX() - robotPose.getPosition().getX(), 2) + Math.pow(vvPose.getY() - robotPose.getPosition().getY(), 2));			
		}
		return dist;
	}

	public void setMetaSolver(ViewCoordinator metaSolver) {		
		this.metaSolver = metaSolver;		
	}

}
