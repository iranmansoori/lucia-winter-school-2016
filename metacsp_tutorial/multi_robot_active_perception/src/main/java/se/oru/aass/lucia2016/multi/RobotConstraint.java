package se.oru.aass.lucia2016.multi;

import org.metacsp.framework.BinaryConstraint;
import org.metacsp.framework.Constraint;

public class RobotConstraint extends BinaryConstraint {

	private int robotId = 0;
	
	/**
	 * 
	 */
	private static final long serialVersionUID = -2844997478387290946L;

	public RobotConstraint(int robotId){
		this.robotId = robotId;
	}
	
	public int getRobotId(){
		return this.robotId;
	}
	
	@Override
	public String getEdgeLabel() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Object clone() {
		RobotConstraint ret = new RobotConstraint(this.robotId);
		ret.setFrom(this.getFrom());
		ret.setTo(this.getTo());
		return ret;
	}

	@Override
	public boolean isEquivalent(Constraint c) {
		// TODO Auto-generated method stub
		return false;
	}
	
	@Override
	public String toString(){
		return this.getFrom() + " has Robot Id " + this.robotId; 
	}

}
