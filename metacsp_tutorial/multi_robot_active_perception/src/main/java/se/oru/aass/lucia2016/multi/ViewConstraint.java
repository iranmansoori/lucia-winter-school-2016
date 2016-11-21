package se.oru.aass.lucia2016.multi;

import org.metacsp.framework.BinaryConstraint;
import org.metacsp.framework.Constraint;
import org.metacsp.multi.spatial.DE9IM.DE9IMRelation;

public class ViewConstraint extends BinaryConstraint{

	@Override
	public String getEdgeLabel() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Object clone() {
		ViewConstraint ret = new ViewConstraint();
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
	public String toString() {
		return this.getFrom() + " is selected";
	}

}
