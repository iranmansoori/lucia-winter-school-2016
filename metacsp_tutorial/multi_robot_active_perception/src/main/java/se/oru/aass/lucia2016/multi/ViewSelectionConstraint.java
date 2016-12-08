package se.oru.aass.lucia2016.multi;

import org.metacsp.framework.BinaryConstraint;
import org.metacsp.framework.Constraint;

/**
 * This (unary) constraint restricts a {@link SelectionVariable} to be selected as a view pose.
 * 
 * @author iran
 *
 */
public class ViewSelectionConstraint extends BinaryConstraint{

	private static final long serialVersionUID = -4236978858344883532L;

	@Override
	public String getEdgeLabel() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Object clone() {
		ViewSelectionConstraint ret = new ViewSelectionConstraint();
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
