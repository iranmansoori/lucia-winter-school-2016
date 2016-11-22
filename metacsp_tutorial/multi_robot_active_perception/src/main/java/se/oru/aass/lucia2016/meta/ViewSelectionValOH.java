package se.oru.aass.lucia2016.meta;

import java.util.Vector;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ValueOrderingH;
import org.metacsp.framework.Constraint;

import se.oru.aass.lucia2016.multi.ViewConstraint;
import se.oru.aass.lucia2016.multi.ViewVariable;


public class ViewSelectionValOH extends ValueOrderingH{

	@Override
	public int compare(ConstraintNetwork arg0, ConstraintNetwork arg1) {
		Constraint[] cons0 = arg0.getConstraints();
		Constraint[] cons1 = arg1.getConstraints();
		
		Vector<ViewVariable> vvs0 = new Vector<ViewVariable>();
		Vector<ViewVariable> vvs1 = new Vector<ViewVariable>();
		for (int i = 0; i < cons0.length; i++) {
			ViewVariable vv0 = (ViewVariable)((ViewConstraint)cons0[i]).getFrom();
			ViewVariable vv1 = (ViewVariable)((ViewConstraint)cons1[i]).getFrom();
			vvs0.add(vv0);
			vvs0.add(vv1);
		}
		
		int sumDist0 = getSumDistance(vvs0);
		int sumDist1 = getSumDistance(vvs1);
		
		return 0;
	}

	private int getSumDistance(Vector<ViewVariable> vvs0) {

		return 0;
	}

}
