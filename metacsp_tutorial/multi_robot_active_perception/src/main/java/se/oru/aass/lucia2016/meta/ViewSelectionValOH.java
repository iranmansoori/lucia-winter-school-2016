package se.oru.aass.lucia2016.meta;

import java.util.Vector;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ValueOrderingH;
import org.metacsp.framework.Constraint;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;

import com.vividsolutions.jts.geom.Geometry;

import se.oru.aass.lucia2016.multi.ViewConstraint;
import se.oru.aass.lucia2016.multi.ViewVariable;


public class ViewSelectionValOH extends ValueOrderingH{

	private static double alpha = 0.2;
	
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
			vvs1.add(vv1);
		}
		double sumDist0 = getSumDistance(vvs0);
		double sumDist1 = getSumDistance(vvs1);
		
		double sumInfoGain0 = getSumInfoGain(vvs0);
		double sumInfoGain1 = getSumInfoGain(vvs1);
		
		double hr0 = (1 - alpha) * sumDist0 + alpha * sumInfoGain0;
		double hr1 = (1 - alpha) * sumDist1 + alpha * sumInfoGain1;
		
		if(hr0 < hr1) return 1;
		if(hr0 > hr1) return -1;
		return 0;
		
	}

	private double getSumInfoGain(Vector<ViewVariable> vvs) {
		double sum = 0.0; 
		for (int i = 0; i < vvs.size(); i++) {
			sum += vvs.get(i).getInfoGain();				
		}
		return sum;
	}

	private double getSumDistance(Vector<ViewVariable> vvs) {
		double sum = 0.0; 
		for (int i = 0; i < vvs.size(); i++) {
			for (int j = 0; j < vvs.size(); j++) {
				if(i == j) continue;
				Geometry shape1 = ((GeometricShapeDomain)vvs.get(i).getTrajectoryEnvelope().
						getEnvelopeVariable().getDomain()).getGeometry();
				Geometry shape2 = ((GeometricShapeDomain)vvs.get(j).getTrajectoryEnvelope().
						getEnvelopeVariable().getDomain()).getGeometry();
				sum += shape1.distance(shape2);
			}			
		}
		return sum;
	}	
		
	public static void setAlpha(double alpha) {
		ViewSelectionValOH.alpha = alpha;
	}
	

}
