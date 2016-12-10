package se.oru.aass.lucia2016.meta;

import java.util.ArrayList;
import java.util.Vector;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ValueOrderingH;
import org.metacsp.framework.Constraint;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;

import com.vividsolutions.jts.geom.Geometry;

import se.oru.aass.lucia2016.multi.ViewSelectionConstraint;
import se.oru.aass.lucia2016.multi.ViewVariable;

/**
 * This heuristic is used by the {@link ViewSelectionMetaConstraint}. It
 * prefers selection of views that maximize a linear combination of
 * information gain and physical distance.
 *   
 * @author iran
 *
 */
public class ViewSelectionValOH extends ValueOrderingH{

	private static double alpha = 0.0;
	//private static double beta = 0.7;
	
	@Override
	public int compare(ConstraintNetwork arg0, ConstraintNetwork arg1) {
		Constraint[] cons0 = arg0.getConstraints();
		Constraint[] cons1 = arg1.getConstraints();
		
		Vector<ViewVariable> vvs0 = new Vector<ViewVariable>();
		Vector<ViewVariable> vvs1 = new Vector<ViewVariable>();
		for (int i = 0; i < cons0.length; i++) {
			ViewVariable vv0 = (ViewVariable)((ViewSelectionConstraint)cons0[i]).getFrom();
			ViewVariable vv1 = (ViewVariable)((ViewSelectionConstraint)cons1[i]).getFrom();
			vvs0.add(vv0);
			vvs1.add(vv1);
		}
		
		double avgInfoGain0 = getAverageInfoGain(vvs0);
		double avgInfoGain1 = getAverageInfoGain(vvs1);
		
		double separationDegree0 = getDegreeOfNonOverlappingOfFoVs(vvs0);
		double separationDegree1 = getDegreeOfNonOverlappingOfFoVs(vvs1);

		double hr0 = alpha*avgInfoGain0 + (1.0-alpha)*separationDegree0;
		double hr1 = alpha*avgInfoGain1 + (1.0-alpha)*separationDegree1;
		
		System.out.println("Comparing (" + avgInfoGain0 + "," + separationDegree0 + ") and (" + avgInfoGain1 + "," + separationDegree1 + ")");

		if(hr0 < hr1) return 1;
		if(hr0 > hr1) return -1;
		return 0;
	}

	private double getAverageInfoGain(Vector<ViewVariable> vvs) {
		double sum = 0.0; 
		for (int i = 0; i < vvs.size(); i++) {
			sum += vvs.get(i).getInfoGain();
		}
		return sum/(double)vvs.size();
	}
	
	private double getDegreeOfNonOverlappingOfFoVs(Vector<ViewVariable> vvs) {
		double maxCover = 0.0;
		for (int i = 0; i < vvs.size(); i++) {
			for (int j = 0; j < vvs.size(); j++) {
				if(i == j) continue;
				Geometry shape1 = ((GeometricShapeDomain)vvs.get(i).getTrajectoryEnvelope().
						getEnvelopeVariable().getDomain()).getGeometry();
				Geometry shape2 = ((GeometricShapeDomain)vvs.get(j).getTrajectoryEnvelope().
						getEnvelopeVariable().getDomain()).getGeometry();
				double intArea = shape1.intersection(shape2).getArea();
				System.out.println("INT AREA IS " + intArea);
				double percentCover1 = intArea/shape1.getArea();
				double percentCover2 = intArea/shape2.getArea();
				double maxPercentCover = Math.max(percentCover1, percentCover2);
				if (maxPercentCover > maxCover) maxCover = maxPercentCover;
			}
		}
		return (1.0-maxCover);
	}
		
	public static void setAlpha(double alpha) {
		ViewSelectionValOH.alpha = alpha;
	}
	

}
