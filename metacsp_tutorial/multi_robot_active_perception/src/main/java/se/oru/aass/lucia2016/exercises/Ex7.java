package se.oru.aass.lucia2016.exercises;

import java.util.ArrayList;
import java.util.Vector;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ValueOrderingH;
import org.metacsp.framework.Constraint;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;

import com.vividsolutions.jts.geom.Geometry;

import se.oru.aass.lucia2016.meta.ViewSelectionMetaConstraint;
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
public class Ex7 extends ValueOrderingH{
	
	@Override
	public int compare(ConstraintNetwork arg0, ConstraintNetwork arg1) {
		
		//This method is used by the value ordering heuristic to sort selections.
		//The two constraint networks below contain a possible selection
		// of ViewVariables to explore, which are to be compared		
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
		
		//TODO 1: add another criteria to information gain, which
		//        accounts for FoV overlap, e.g., compute degree of
		//        overlap of FoVs in the two selections being compared
		//You are given the following:
		// -- getAreaOfFoV, computes area of the FoV of a ViewVariable
		// -- getAreaOfFovIntersection, computes the area of the intersection of two ViewVariables' FoVs
		double alpha = 0.6;
		double separationDegree0 = getDegreeOfNonOverlappingOfFoVs(vvs0);
		double separationDegree1 = getDegreeOfNonOverlappingOfFoVs(vvs1);

		double hr0 = alpha*avgInfoGain0 + (1.0-alpha)*separationDegree0;
		double hr1 = alpha*avgInfoGain1 + (1.0-alpha)*separationDegree1;		
		
		if(hr0 < hr1) return 1;
		if(hr0 > hr1) return -1;
		return 0;
	}

	private double getDegreeOfNonOverlappingOfFoVs(Vector<ViewVariable> vvs) {
		double maxCover = 0.0;
		for (int i = 0; i < vvs.size(); i++) {
			for (int j = 0; j < vvs.size(); j++) {
				if(i == j) continue;
				double intArea = getAreaOfFovIntersection(vvs.get(i), vvs.get(j));
				double percentCover1 = intArea/getAreaOfFoV(vvs.get(i));
				double percentCover2 = intArea/getAreaOfFoV(vvs.get(j));
				double maxPercentCover = Math.max(percentCover1, percentCover2);
				if (maxPercentCover > maxCover) maxCover = maxPercentCover;
			}
		}
		return (1.0-maxCover);
	}
	
	private double getAverageInfoGain(Vector<ViewVariable> vvs) {
		double sum = 0.0; 
		for (int i = 0; i < vvs.size(); i++) {
			sum += vvs.get(i).getInfoGain();
		}
		return sum/(double)vvs.size();
	}
	
	
	private double getAreaOfFoV(ViewVariable vv) {
		Geometry shape = ((GeometricShapeDomain)vv.getFoV().getDomain()).getGeometry();
		return shape.getArea();
	}
	
	private double getAreaOfFovIntersection(ViewVariable vv1, ViewVariable vv2) {
		Geometry shape1 = ((GeometricShapeDomain)vv1.getFoV().getDomain()).getGeometry();
		Geometry shape2 = ((GeometricShapeDomain)vv2.getFoV().getDomain()).getGeometry();
		return shape1.intersection(shape2).getArea();
	}

}
