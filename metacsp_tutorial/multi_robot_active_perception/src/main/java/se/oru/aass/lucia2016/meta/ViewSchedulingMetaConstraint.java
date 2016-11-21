package se.oru.aass.lucia2016.meta;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Vector;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.ValueOrderingH;
import org.metacsp.framework.Variable;
import org.metacsp.framework.VariableOrderingH;
import org.metacsp.framework.meta.MetaConstraint;
import org.metacsp.framework.meta.MetaVariable;
import org.metacsp.meta.symbolsAndTime.Schedulable;
import org.metacsp.multi.activity.Activity;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeVariable;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;


import se.oru.aass.lucia2016.multi.ViewConstraintSolver;
import se.oru.aass.lucia2016.multi.ViewVariable;
import com.vividsolutions.jts.geom.Geometry;

public class ViewSchedulingMetaConstraint extends Schedulable{

	private HashMap<TrajectoryEnvelope,ArrayList<TrajectoryEnvelope>> refinedWith = new HashMap<TrajectoryEnvelope, ArrayList<TrajectoryEnvelope>>();
	private static final int MINIMUM_SIZE = 5;
	public ViewSchedulingMetaConstraint(VariableOrderingH varOH,
			ValueOrderingH valOH) {
		super(varOH, valOH);
		this.setPeakCollectionStrategy(PEAKCOLLECTION.BINARY);
	}

	@Override
	public boolean isConflicting(Activity[] peak) {
		if (peak.length < 2) return false;
		if(peak[0] instanceof ViewVariable && peak[1] instanceof ViewVariable){
			ViewVariable vv1 = (ViewVariable)peak[0];
			ViewVariable vv2 = (ViewVariable)peak[1];
			if (vv1.getTrajectoryEnvelope().getRobotID() == vv2.getTrajectoryEnvelope().getRobotID()) return false;
			
			if(checkSpatialInterstion(vv1.getTrajectoryEnvelope().getEnvelopeVariable(), vv2.getFoV()))
				return true;
			if(checkSpatialInterstion(vv1.getFoV(), vv2.getTrajectoryEnvelope().getEnvelopeVariable()))
				return true;
			else 
				return false;			
					
		}
		else if(peak[0] instanceof ViewVariable && peak[1] instanceof TrajectoryEnvelope){
			ViewVariable vv1 = (ViewVariable)peak[0];
			TrajectoryEnvelope vv2 = (TrajectoryEnvelope)peak[1];
			if (vv1.getTrajectoryEnvelope().getRobotID() == vv2.getRobotID()) return false;			
			if(checkSpatialInterstion(vv1.getFoV(), vv2.getEnvelopeVariable()))
				return true;
			else 
				return false;			
			
		}
		else if(peak[0] instanceof TrajectoryEnvelope && peak[1] instanceof ViewVariable){
			ViewVariable vv1 = (ViewVariable)peak[1];
			TrajectoryEnvelope vv2 = (TrajectoryEnvelope)peak[0];
			if (vv1.getTrajectoryEnvelope().getRobotID() == vv2.getRobotID()) return false;			
			if(checkSpatialInterstion(vv1.getFoV(), vv2.getEnvelopeVariable()))
				return true;
			else 
				return false;			
			
		}else{
			return false;
		}
		
	}
	
	private boolean checkSpatialInterstion(GeometricShapeVariable poly1, GeometricShapeVariable poly2){

		Geometry shape1 = ((GeometricShapeDomain)poly1.getDomain()).getGeometry();
		Geometry shape2 = ((GeometricShapeDomain)poly2.getDomain()).getGeometry();
		if(shape1.intersects(shape2))
			return true;
		return false;
	}
	
	@Override
	public ConstraintNetwork[] getMetaValues(MetaVariable metaVariable) {
		
		ConstraintNetwork conflict = metaVariable.getConstraintNetwork();
		Vector<ConstraintNetwork> ret = new Vector<ConstraintNetwork>();
		ConstraintNetwork resolver1 = new ConstraintNetwork(null);
		ConstraintNetwork resolver2 = new ConstraintNetwork(null);

		
		
//		ViewVariable vv1 = (ViewVariable)conflict.getVariables()[0];
//		ViewVariable vv2 = (ViewVariable)conflict.getVariables()[1];
		
//		refineTrajectoryEnvelopes(moveOut1TE, vv2.getFoV());
//		refineTrajectoryEnvelopes(moveOut2TE, vv1.getFoV());

		Variable vv1 = conflict.getVariables()[0];
		Variable vv2 = conflict.getVariables()[1];
		TrajectoryEnvelope te1 = null;
		TrajectoryEnvelope te2 = null;
		if(vv1 instanceof ViewVariable) te1 = ((ViewVariable)vv1).getTrajectoryEnvelope();
		else te1 = (TrajectoryEnvelope)vv1;
		if(vv2 instanceof ViewVariable) te2 = ((ViewVariable)vv2).getTrajectoryEnvelope();
		else te2 = (TrajectoryEnvelope)vv2;
		
		
		
		
		AllenIntervalConstraint before01 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before);
		before01.setFrom(te1);			
		before01.setTo(te2);
		resolver1.addConstraint(before01);

		
		AllenIntervalConstraint before10 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before);
		before10.setFrom(te2);			
		before10.setTo(te1);
		resolver2.addConstraint(before10);

		
		//this is not neccesary correct, it needs double check which one is conflicting with others
		ret.add(resolver1);
		ret.add(resolver2);
		
		
		return ret.toArray(new ConstraintNetwork[ret.size()]);
	}


	@Override
	public void draw(ConstraintNetwork network) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public ConstraintSolver getGroundSolver() {
		return (((ViewConstraintSolver)this.metaCS.getConstraintSolvers()[0]));
	}

	@Override
	public String toString() {
		return this.getClass().getName(); 
	}

	@Override
	public String getEdgeLabel() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Object clone() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public boolean isEquivalent(Constraint c) {
		// TODO Auto-generated method stub
		return false;
	}


	@Override
	public void markResolvedSub(MetaVariable metaVariable,
			ConstraintNetwork metaValue) {
		// TODO Auto-generated method stub

	}
	




}
