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
import org.metacsp.framework.VariablePrototype;
import org.metacsp.framework.meta.MetaConstraint;
import org.metacsp.framework.meta.MetaVariable;
import org.metacsp.meta.symbolsAndTime.Schedulable;
import org.metacsp.multi.activity.Activity;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeVariable;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;


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
			if(vv1.getTrajectoryEnvelope().getRobotID() == -1 || vv2.getTrajectoryEnvelope().getRobotID() == -1) return false;
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
			if(vv1.getTrajectoryEnvelope().getRobotID() == -1 || vv2.getRobotID() == -1) return false;
			if (vv1.getTrajectoryEnvelope().getRobotID() == vv2.getRobotID()) return false;			
			if(checkSpatialInterstion(vv1.getFoV(), vv2.getEnvelopeVariable()))
				return true;
			else 
				return false;			
			
		}
		else if(peak[0] instanceof TrajectoryEnvelope && peak[1] instanceof ViewVariable){
			ViewVariable vv1 = (ViewVariable)peak[1];
			TrajectoryEnvelope vv2 = (TrajectoryEnvelope)peak[0];
			if(vv1.getTrajectoryEnvelope().getRobotID() == -1 || vv2.getRobotID() == -1) return false;
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
		
		Variable vv1 = conflict.getVariables()[0];
		Variable vv2 = conflict.getVariables()[1];
		TrajectoryEnvelope te1 = null;
		TrajectoryEnvelope te2 = null;
		if(vv1 instanceof ViewVariable){
			te1 = ((ViewVariable)vv1).getTrajectoryEnvelope();
		}
		else te1 = (TrajectoryEnvelope)vv1;
		if(vv2 instanceof ViewVariable) {
			te2 = ((ViewVariable)vv2).getTrajectoryEnvelope();			
		}
		else te2 = (TrajectoryEnvelope)vv2;
		
		//moveOut
		
		ConstraintNetwork resolver1 = getResolver(te1, te2);
		ConstraintNetwork resolver2 = getResolver(te2, te1);

		ret.add(resolver1);
		ret.add(resolver2);
		
		return ret.toArray(new ConstraintNetwork[ret.size()]);
	}


	private ConstraintNetwork getResolver(TrajectoryEnvelope te1, TrajectoryEnvelope te2) {
		ViewCoordinator metaSolver = ((ViewCoordinator)this.metaCS);		
		ViewConstraintSolver viewSolver= (ViewConstraintSolver)this.getGroundSolver();
		ConstraintNetwork resolver = new ConstraintNetwork(null);

		AllenIntervalConstraint before = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before);
		before.setFrom(te1);			
		before.setTo(te2);
		resolver.addConstraint(before);

		TrajectoryEnvelope moveOutTE = getMoveOut(te1.getRobotID());
		if(moveOutTE == null){
			VariablePrototype moveOut = new VariablePrototype(viewSolver.getTrajectoryEnvelopeSolver(), metaSolver.getPrefix() + te1.getRobotID(),te1.getFootprint(),te1.getRobotID());
			resolver.addVariable(moveOut);					

			AllenIntervalConstraint beforeMoveout = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before);
			beforeMoveout.setFrom(moveOut);			
			beforeMoveout.setTo(te2);
			resolver.addConstraint(beforeMoveout);
			
			AllenIntervalConstraint senseBeforeMoveAway = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before);
			senseBeforeMoveAway.setFrom(te1);
			senseBeforeMoveAway.setTo(moveOut);
			resolver.addConstraint(senseBeforeMoveAway);				
		}else{
			
			AllenIntervalConstraint beforeMoveout = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before);
			beforeMoveout.setFrom(moveOutTE);			
			beforeMoveout.setTo(te2);
			resolver.addConstraint(beforeMoveout);
			
			AllenIntervalConstraint senseBeforeMoveOut = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before);
			senseBeforeMoveOut.setFrom(te1);
			senseBeforeMoveOut.setTo(moveOutTE);
			resolver.addConstraint(senseBeforeMoveOut);				
			
		}
		
		return resolver;
		
	}

	private TrajectoryEnvelope getMoveOut(int robotID) {
		TrajectoryEnvelopeSolver teSolver = ((ViewConstraintSolver)this.getGroundSolver()).getTrajectoryEnvelopeSolver();
		Variable[] tes = teSolver.getVariables();
		for (int i = 0; i < tes.length; i++) {
			TrajectoryEnvelope te = (TrajectoryEnvelope)tes[i];
			if(te.getMarking() != null && te.getMarking().equals("moveOut") && te.getRobotID() == robotID){
				return te;
			}			
		}		
		return null;
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
