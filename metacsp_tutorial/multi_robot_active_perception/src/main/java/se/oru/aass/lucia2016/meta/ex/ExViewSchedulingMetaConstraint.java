/*******************************************************************************
 * Copyright (c) 2010-2013 Federico Pecora <federico.pecora@oru.se>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ******************************************************************************/
package se.oru.aass.lucia2016.meta.ex;

import java.util.ArrayList;
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
import org.metacsp.multi.activity.Activity;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatial.DE9IM.DE9IMRelation;
import org.metacsp.multi.spatial.DE9IM.DE9IMRelationSolver;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeVariable;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.time.Bounds;

import se.oru.aass.lucia2016.meta.ViewCoordinator;
import se.oru.aass.lucia2016.multi.ViewConstraintSolver;
import se.oru.aass.lucia2016.multi.ViewVariable;

import com.vividsolutions.jts.geom.Geometry;

/**
 * This {@link MetaConstraint} identifies {@link TrajectoryEnvelope}s that overlap in space and time. These may either be {@link TrajectoryEnvelope}s underlying {@link ViewVariable}s
 * (thus, representing the stationary pose of a robot while it is sensing), or {@link TrajectoryEnvelope}s that represent the motion of robots. It resolves spatio-temporally overlapping
 * variables by posting {@link AllenIntervalConstraint}s of type {@link AllenIntervalConstraint.Type#Before}, or by posting {@link TrajectoryEnvelope}s that make robots move away from
 * the offending pose. 
 * 
 * @author iran
 *
 */
public class ExViewSchedulingMetaConstraint extends MetaConstraint {
	
	private static final long serialVersionUID = 5719994497319584156L;
	
	protected Vector<Activity> activities;
	
	public ExViewSchedulingMetaConstraint (VariableOrderingH varOH, ValueOrderingH valOH) {
		super(varOH, valOH);
	}
	
	protected ConstraintNetwork[] binaryPeakCollection() {
		if (activities != null && !activities.isEmpty()) {
			Vector<ConstraintNetwork> ret = new Vector<ConstraintNetwork>();
			logger.finest("Doing binary peak collection with " + activities.size() + " activities...");
			Activity[] groundVars = activities.toArray(new Activity[activities.size()]);
			//TODO 1 : check for temporal intersection
			//--there is how you get the temporal bounds e.g., Earliest Start Time (EST) or Earliest End Time (EET)			
			//long est = groundVars[INDEX].getTemporalVariable().getEST()			
			//long eet = groundVars[INDEX].getTemporalVariable().getEST()
			
			
			//you need to check for spatial and temporal intersection
			//to check spatial intersection you need to use "isConflicting" method
			//partial implementation of isConflicting is given, you should complete it 
			
			//--notice that you have to return an array of constraint network, 
			//--so create a constraint network with appropriates variables 
			//--and add it to the Vector<ConstraintNetwork> ret 
			//--this is how you create a ConstraintNetwork and add a variable to that		
			//ConstraintNetwork cn = new ConstraintNetwork(null);
			//cn.addVariable(groundVars[INDEX].getVariable());
			
			
//			for (int i = 0; i < groundVars.length-1; i++) {
//				for (int j = i+1; j < groundVars.length; j++) {
//					Bounds bi = new Bounds(groundVars[i].getTemporalVariable().getEST(), groundVars[i].getTemporalVariable().getEET());
//					Bounds bj = new Bounds(groundVars[j].getTemporalVariable().getEST(), groundVars[j].getTemporalVariable().getEET());						
//					if (bi.intersectStrict(bj) != null && isConflicting(new Activity[] {groundVars[i], groundVars[j]})) {
//						ConstraintNetwork cn = new ConstraintNetwork(null);
//						cn.addVariable(groundVars[i].getVariable());
//						cn.addVariable(groundVars[j].getVariable());
//						ret.add(cn);
//					}
//				}
//			}
			if (!ret.isEmpty()) {
				return ret.toArray(new ConstraintNetwork[ret.size()]);			
			}
		}
		return (new ConstraintNetwork[0]);
	}
		
	
	@Override
	public ConstraintNetwork[] getMetaVariables() {
		return binaryPeakCollection();
	}

	@Override
	public void markResolvedSub(MetaVariable con, ConstraintNetwork metaValue) {
	}
	
	
	
	public void setUsage(Activity... acts) {
		if (activities == null) activities = new Vector<Activity>();
		for (Activity act : acts) 
			if (!activities.contains(act)) 
				activities.add(act);
	}

	public void removeUsage(Activity... acts) {
		if (activities != null) {
			for (Activity act : acts) activities.removeElement(act);
		}
	}
	
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
		//TODO 2 : check for the spatial intersection
		//with DE9IMRelation.getRelations(poly1, poly2), you can get all spatial relations between two polygons poly1 and poly2
		
//		for (DE9IMRelation.Type t : DE9IMRelation.getRelations(poly1, poly2)) {
//			if (t.equals(DE9IMRelation.Type.Disjoint)) return false;
//		}
//		return true;
		
		return false;
	}
	
	@Override
	public ConstraintNetwork[] getMetaValues(MetaVariable metaVariable) {
		ConstraintNetwork conflict = metaVariable.getConstraintNetwork();
		ArrayList<ConstraintNetwork> ret = new ArrayList<ConstraintNetwork>();
		
		Variable var1 = conflict.getVariables()[0];
		Variable var2 = conflict.getVariables()[1];
		
		//Two view variables
		if(var1 instanceof ViewVariable && var2 instanceof ViewVariable) {
			boolean vv1SeesVv2 = false;
			boolean vv2SeesVv1 = false;
			ViewVariable vv1 = (ViewVariable)var1;
			ViewVariable vv2 = (ViewVariable)var2;
			ArrayList<ConstraintNetwork> allResolvers = new ArrayList<ConstraintNetwork>();
			if(checkSpatialInterstion(vv1.getFoV(), vv2.getTrajectoryEnvelope().getEnvelopeVariable())) {
				vv1SeesVv2 = true;
				for (ConstraintNetwork resolver : getViewVarViewVarResolver(vv1, vv2)) {
					allResolvers.add(resolver);
				}				
			}
			if(checkSpatialInterstion(vv1.getTrajectoryEnvelope().getEnvelopeVariable(), vv2.getFoV())) {
				vv2SeesVv1 = true;
				for (ConstraintNetwork resolver : getViewVarViewVarResolver(vv2, vv1)) {
					allResolvers.add(resolver);
				}
			}					
			if (vv1SeesVv2 && vv2SeesVv1) {
				for (ConstraintNetwork cn : allResolvers) {					
					if (cn.getConstraints().length > 1) {						
						ret.add(cn);
					}
					else if (cn.getConstraints()[0].getScope()[0] instanceof TrajectoryEnvelope && cn.getConstraints()[0].getScope()[1] instanceof TrajectoryEnvelope) {						
						ret.add(cn);
					}
				}
			}
			else {
				ret = allResolvers;
			}
		}
		
		//view variable sees a move variable
		else if(var1 instanceof ViewVariable && var2 instanceof TrajectoryEnvelope){
			ViewVariable vv1 = (ViewVariable)var1;
			TrajectoryEnvelope te2 = (TrajectoryEnvelope)var2;
			
			//TODO 4 : implement the temporal resolver between two intersecting viewVariable and a TrajectoryEnvelope 
			ret = getViewVarTrajResolver(vv1, te2);
		}
		else if(var1 instanceof TrajectoryEnvelope && var2 instanceof ViewVariable){
			ViewVariable vv1 = (ViewVariable)var2;
			TrajectoryEnvelope te2 = (TrajectoryEnvelope)var1;
			
			//TODO 5 : implement the temporal resolver between two intersecting viewVariable and a TrajectoryEnvelope
			ret = getViewVarTrajResolver(vv1, te2);
		}				
		
		return ret.toArray(new ConstraintNetwork[ret.size()]);
	}

	private ArrayList<ConstraintNetwork> getViewVarTrajResolver(ViewVariable vv1, TrajectoryEnvelope te2) {
		ArrayList<ConstraintNetwork> ret = new ArrayList<ConstraintNetwork>();

		ConstraintNetwork resolver1 = new ConstraintNetwork(null);
		AllenIntervalConstraint vv1FPBeforeTe2 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before);
		vv1FPBeforeTe2.setFrom(vv1.getTrajectoryEnvelope());			
		vv1FPBeforeTe2.setTo(te2);
		resolver1.addConstraint(vv1FPBeforeTe2);
		ret.add(resolver1);
		
		ConstraintNetwork resolver2 = new ConstraintNetwork(null);
		AllenIntervalConstraint te2FPBeforeVv1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before);
		te2FPBeforeVv1.setFrom(te2);			
		te2FPBeforeVv1.setTo(vv1.getTrajectoryEnvelope());
		resolver2.addConstraint(te2FPBeforeVv1);
		ret.add(resolver2);
		
		return ret;
	}
	
	//Cone of vv1 intersects footprint of vv2
	private ArrayList<ConstraintNetwork> getViewVarViewVarResolver(ViewVariable vv1, ViewVariable vv2) {
		ViewCoordinator metaSolver = ((ViewCoordinator)this.metaCS);		
		ViewConstraintSolver viewSolver= (ViewConstraintSolver)this.getGroundSolver();
		ArrayList<ConstraintNetwork> ret = new ArrayList<ConstraintNetwork>();

		//RESOLVER 1: vv2.footprint MEETS vv2.moveaway && vv2.moveaway BEFORE vv1.footprint
		// OR
		//RESOLVER 2: vv1.footprint BEFORE vv2.footprint

		ConstraintNetwork resolver1 = new ConstraintNetwork(null);
		
		AllenIntervalConstraint vv1FPBeforeVv2FP = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before);
		vv1FPBeforeVv2FP.setFrom(vv1);			
		vv1FPBeforeVv2FP.setTo(vv2);
		resolver1.addConstraint(vv1FPBeforeVv2FP);		
		ret.add(resolver1);
		

		ConstraintNetwork resolver2 = new ConstraintNetwork(null);
		TrajectoryEnvelope moveAwayTE = getMoveOut(vv2.getTrajectoryEnvelope().getRobotID());
		
		if(moveAwayTE == null){
			VariablePrototype moveOut = new VariablePrototype(viewSolver.getTrajectoryEnvelopeSolver(), metaSolver.getPrefix() + vv2.getTrajectoryEnvelope().getRobotID(),vv2.getTrajectoryEnvelope().getFootprint(),vv2.getTrajectoryEnvelope().getRobotID(), false);
			resolver2.addVariable(moveOut);					

			AllenIntervalConstraint beforeMoveout = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before);
			beforeMoveout.setFrom(moveOut);			
			beforeMoveout.setTo(vv1.getTrajectoryEnvelope());
			resolver2.addConstraint(beforeMoveout);
			
			AllenIntervalConstraint senseMeetsMoveAway = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Meets);
			senseMeetsMoveAway.setFrom(vv2.getTrajectoryEnvelope());
			senseMeetsMoveAway.setTo(moveOut);
			resolver2.addConstraint(senseMeetsMoveAway);
			
		}
		else getViewVarViewVarResolverHelper(vv1, moveAwayTE, resolver2);

		ret.add(resolver2);
		return ret;
	}
	
	private void getViewVarViewVarResolverHelper(ViewVariable vv1, TrajectoryEnvelope moveOutTE, ConstraintNetwork resolver) {
		AllenIntervalConstraint beforeMoveout = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before);
		beforeMoveout.setFrom(moveOutTE);			
		beforeMoveout.setTo(vv1.getTrajectoryEnvelope());
		resolver.addConstraint(beforeMoveout);	
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
		return null;
	}

	@Override
	public Object clone() {
		return null;
	}

	@Override
	public boolean isEquivalent(Constraint c) {
		return false;
	}

}
