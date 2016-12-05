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
package se.oru.aass.lucia2016.meta;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
import java.util.Vector;
import java.util.logging.Logger;

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
import org.metacsp.multi.activity.ActivityComparator;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeVariable;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.PowerSet;
import org.metacsp.utility.logging.MetaCSPLogging;

import com.vividsolutions.jts.geom.Geometry;

import se.oru.aass.lucia2016.multi.ViewConstraintSolver;
import se.oru.aass.lucia2016.multi.ViewVariable;

public class ViewSchedulingMetaConstraint extends MetaConstraint {
	
	private static final long serialVersionUID = 5719994497319584156L;
	
	protected Vector<Activity> activities;
	
	public ViewSchedulingMetaConstraint(VariableOrderingH varOH, ValueOrderingH valOH) {
		super(varOH, valOH);
	}
	
	protected ConstraintNetwork[] binaryPeakCollection() {
		if (activities != null && !activities.isEmpty()) {
			Vector<ConstraintNetwork> ret = new Vector<ConstraintNetwork>();
			logger.finest("Doing binary peak collection with " + activities.size() + " activities...");
			Activity[] groundVars = activities.toArray(new Activity[activities.size()]);
			for (int i = 0; i < groundVars.length-1; i++) {
				for (int j = i+1; j < groundVars.length; j++) {
					Bounds bi = null;
					Bounds bj = null;
					if (groundVars[i] instanceof ViewVariable && groundVars[j] instanceof ViewVariable) {
						bi = new Bounds(groundVars[i].getTemporalVariable().getEST(), groundVars[i].getTemporalVariable().getEET());
						bj = new Bounds(groundVars[j].getTemporalVariable().getEST(), groundVars[j].getTemporalVariable().getEET());	
//						bi = new Bounds(groundVars[i].getTemporalVariable().getEST(), groundVars[i].getTemporalVariable().getLET());
//						bj = new Bounds(groundVars[j].getTemporalVariable().getEST(), groundVars[j].getTemporalVariable().getLET());						
					}
					else {
						bi = new Bounds(groundVars[i].getTemporalVariable().getEST(), groundVars[i].getTemporalVariable().getEET());
						bj = new Bounds(groundVars[j].getTemporalVariable().getEST(), groundVars[j].getTemporalVariable().getEET());						
					}
					if (bi.intersectStrict(bj) != null && isConflicting(new Activity[] {groundVars[i], groundVars[j]})) {
						ConstraintNetwork cn = new ConstraintNetwork(null);
						cn.addVariable(groundVars[i].getVariable());
						cn.addVariable(groundVars[j].getVariable());
						ret.add(cn);
					}
				}
			}
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
		// TODO Auto-generated method stub
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

		Geometry shape1 = ((GeometricShapeDomain)poly1.getDomain()).getGeometry();
		Geometry shape2 = ((GeometricShapeDomain)poly2.getDomain()).getGeometry();
		if(shape1.intersects(shape2))
			return true;
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
			ret = getViewVarTrajResolver(vv1, te2);
		}
		else if(var1 instanceof TrajectoryEnvelope && var2 instanceof ViewVariable){
			ViewVariable vv1 = (ViewVariable)var2;
			TrajectoryEnvelope te2 = (TrajectoryEnvelope)var1;
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

		//vv2.footprint MEETS vv2.moveaway && vv2.moveaway BEFORE vv1.footprint
		// OR
		//vv1.footprint BEFORE vv2.footprint

		ConstraintNetwork resolver1 = new ConstraintNetwork(null);
		AllenIntervalConstraint vv1FPBeforeVv2FP = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before);
		vv1FPBeforeVv2FP.setFrom(vv1);			
		vv1FPBeforeVv2FP.setTo(vv2);
		resolver1.addConstraint(vv1FPBeforeVv2FP);
		ret.add(resolver1);

		ConstraintNetwork resolver2 = new ConstraintNetwork(null);
		TrajectoryEnvelope moveOutTE = getMoveOut(vv2.getTrajectoryEnvelope().getRobotID());
		
		if(moveOutTE == null){
			VariablePrototype moveOut = new VariablePrototype(viewSolver.getTrajectoryEnvelopeSolver(), metaSolver.getPrefix() + vv2.getTrajectoryEnvelope().getRobotID(),vv2.getTrajectoryEnvelope().getFootprint(),vv2.getTrajectoryEnvelope().getRobotID());
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
		else {
			AllenIntervalConstraint beforeMoveout = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before);
			beforeMoveout.setFrom(moveOutTE);			
			beforeMoveout.setTo(vv1.getTrajectoryEnvelope());
			resolver2.addConstraint(beforeMoveout);
			
		}
		
		return ret;
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

}
