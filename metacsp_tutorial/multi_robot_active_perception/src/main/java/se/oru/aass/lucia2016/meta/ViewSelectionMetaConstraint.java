package se.oru.aass.lucia2016.meta;

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
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;
import org.metacsp.utility.Combination;

import com.vividsolutions.jts.geom.Geometry;

import se.oru.aass.lucia2016.multi.ViewConstraint;
import se.oru.aass.lucia2016.multi.ViewConstraintSolver;
import se.oru.aass.lucia2016.multi.ViewVariable;



public class ViewSelectionMetaConstraint extends MetaConstraint{

	/**
	 * Iran
	 */
	private static final long serialVersionUID = 3967833964891433564L;
	private int robotNumber = 0;
	
	public ViewSelectionMetaConstraint(VariableOrderingH varOH,
			ValueOrderingH valOH) {
		super(varOH, valOH);
		
	}

	public void setRobotNumber(int robotNumber) {
		this.robotNumber = robotNumber;
	}
	
	@Override
	public ConstraintNetwork[] getMetaVariables() {
		Vector<ConstraintNetwork> ret = new Vector<ConstraintNetwork>();
		ViewConstraintSolver viewSolver= (ViewConstraintSolver)this.getGroundSolver();
		Variable[] vars = viewSolver.getVariables();		
		ConstraintNetwork cn = new ConstraintNetwork(null);
		int selectionCounter = 0;
		for (int i = 0; i < vars.length; i++) {
			if(vars[i] instanceof ViewVariable){
				ViewVariable vv = ((ViewVariable)vars[i]);
				//if(viewSolver.getRobotAllocationSolver().getViewConstraintByVariable(vv.getSelectionVar()) != null)
				if(viewSolver.getViewConstraintByVariable(vv) != null)
					selectionCounter++;
				else{					
					cn.addVariable(vv);					
				}
			}
		}
		if(selectionCounter < robotNumber){
			ret.add(cn);
		}
		return ret.toArray(new ConstraintNetwork[ret.size()]);
	}

	@Override
	public ConstraintNetwork[] getMetaValues(MetaVariable metaVariable) {
		ConstraintNetwork conflict = metaVariable.getConstraintNetwork();		
		Vector<ConstraintNetwork> ret = new Vector<ConstraintNetwork>();
		Variable[] vars = conflict.getVariables();
		
		Combination c = new Combination(vars.length , robotNumber);		
		HashMap<Integer, ViewVariable> indexToVar = new HashMap<Integer, ViewVariable>();
		for (int i = 0; i < vars.length; i++) {
			indexToVar.put(i, (ViewVariable)vars[i]);
		}
		while (c.hasNext()) {
			int[] a = c.next();
			ConstraintNetwork cn = new ConstraintNetwork(null);
			Vector<ViewVariable> vvs = new Vector<ViewVariable>();
			boolean infeasible = false;
			for (int i = 0; i < a.length; i++) {
				ViewConstraint vc = new ViewConstraint();
				ViewVariable curVV = indexToVar.get(a[i]);
				for (int j = 0; j < vvs.size(); j++) {
					Geometry shape1 = ((GeometricShapeDomain)curVV.getTrajectoryEnvelope().getEnvelopeVariable().getDomain()).getGeometry();
					Geometry shape2 = ((GeometricShapeDomain)vvs.get(j).getTrajectoryEnvelope().getEnvelopeVariable().getDomain()).getGeometry();
					if(shape1.intersects(shape2)){
						infeasible = true;
						break;
					}					
				}
				if(infeasible) break;
				vvs.add(curVV);				
				vc.setFrom(curVV);
				vc.setTo(curVV);
				cn.addConstraint(vc);				
			}
			if(!infeasible)
				ret.add(cn);
		}
		return ret.toArray(new ConstraintNetwork[ret.size()]);
	}

	@Override
	public void markResolvedSub(MetaVariable metaVariable,
			ConstraintNetwork metaValue) {
		// TODO Auto-generated method stub
		
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
		return this.getClass().toString();
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
