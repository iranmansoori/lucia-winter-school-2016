package se.oru.aass.lucia2016.meta;

import java.util.List;
import java.util.Vector;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.ValueOrderingH;
import org.metacsp.framework.Variable;
import org.metacsp.framework.VariableOrderingH;
import org.metacsp.framework.meta.MetaConstraint;
import org.metacsp.framework.meta.MetaVariable;
import org.metacsp.utility.Permutation;

import se.oru.aass.lucia2016.multi.RobotAllocationConstraint;
import se.oru.aass.lucia2016.multi.ViewConstraintSolver;
import se.oru.aass.lucia2016.multi.ViewVariable;

/**
 * This {@link MetaConstraint} allocates robots to view poses.
 * @author iran
 *
 */
public class RobotAllocationMetaConstraint extends MetaConstraint {

	
	private static final long serialVersionUID = -4876976005187040794L;

	private List<Integer> robotIDs = null;
	
	public void setRobotIDs(List<Integer> robotIDs) {
		this.robotIDs = robotIDs;
	}
	
	public RobotAllocationMetaConstraint(VariableOrderingH varOH,
			ValueOrderingH valOH) {
		super(varOH, valOH);
	}

	@Override
	public ConstraintNetwork[] getMetaVariables() {
		Vector<ConstraintNetwork> ret = new Vector<ConstraintNetwork>();
		ViewConstraintSolver viewSolver= (ViewConstraintSolver)this.getGroundSolver();
		Variable[] vars = viewSolver.getVariables();		
		ConstraintNetwork cn = new ConstraintNetwork(null);
		
		for (int i = 0; i < vars.length; i++) {
			if(vars[i] instanceof ViewVariable){
				ViewVariable vv = ((ViewVariable)vars[i]);
				if(viewSolver.getViewConstraintByVariable(vv) != null
						&& viewSolver.getRobotConstraintByVariable(vv) == null)
					cn.addVariable(vv);					
			}
		}
		if(cn.getVariables().length != 0)
			ret.add(cn);
		return ret.toArray(new ConstraintNetwork[ret.size()]);
	}

	@Override
	public ConstraintNetwork[] getMetaValues(MetaVariable metaVariable) {
		ConstraintNetwork conflict = metaVariable.getConstraintNetwork();		
		Vector<ConstraintNetwork> ret = new Vector<ConstraintNetwork>();
		Variable[] vars = conflict.getVariables();
		
		Permutation c = new Permutation(vars.length , vars.length);		
		while (c.hasNext()) {
			int[] a = c.next();
			ConstraintNetwork cn = new ConstraintNetwork(null);
			for (int i = 0; i < a.length; i++) {
				RobotAllocationConstraint rc = new RobotAllocationConstraint(robotIDs.get(a[i]));
				rc.setFrom(((ViewVariable)vars[i]));
				rc.setTo(((ViewVariable)vars[i]));
				cn.addConstraint(rc);				
			}
			ret.add(cn);
			//break;
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
