package se.oru.aass.lucia2016.multi;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.Variable;

import se.oru.aass.lucia2016.meta.RobotAllocationMetaConstraint;


/**
 * This is one of the {@link ConstraintSolver}s underlying the {@link ViewConstraintSolver}.
 * It is currently not very useful, because no inference is done on {@link RobotAllocationConstraint}s,
 * rather they are directly "guessed" by the {@link RobotAllocationMetaConstraint}. This solver could be
 * used to propagate {@link RobotAllocationConstraint}s.
 *  
 * @author iran
 *
 */
public class RobotAllocationSolver extends ConstraintSolver {
	
	private static final long serialVersionUID = -3080579700239701010L;

	public RobotAllocationSolver() {
		super(new Class[]{RobotAllocationConstraint.class}, RobotVariable.class);
		this.setOptions(OPTIONS.AUTO_PROPAGATE);
		this.setOptions(OPTIONS.DOMAINS_AUTO_INSTANTIATED);
	}

	@Override
	public boolean propagate() {
		return true;
	}

	@Override
	protected boolean addConstraintsSub(Constraint[] c) {
		return true;
	}

	@Override
	protected void removeConstraintsSub(Constraint[] c) {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected Variable[] createVariablesSub(int num) {
		RobotVariable[] ret = new RobotVariable[num];
		for (int i = 0; i < num; i++) ret[i] = new RobotVariable(this, IDs++);
		return ret;

	}

	@Override
	protected void removeVariablesSub(Variable[] v) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void registerValueChoiceFunctions() {
		// TODO Auto-generated method stub
		
	}


}
