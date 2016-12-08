package se.oru.aass.lucia2016.multi;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.Variable;

/**
 * This solver is a ground solver underlying the {@link ViewConstraintSolver}. It manages variables of type
 * {@link SelectionVariable} and understands {@link ViewSelectionConstraint}s, though it does not
 * perform inference on {@link ViewSelectionConstraint}s. 
 * 
 * @author iran
 *
 */
public class ViewSelectionSolver extends ConstraintSolver{

	private static final long serialVersionUID = 669559549933539227L;

	protected ViewSelectionSolver() {
		super(new Class[]{ViewSelectionConstraint.class}, SelectionVariable.class);
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
		SelectionVariable[] ret = new SelectionVariable[num];
		for (int i = 0; i < num; i++) ret[i] = new SelectionVariable(this, IDs++);
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
