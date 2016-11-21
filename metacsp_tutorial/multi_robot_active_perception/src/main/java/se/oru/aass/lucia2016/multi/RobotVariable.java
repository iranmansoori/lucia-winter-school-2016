package se.oru.aass.lucia2016.multi;

import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.Domain;
import org.metacsp.framework.Variable;

public class RobotVariable extends Variable {
	
	/**
	 * 
	 */
	private static final long serialVersionUID = 85453100979954279L;
	private Domain dom;	
	protected RobotVariable(ConstraintSolver cs, int id) {
		super(cs, id);

	}

	@Override
	public int compareTo(Variable arg0) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public Domain getDomain() {
		// TODO Auto-generated method stub
		return dom;
	}

	@Override
	public void setDomain(Domain d) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public String toString() {
		return "ID" + this.id;
	}

}
