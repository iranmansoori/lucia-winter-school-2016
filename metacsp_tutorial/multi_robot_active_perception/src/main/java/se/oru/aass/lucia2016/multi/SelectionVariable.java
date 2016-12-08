package se.oru.aass.lucia2016.multi;

import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.Domain;
import org.metacsp.framework.Variable;

/**
 * This variable represents whether a particular view pose has been selected for mapping. It is used as
 * a ground variable underlying a {@link ViewVariable}.
 * 
 * @author iran
 *
 */
public class SelectionVariable extends Variable {
	
	private Domain dom;
	
	protected SelectionVariable(ConstraintSolver cs, int id) {
		super(cs, id);
		// TODO Auto-generated constructor stub
	}

	private static final long serialVersionUID = -9173221054510086262L;

	@Override
	public int compareTo(Variable arg0) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public Domain getDomain() {
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
