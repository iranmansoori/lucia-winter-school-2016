package se.oru.aass.lucia2016.multi;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.multi.MultiConstraintSolver;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatial.DE9IM.DE9IMRelation;
import org.metacsp.multi.spatial.DE9IM.DE9IMRelationSolver;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.spatial.geometry.Polygon;

import se.oru.aass.lucia2016.meta.ViewCoordinator;

/**
 * Ground solver for the {@link ViewCoordinator}. It creates variables of type {@link ViewVariable} (which, in turn, are aggregations of {@link Polygon}s and {@link TrajectoryEnvelope}s).
 * It understands {@link AllenIntervalConstraint}s and {@link DE9IMRelation}s.
 *  
 * @author iran
 *
 */
public class ViewConstraintSolver extends MultiConstraintSolver {

	private static final long serialVersionUID = 3569061369909154762L;

	protected ViewConstraintSolver(Class<?>[] constraintTypes, Class<?> variableType,
			ConstraintSolver[] internalSolvers, int[] ingredients) {
		super(constraintTypes, variableType, internalSolvers, ingredients);
	}

	public ViewConstraintSolver(long origin, long horizon, int maxTrajectories) {
		super(new Class[]{AllenIntervalConstraint.class, DE9IMRelation.class, ViewSelectionConstraint.class, RobotAllocationConstraint.class}, ViewVariable.class, createInternalConstraintSolvers(origin, horizon, maxTrajectories), new int[]{1,1,1,1});
	}
	
	private static ConstraintSolver[] createInternalConstraintSolvers(long origin, long horizon, int maxTrajectories) {
		ConstraintSolver[] ret = new ConstraintSolver[4];
		if (maxTrajectories >= 1) {
			ret[0] = new TrajectoryEnvelopeSolver(origin, horizon, maxTrajectories);
		}
		else {
			ret[0] = new TrajectoryEnvelopeSolver(origin, horizon);
		}
		ret[1] = ((TrajectoryEnvelopeSolver)ret[0]).getSpatialSolver();
		ret[2] = new ViewSelectionSolver();
		ret[3] = new RobotAllocationSolver();
		return ret;
	}
	
	public TrajectoryEnvelopeSolver getTrajectoryEnvelopeSolver() {
		return (TrajectoryEnvelopeSolver)this.getConstraintSolvers()[0];
	}
	
	public DE9IMRelationSolver getSpatialSolver() {
		return (DE9IMRelationSolver)this.getConstraintSolvers()[1];
	}
	
	public RobotAllocationSolver getRobotAllocationSolver() {
		return (RobotAllocationSolver)this.getConstraintSolvers()[2];
	}
	
	public ViewSelectionConstraint getViewConstraintByVariable(ViewVariable vv){
		Constraint[] cn = this.getConstraints();
		for (int i = 0; i < cn.length; i++) {
			if(cn[i] instanceof ViewSelectionConstraint){
				ViewSelectionConstraint vc = (ViewSelectionConstraint)cn[i];
				if(vc.getFrom().equals(vv))
					return vc; 
			}
		}
		return null;
	}

	
	
	public RobotAllocationConstraint getRobotConstraintByVariable(ViewVariable sv) {
		Constraint[] cn = this.getConstraints();
		for (int i = 0; i < cn.length; i++) {
			if(cn[i] instanceof RobotAllocationConstraint){
				RobotAllocationConstraint rc = (RobotAllocationConstraint)cn[i];
				if(rc.getFrom().equals(sv))
					return rc; 
			}
		}
		return null;
	}
	
	@Override
	public boolean propagate() {
		// TODO Auto-generated method stub
		return true;
	}
	
}
