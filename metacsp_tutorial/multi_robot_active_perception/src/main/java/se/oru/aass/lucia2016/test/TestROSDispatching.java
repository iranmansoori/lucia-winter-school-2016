package se.oru.aass.lucia2016.test;
import java.util.logging.Level;

import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.utility.logging.MetaCSPLogging;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.apache.commons.logging.Log;

import se.oru.aass.lucia2016.meta.ViewCoordinator;
import se.oru.aass.lucia2016.multi.ViewConstraintSolver;





public class TestROSDispatching extends AbstractNodeMain {

	private ConnectedNode connectedNode;
	private final String nodeName = "lucia_meta_csp_lecture";
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(nodeName);
	}
	
	@Override
	public void onStart(ConnectedNode cn) {
		this.connectedNode = cn;
		
		while (true) {
			try {
				this.connectedNode.getCurrentTime();
				break;
			}
			catch(NullPointerException e) { }
		}
		
		
		final ViewCoordinator metaSolver = new ViewCoordinator(0, 10000000, 100);
		final ViewConstraintSolver solver = (ViewConstraintSolver)metaSolver.getConstraintSolvers()[0];
		ActivityNetworkSolver ans = (ActivityNetworkSolver)((TrajectoryEnvelopeSolver)solver.getConstraintSolvers()[0]).getConstraintSolvers()[0];
		MetaCSPLogging.setLevel(ViewCoordinator.class, Level.FINEST);
		
		
		
	}


}
