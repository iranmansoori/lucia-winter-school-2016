package se.oru.aass.lucia2016.test;
import java.util.Vector;
import java.util.logging.Level;

import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.utility.logging.MetaCSPLogging;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.apache.commons.logging.Log;

import se.oru.aass.lucia2016.meta.ViewCoordinator;
import se.oru.aass.lucia2016.multi.ViewConstraintSolver;
import se.oru.aass.lucia2016.multi.ViewVariable;





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
		
		Vector<Pose> cameraPoses = new Vector<Pose>();
		Vector<Double> infoGains = new Vector<Double>();
		setCameraPoses(cameraPoses, infoGains);
		createViewVariables(cameraPoses, infoGains);
		
		
	}

	private void createViewVariables(Vector<Pose> cameraPoses,
			Vector<Double> infoGains) {

		
	}

	private void setCameraPoses(Vector<Pose> cameraPoses, Vector<Double> infoGains) {
		cameraPoses.add(new Pose(-12.7615280151367, -6.15457534790039, -3.12842980123163));
		infoGains.add(0.8);
		cameraPoses.add(new Pose(-14.5186862945557, -2.39040231704712, -1.51838576694655));
		infoGains.add(0.8);
		cameraPoses.add(new Pose(-19.23854637146, -6.94821929931641, 0.551774602413026));
		infoGains.add(0.8);
	}


}
