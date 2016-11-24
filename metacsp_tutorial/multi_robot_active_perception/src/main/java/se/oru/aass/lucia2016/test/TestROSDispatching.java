package se.oru.aass.lucia2016.test;
import java.util.Calendar;
import java.util.Random;
import java.util.Vector;
import java.util.logging.Level;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ValueOrderingH;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatial.DE9IM.PolygonalDomain;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.sensing.ConstraintNetworkAnimator;
import org.metacsp.sensing.InferenceCallback;
import org.metacsp.time.Bounds;
import org.metacsp.utility.UI.TrajectoryEnvelopeAnimator;
import org.metacsp.utility.logging.MetaCSPLogging;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.apache.commons.logging.Log;

import com.vividsolutions.jts.geom.Geometry;

import se.oru.aass.lucia2016.execution.FlapForChaosDispatchingFunction;
import se.oru.aass.lucia2016.meta.RobotAllocationMetaConstraint;
import se.oru.aass.lucia2016.meta.ViewCoordinator;
import se.oru.aass.lucia2016.meta.ViewSchedulingMetaConstraint;
import se.oru.aass.lucia2016.meta.ViewSelectionMetaConstraint;
import se.oru.aass.lucia2016.meta.ViewSelectionValOH;
import se.oru.aass.lucia2016.multi.ViewConstraintSolver;
import se.oru.aass.lucia2016.multi.ViewVariable;
import se.oru.aass.lucia2016.utility.FootPrintFactory;


public class TestROSDispatching extends AbstractNodeMain {

	private ConnectedNode connectedNode;
	private final String nodeName = "lucia_meta_csp_lecture";
	private ViewCoordinator metaSolver = null;
	private ViewConstraintSolver viewSolver = null;
	private ActivityNetworkSolver ans = null;
	
	private int ROBOTNUMBER = 3; 
	
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
		
		
		metaSolver = new ViewCoordinator(0, 10000000, 100);
		viewSolver = (ViewConstraintSolver)metaSolver.getConstraintSolvers()[0];
		ans = (ActivityNetworkSolver)((TrajectoryEnvelopeSolver)viewSolver.getConstraintSolvers()[0]).getConstraintSolvers()[0];
		MetaCSPLogging.setLevel(ViewCoordinator.class, Level.FINEST);
		
		Vector<Pose> cameraPoses = new Vector<Pose>();
		Vector<Double> infoGains = new Vector<Double>();
		setCameraPoses(cameraPoses, infoGains);
		final ViewVariable[] vvs = createViewVariables(cameraPoses, infoGains);
		
		
		//adding the meta-constraints
		ViewSelectionMetaConstraint viewSelectionMC = new ViewSelectionMetaConstraint(null, new ViewSelectionValOH());
		viewSelectionMC.setRobotNumber(3);
		metaSolver.addMetaConstraint(viewSelectionMC);
		
		RobotAllocationMetaConstraint RobotAllocationMC = new RobotAllocationMetaConstraint(null, null);
		metaSolver.addMetaConstraint(RobotAllocationMC);
		
		ViewSchedulingMetaConstraint viewSchedulingMC = new ViewSchedulingMetaConstraint(null, null);
		metaSolver.addMetaConstraint(viewSchedulingMC);
		viewSchedulingMC.setUsage(vvs);

		final Random rand = new Random(Calendar.getInstance().getTimeInMillis());
		viewSchedulingMC.setValOH(new ValueOrderingH() {
			@Override
			public int compare(ConstraintNetwork arg0, ConstraintNetwork arg1) {
				return (rand.nextInt(3)-1);
			}
		});
		
		final TrajectoryEnvelopeAnimator tea = new TrajectoryEnvelopeAnimator("Solution");		
		InferenceCallback cb = new InferenceCallback() {
			
			@Override
			public void doInference(long timeNow) {
				metaSolver.backtrack(); 			
				if (metaSolver.getAddedResolvers().length > 0) {
					tea.setTrajectoryEnvelopes(viewSolver.getTrajectoryEnvelopeSolver().getConstraintNetwork());
					//TODO: this has to be changed to those only is chosen
					Geometry[] gms = new Geometry[vvs.length]; 
					for (int i = 0; i < vvs.length; i++) {
						gms[i] = ((PolygonalDomain)vvs[i].getFoV().getDomain()).getGeometry();
					}
					tea.addExtraGeometries(gms);
					metaSolver.clearResolvers();
				}
			}
		};
		
		ConstraintNetworkAnimator animator = new ConstraintNetworkAnimator(ans, 1000, cb){
			@Override
			protected long getCurrentTimeInMillis() {
				return connectedNode.getCurrentTime().totalNsecs()/1000000;
			}
		};		
		FlapForChaosDispatchingFunction[] dfs = new FlapForChaosDispatchingFunction[ROBOTNUMBER];
		for (int i = 1; i <= ROBOTNUMBER; i++) {
			dfs[i] = new FlapForChaosDispatchingFunction("Robot"+i);
		}
		animator.addDispatchingFunctions(ans, dfs);
		tea.setConstraintNetworkAnimator(animator);
		
	}

	private ViewVariable[] createViewVariables(Vector<Pose> cameraPoses, Vector<Double> infoGains) {
		ViewVariable[] ret = new ViewVariable[cameraPoses.size()];
		Variable[] vars = viewSolver.createVariables(cameraPoses.size());
		for (int i = 0; i < vars.length; i++) {
			ViewVariable vv1 = (ViewVariable)vars[i];
			ret[i] = vv1;
			vv1.getTrajectoryEnvelope().getSymbolicVariableActivity().setSymbolicDomain("Sense");
			Trajectory trajRobot1 = new Trajectory(new Pose[] {cameraPoses.get(i)});
			vv1.getTrajectoryEnvelope().setFootprint(FootPrintFactory.getTurtlebotFootprint());
			vv1.getTrajectoryEnvelope().setTrajectory(trajRobot1);
			vv1.setFOVCoordinates(FootPrintFactory.getFoVCoordinates());
			vv1.setInfoGain(infoGains.get(i));
			AllenIntervalConstraint duration1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(10000,15000));
			duration1.setFrom(vv1);
			duration1.setTo(vv1);
			viewSolver.addConstraint(duration1);
		}				
		return ret;
		
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
