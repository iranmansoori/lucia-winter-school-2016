package se.oru.aass.lucia2016.test;
import java.util.HashMap;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatial.DE9IM.DE9IMRelation;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.sensing.ConstraintNetworkAnimator;
import org.metacsp.sensing.InferenceCallback;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.logging.MetaCSPLogging;
import org.metacsp.utility.timelinePlotting.TimelinePublisher;
import org.metacsp.utility.timelinePlotting.TimelineVisualizer;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import se.oru.aass.lucia2016.execution.FlapForChaosDispatchingFunction;
import se.oru.aass.lucia2016.meta.RobotAllocationMetaConstraint;
import se.oru.aass.lucia2016.meta.RobotAllocationValOH;
import se.oru.aass.lucia2016.meta.ViewCoordinator;
import se.oru.aass.lucia2016.meta.ViewSelectionMetaConstraint;
import se.oru.aass.lucia2016.multi.ViewConstraintSolver;
import se.oru.aass.lucia2016.multi.ViewVariable;
import se.oru.aass.lucia2016.utility.FootPrintFactory;
//import org.metacsp.multi.spatioTemporal.paths.Pose;


public class TestManualScheduling extends AbstractNodeMain {

	private static final int TEMPORAL_RESOLUTION = 1000000;
	private static final int CONTROL_PERIOD = 1000;
	private static Logger metaCSPLogger = MetaCSPLogging.getLogger(MultiRobotCoordinator.class);
	private ConnectedNode connectedNode;
	private final String nodeName = "lucia_meta_csp_lecture";
	private ViewCoordinator metaSolver = null;
	private ViewConstraintSolver viewSolver = null;
	private TrajectoryEnvelopeSolver trajectoryEnvelopeSolver = null;
	private ActivityNetworkSolver ans = null;
	private HashMap<Integer, geometry_msgs.Pose> robotsCurrentPose = new HashMap<Integer, geometry_msgs.Pose>();
	private static final int NUMBEROFROBOTS = 3; 
	private ConstraintNetworkAnimator animator = null;
	private int cycle = 0;
	private static final long HORIZON = 10000000;
	private long ORIGIN = 0;
	private long SENSING_DURATION = 5000;
	private FlapForChaosDispatchingFunction[] dfs = null;


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
		//subscribe to robot pose topic
		for (int i = 1; i <= NUMBEROFROBOTS; i++) {
			subscribeToRobotReportTopic(i);
		}
		setupSolvers();
		
		Pose pose1 = new Pose(1.05, 4.31, -3.12842980123163);
		Pose pose2 = new Pose(6.09, 0.82, 0.551774602413026);
		
		TrajectoryEnvelope trajectoryEnvelope1 = createTrajectoryEnvelope(pose1);
		TrajectoryEnvelope trajectoryEnvelope2 = createTrajectoryEnvelope(pose2);

		// TODO code here
	
		AllenIntervalConstraint overlap = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Overlaps, new Bounds(5000, APSPSolver.INF));
		overlap.setFrom(trajectoryEnvelope1);
		overlap.setTo(trajectoryEnvelope2);
		trajectoryEnvelopeSolver.addConstraint(overlap);
		
		
		//Printing all the spatial relations between two trajectory envelopes
		for (DE9IMRelation.Type t : DE9IMRelation.getRelations(trajectoryEnvelope1.getEnvelopeVariable(), trajectoryEnvelope2.getEnvelopeVariable())) {
			System.out.println(t);
		}
		
		
		// The callback that is called at each cycle
		InferenceCallback cb = new InferenceCallback() {	

			private int myCycle = cycle++;
			@Override
			public void doInference(long timeNow) {
				metaSolver.backtrack(); 
				if (metaSolver.getAddedResolvers().length > 0) {
					metaCSPLogger.info("== END SOLUTION ==");
					//show a timeline
					TimelinePublisher tp = new TimelinePublisher(ans.getConstraintNetwork(), new Bounds(0,60000), true, "Time", "turtlebot1", "turtlebot2", "turtlebot3");
					TimelineVisualizer tv = new TimelineVisualizer(tp);
					tv.startAutomaticUpdate(CONTROL_PERIOD);
					//tp.publish(false, false);

					//Save a copy of the constraint network for inspection/debugging
					ConstraintNetwork.saveConstraintNetwork(viewSolver.getTrajectoryEnvelopeSolver().getConstraintNetwork(), "testingLucia" + myCycle + ".cn");
					metaSolver.clearResolvers();									
				}
			}

		};

		// This class takes care of "animating" the constraint network:
		//    a. advance time at every clock tick
		//    b. dispatch actions via a FlapForChaosDispatchingFunction (there is one such function for every robot)
		//    c. model feedback of the robots into the ground constraint network (when robots finish executing actions)
		animator = new ConstraintNetworkAnimator(ans, CONTROL_PERIOD, cb){
			@Override
			protected long getCurrentTimeInMillis() {				
				return getCurrentTime();
			}
		};		
		dfs = new FlapForChaosDispatchingFunction[NUMBEROFROBOTS];
		for (int i = 1; i <= NUMBEROFROBOTS; i++) {
			dfs[i-1] = new FlapForChaosDispatchingFunction("turtlebot"+i, metaSolver, connectedNode);
		}
		animator.addDispatchingFunctions(ans, dfs);		

	}
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(nodeName);
	}

	public long getCurrentTime() {
		return connectedNode.getCurrentTime().totalNsecs()/TEMPORAL_RESOLUTION;
	}

	// Instantiate and set up all solvers
	private void setupSolvers() {
		ORIGIN = getCurrentTime();

		//Instantiate meta-solver, and get references for underlying ground solvers
		metaSolver = new ViewCoordinator(ORIGIN, ORIGIN+HORIZON, 100);
		viewSolver = (ViewConstraintSolver)metaSolver.getConstraintSolvers()[0];
		trajectoryEnvelopeSolver = ((TrajectoryEnvelopeSolver)viewSolver.getConstraintSolvers()[0]);
		ans = (ActivityNetworkSolver)((TrajectoryEnvelopeSolver)viewSolver.getConstraintSolvers()[0]).getConstraintSolvers()[0];
		MetaCSPLogging.setLevel(ViewCoordinator.class, Level.FINEST);
		metaSolver.setROSNode(connectedNode);
		metaSolver.setRobotCurrentPose(robotsCurrentPose);
		metaSolver.setTimeNow(getCurrentTime());


		//Define and add the meta-constraints:
		//  1. ViewSelectionMetaConstraint: selects view poses to navigate to 
		ViewSelectionMetaConstraint viewSelectionMC = new ViewSelectionMetaConstraint(null, null);	
		viewSelectionMC.setNumberOfRobots(NUMBEROFROBOTS);
		metaSolver.addMetaConstraint(viewSelectionMC);

		//  2. RobotAllocationMetaConstraint: takes care of assigning robots to selected view poses
		RobotAllocationValOH robotSelectionValOH = new RobotAllocationValOH(metaSolver);
		RobotAllocationMetaConstraint robotAllocationMC = new RobotAllocationMetaConstraint(null, null);
		metaSolver.addMetaConstraint(robotAllocationMC);

	}



	// create TrajectoryEnvelope given a goal
	private TrajectoryEnvelope createTrajectoryEnvelope(Pose goalPose) {
		ViewVariable vv1 = (ViewVariable)viewSolver.createVariable();
		vv1.getTrajectoryEnvelope().getSymbolicVariableActivity().setSymbolicDomain("Sense");
		Trajectory trajRobot1 = new Trajectory(new Pose[] {goalPose});
		vv1.getTrajectoryEnvelope().setFootprint(FootPrintFactory.getTurtlebotFootprint());
		vv1.getTrajectoryEnvelope().setTrajectory(trajRobot1);
		vv1.setFOVCoordinates(FootPrintFactory.getFoVCoordinates());
		AllenIntervalConstraint duration1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(SENSING_DURATION ,APSPSolver.INF));
		duration1.setFrom(vv1);
		duration1.setTo(vv1);
		viewSolver.addConstraint(duration1);
		return vv1.getTrajectoryEnvelope();		
	}

	private void subscribeToRobotReportTopic(final int i) {
		Subscriber<geometry_msgs.PoseWithCovarianceStamped> poseFeedback = connectedNode.newSubscriber("/turtlebot" + i + "/amcl_pose", geometry_msgs.PoseWithCovarianceStamped._TYPE);
		poseFeedback.addMessageListener(new MessageListener<geometry_msgs.PoseWithCovarianceStamped>() {
			@Override
			public void onNewMessage(geometry_msgs.PoseWithCovarianceStamped message) {
				robotsCurrentPose.put(i, message.getPose().getPose());
			}
		}, 10);	
	}




}


