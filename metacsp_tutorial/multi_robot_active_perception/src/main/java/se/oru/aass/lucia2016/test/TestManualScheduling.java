package se.oru.aass.lucia2016.test;
import java.util.HashMap;
import java.util.Vector;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.sensing.ConstraintNetworkAnimator;
import org.metacsp.sensing.InferenceCallback;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.UI.TrajectoryEnvelopeAnimator;
import org.metacsp.utility.logging.MetaCSPLogging;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import se.oru.aass.lucia2016.execution.FlapForChaosDispatchingFunction;
import se.oru.aass.lucia2016.meta.ViewCoordinator;
import se.oru.aass.lucia2016.utility.Convertor;
import se.oru.aass.lucia2016.utility.FootPrintFactory;
import se.oru.aass.lucia2016.utility.PathPlanFactory;


public class TestManualScheduling extends AbstractNodeMain {

	private static final int TEMPORAL_RESOLUTION = 1000000;
	private static final int CONTROL_PERIOD = 1000;
	private static Logger metaCSPLogger = MetaCSPLogging.getLogger(MultiRobotCoordinator.class);
	private ConnectedNode connectedNode;
	private final String nodeName = "lucia_meta_csp_lecture";
	private TrajectoryEnvelopeSolver trajectoryEnvelopeSolver = null;
	private ActivityNetworkSolver ans = null;
	private HashMap<Integer, geometry_msgs.Pose> robotsCurrentPose = new HashMap<Integer, geometry_msgs.Pose>();
	private static final int NUMBEROFROBOTS = 3; 
	private ConstraintNetworkAnimator animator = null;
	private int cycle = 0;
	private static final long HORIZON = 10000000;
	private long ORIGIN = 0;
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
		//setupMonitoring();
		
		//Exercises: see in this method
		setupConstraintNetwork();
	}
	
	private void setupMonitoring() {
		// This class takes care of "animating" the constraint network:
		//    a. advance time at every clock tick
		//    b. dispatch actions via a FlapForChaosDispatchingFunction (there is one such function for every robot)
		//    c. model feedback of the robots into the ground constraint network (when robots finish executing actions)
		animator = new ConstraintNetworkAnimator(ans, CONTROL_PERIOD, null){
			@Override
			protected long getCurrentTimeInMillis() {				
				return getCurrentTime();
			}
		};
		
		dfs = new FlapForChaosDispatchingFunction[NUMBEROFROBOTS];
		for (int i = 1; i <= NUMBEROFROBOTS; i++) {
			dfs[i-1] = new FlapForChaosDispatchingFunction("turtlebot"+i, trajectoryEnvelopeSolver, connectedNode);
		}
		animator.addDispatchingFunctions(ans, dfs);
	}

	//Exercise method
	private void setupConstraintNetwork() {
		Pose pose1 = new Pose(4.7579498291, 0.84562087059, 0.551774602413026);
		Pose pose2 = new Pose(2.45750570297, 2.96492242813, -3.12842980123163);
		
		TrajectoryEnvelope trajectoryEnvelopeRobot1 = createTrajectoryEnvelope(pose1, 1);
		TrajectoryEnvelope trajectoryEnvelopeRobot2 = createTrajectoryEnvelope(pose2, 2);
		
		TrajectoryEnvelope parkingRobot1 = (TrajectoryEnvelope)trajectoryEnvelopeSolver.createVariable("turtlebot1");
		parkingRobot1.setFootprint(FootPrintFactory.getTurtlebotFootprint());
		Trajectory parkingPoseRobot1 = new Trajectory(new Pose[] {trajectoryEnvelopeRobot1.getTrajectory().getPose()[0]});
		parkingRobot1.setTrajectory(parkingPoseRobot1);
		parkingRobot1.setRobotID(1);
		parkingRobot1.getSymbolicVariableActivity().setSymbolicDomain("parking");

		TrajectoryEnvelope parkingRobot2 = (TrajectoryEnvelope)trajectoryEnvelopeSolver.createVariable("turtlebot2");
		parkingRobot2.setFootprint(FootPrintFactory.getTurtlebotFootprint());
		Trajectory parkingPoseRobot2 = new Trajectory(new Pose[] {trajectoryEnvelopeRobot2.getTrajectory().getPose()[0]});
		parkingRobot2.setTrajectory(parkingPoseRobot2);
		parkingRobot2.setRobotID(2);
		parkingRobot2.getSymbolicVariableActivity().setSymbolicDomain("parking");

		AllenIntervalConstraint parkingMeetsMoveRobot1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Meets);
		parkingMeetsMoveRobot1.setFrom(parkingRobot1);
		parkingMeetsMoveRobot1.setTo(trajectoryEnvelopeRobot1);
		trajectoryEnvelopeSolver.addConstraint(parkingMeetsMoveRobot1);
		
		AllenIntervalConstraint parkingMeetsMoveRobot2 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Meets);
		parkingMeetsMoveRobot2.setFrom(parkingRobot2);
		parkingMeetsMoveRobot2.setTo(trajectoryEnvelopeRobot2);
		trajectoryEnvelopeSolver.addConstraint(parkingMeetsMoveRobot2);

		AllenIntervalConstraint releaseParkingRobot1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(ORIGIN,ORIGIN));
		releaseParkingRobot1.setFrom(parkingRobot1);
		releaseParkingRobot1.setTo(parkingRobot1);
		trajectoryEnvelopeSolver.addConstraint(releaseParkingRobot1);
		
		AllenIntervalConstraint releaseParkingRobot2 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(ORIGIN,ORIGIN));
		releaseParkingRobot2.setFrom(parkingRobot2);
		releaseParkingRobot2.setTo(parkingRobot2);
		trajectoryEnvelopeSolver.addConstraint(releaseParkingRobot2);

		AllenIntervalConstraint releaseMoveRobot1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(ORIGIN+25000,APSPSolver.INF));
		releaseMoveRobot1.setFrom(trajectoryEnvelopeRobot1);
		releaseMoveRobot1.setTo(trajectoryEnvelopeRobot1);
		trajectoryEnvelopeSolver.addConstraint(releaseMoveRobot1);
		
		AllenIntervalConstraint releaseMoveRobot2 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(ORIGIN+40000,APSPSolver.INF));
		releaseMoveRobot2.setFrom(trajectoryEnvelopeRobot2);
		releaseMoveRobot2.setTo(trajectoryEnvelopeRobot2);
		trajectoryEnvelopeSolver.addConstraint(releaseMoveRobot2);

		
		//Visualize everything
		TrajectoryEnvelopeAnimator tea = new TrajectoryEnvelopeAnimator("Test");
		tea.addTrajectoryEnvelopes(parkingRobot1, parkingRobot2, trajectoryEnvelopeRobot1, trajectoryEnvelopeRobot2);

		// TODO code here
	
//		AllenIntervalConstraint overlap = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Overlaps, new Bounds(5000, APSPSolver.INF));
//		overlap.setFrom(trajectoryEnvelope1);
//		overlap.setTo(trajectoryEnvelope2);
//		trajectoryEnvelopeSolver.addConstraint(overlap);
		
		
//		//Printing all the spatial relations between two trajectory envelopes
//		for (DE9IMRelation.Type t : DE9IMRelation.getRelations(trajectoryEnvelope1.getEnvelopeVariable(), trajectoryEnvelope2.getEnvelopeVariable())) {
//			System.out.println(t);
//		}
		
//		TimelinePublisher tp = new TimelinePublisher(ans.getConstraintNetwork(), new Bounds(0,60000), true, "Time", "turtlebot1", "turtlebot2", "turtlebot3");
//		TimelineVisualizer tv = new TimelineVisualizer(tp);
//		tv.startAutomaticUpdate(CONTROL_PERIOD);

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
		trajectoryEnvelopeSolver = new TrajectoryEnvelopeSolver(ORIGIN, ORIGIN+HORIZON, 100);
		ans = (ActivityNetworkSolver)trajectoryEnvelopeSolver.getConstraintSolversFromConstraintSolverHierarchy(ActivityNetworkSolver.class)[0];
		MetaCSPLogging.setLevel(ViewCoordinator.class, Level.FINEST);
	}



	// create TrajectoryEnvelope given a goal
	private TrajectoryEnvelope createTrajectoryEnvelope(Pose goalPose, int rid) {
		
		TrajectoryEnvelope trajEnvelope = (TrajectoryEnvelope)trajectoryEnvelopeSolver.createVariable("turtlebot" + rid);
		trajEnvelope.getSymbolicVariableActivity().setSymbolicDomain("move_base");
		
		//get path from ROS
		Trajectory traj = new Trajectory(getPathFromROS(goalPose, rid));
		
		trajEnvelope.setFootprint(FootPrintFactory.getTurtlebotFootprint());
		trajEnvelope.setTrajectory(traj);
		trajEnvelope.setRobotID(rid);

		return trajEnvelope;
	}
	
	private Pose[] getPathFromROS(Pose goalPose, int rid) {
		
		HashMap<Integer, Vector<Pose>> robToPathPoses = new HashMap<Integer, Vector<Pose>>();
		HashMap<Integer, Boolean> robToPathStatus = new HashMap<Integer, Boolean>();
		
		robToPathPoses.put(rid, new Vector<Pose>());
		robToPathStatus.put(rid, false);
		
		PathPlanFactory.getRobotPathPlanFromROSSerive(robToPathStatus, robToPathPoses, connectedNode, rid, 
				Convertor.getPoseStamped(robotsCurrentPose.get(rid), connectedNode), 
				Convertor.getPoseStamped(goalPose, connectedNode));
		
		while (!robToPathStatus.get(rid)) {
			try { Thread.sleep(10); }
			catch (InterruptedException e) { e.printStackTrace();}
		}
		
		return robToPathPoses.get(rid).toArray(new Pose[robToPathPoses.get(rid).size()]);
		
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


