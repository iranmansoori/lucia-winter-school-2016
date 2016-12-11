package se.oru.aass.lucia2016.exercises;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Vector;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.allenInterval.AllenInterval;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatial.DE9IM.DE9IMRelation;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeVariable;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
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

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.MultiPolygon;
import com.vividsolutions.jts.geom.Point;

import se.oru.aass.lucia2016.execution.FlapForChaosDispatchingFunction;
import se.oru.aass.lucia2016.execution.MultiRobotCoordinator;
import se.oru.aass.lucia2016.meta.ViewCoordinator;
import se.oru.aass.lucia2016.multi.ViewConstraintSolver;
import se.oru.aass.lucia2016.utility.Convertor;
import se.oru.aass.lucia2016.utility.FootPrintFactory;
import se.oru.aass.lucia2016.utility.PathPlanFactory;


public class Ex4 extends AbstractNodeMain {

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
	private static final long HORIZON = 10000000;
	private long ORIGIN = 0;
	private FlapForChaosDispatchingFunction[] dfs = null;
	
	//TODO 2: set to true to see your robots moving in Gazebo!
	private boolean dispatching = true;

	//Exercise method
	private void setupConstraintNetwork() {
		
		//Hard coded poses in the Lucia map
		Pose pose1 = new Pose(4.7579498291, 0.84562087059, 0.551774602413026);
		Pose pose2 = new Pose(2.45750570297, 2.96492242813, -3.12842980123163);
		
		//Get paths from ROS so that turtlebot1 and turtlebot2
		//can reach these poses, and create TrajectoryEnvelopes around them
		TrajectoryEnvelope trajectoryEnvelopeRobot1 = createTrajectoryEnvelope(pose1, 1);
		TrajectoryEnvelope trajectoryEnvelopeRobot2 = createTrajectoryEnvelope(pose2, 2);
		
		//Create a parking polygon for turtlebot1
		TrajectoryEnvelope parkingRobot1 = (TrajectoryEnvelope)trajectoryEnvelopeSolver.createVariable("turtlebot1");
		parkingRobot1.setFootprint(FootPrintFactory.getTurtlebotFootprint());
		Trajectory parkingPoseRobot1 = new Trajectory(new Pose[] {trajectoryEnvelopeRobot1.getTrajectory().getPose()[0]});
		parkingRobot1.setTrajectory(parkingPoseRobot1);
		parkingRobot1.setRobotID(1);
		parkingRobot1.getSymbolicVariableActivity().setSymbolicDomain("Parking");

		//Create a parking polygon for turtlebot2
		TrajectoryEnvelope parkingRobot2 = (TrajectoryEnvelope)trajectoryEnvelopeSolver.createVariable("turtlebot2");
		parkingRobot2.setFootprint(FootPrintFactory.getTurtlebotFootprint());
		Trajectory parkingPoseRobot2 = new Trajectory(new Pose[] {trajectoryEnvelopeRobot2.getTrajectory().getPose()[0]});
		parkingRobot2.setTrajectory(parkingPoseRobot2);
		parkingRobot2.setRobotID(2);
		parkingRobot2.getSymbolicVariableActivity().setSymbolicDomain("Parking");

		//turtlebot1 is parked from the origin of time
		AllenIntervalConstraint releaseParkingRobot1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(ORIGIN,ORIGIN));
		releaseParkingRobot1.setFrom(parkingRobot1);
		releaseParkingRobot1.setTo(parkingRobot1);
		trajectoryEnvelopeSolver.addConstraint(releaseParkingRobot1);

		//turtlebot2 is parked from the origin of time
		AllenIntervalConstraint releaseParkingRobot2 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(ORIGIN,ORIGIN));
		releaseParkingRobot2.setFrom(parkingRobot2);
		releaseParkingRobot2.setTo(parkingRobot2);
		trajectoryEnvelopeSolver.addConstraint(releaseParkingRobot2);

		//turtlebot1 should start moving immediately after parking (meets)
		AllenIntervalConstraint parkingMeetsMoveRobot1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Meets);
		parkingMeetsMoveRobot1.setFrom(parkingRobot1);
		parkingMeetsMoveRobot1.setTo(trajectoryEnvelopeRobot1);
		trajectoryEnvelopeSolver.addConstraint(parkingMeetsMoveRobot1);

		//turtlebot2 should start moving immediately after parking (meets)
		AllenIntervalConstraint parkingMeetsMoveRobot2 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Meets);
		parkingMeetsMoveRobot2.setFrom(parkingRobot2);
		parkingMeetsMoveRobot2.setTo(trajectoryEnvelopeRobot2);
		trajectoryEnvelopeSolver.addConstraint(parkingMeetsMoveRobot2);

		//turtlebot1 stays parked for at least 10 second
		AllenIntervalConstraint releaseMoveRobot1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(ORIGIN+5000,APSPSolver.INF));
		releaseMoveRobot1.setFrom(trajectoryEnvelopeRobot1);
		releaseMoveRobot1.setTo(trajectoryEnvelopeRobot1);
		trajectoryEnvelopeSolver.addConstraint(releaseMoveRobot1);

		//turtlebot2 stays parked for at least 10 second
		AllenIntervalConstraint releaseMoveRobot2 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(ORIGIN+5000,APSPSolver.INF));
		releaseMoveRobot2.setFrom(trajectoryEnvelopeRobot2);
		releaseMoveRobot2.setTo(trajectoryEnvelopeRobot2);
		trajectoryEnvelopeSolver.addConstraint(releaseMoveRobot2);
		
		//TODO 3: If we had a more sophisticated controller, we could refine our reasoning
		//        -- set dispatching to false again,
		//        -- uncomment below and see the GUI
//		refineTrajectoryEnvelopes(trajectoryEnvelopeRobot1, trajectoryEnvelopeRobot2);
//		refineTrajectoryEnvelopes(trajectoryEnvelopeRobot2, trajectoryEnvelopeRobot1);
		
		//TODO 1: Check whether envelopes overlap spatially and temporally, and if so, separate them temporally
		boolean spatiallyOverlapping = false;
		boolean temporallyOverlapping = false;
		
		//insert code here
		
		//Make turtlebot2 goes before turtlebot1
		if (temporallyOverlapping && spatiallyOverlapping) {
			//insert code here
		}
		
		//Visualize everything
		TrajectoryEnvelopeAnimator tea = new TrajectoryEnvelopeAnimator("Test");
		tea.addTrajectoryEnvelopes(parkingRobot1, parkingRobot2, trajectoryEnvelopeRobot1, trajectoryEnvelopeRobot2);

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
		trajEnvelope.getSymbolicVariableActivity().setSymbolicDomain("MoveBase");
		
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

	private ConstraintNetwork refineTrajectoryEnvelopes(TrajectoryEnvelope var1, TrajectoryEnvelope var2) {
		
		int MINIMUM_SIZE = 5;
		
		ConstraintNetwork toReturn = new ConstraintNetwork(null);
		GeometryFactory gf = new GeometryFactory();
		Geometry se1 = ((GeometricShapeDomain)var1.getEnvelopeVariable().getDomain()).getGeometry();
		Geometry se2 = ((GeometricShapeDomain)var2.getEnvelopeVariable().getDomain()).getGeometry();
		Geometry intersectionse1se2 = se1.intersection(se2);

		boolean useDefaultEnvelopeChunks = false;

		if (!intersectionse1se2.isValid()) {
			intersectionse1se2 = intersectionse1se2.symDifference(intersectionse1se2.getBoundary());
			metaCSPLogger.info("Intersection " + var1 + " with " + var2 + " invalid - fixing");
		}

		if (intersectionse1se2 instanceof MultiPolygon) {
			metaCSPLogger.info("Intersection " + var1 + " with " + var2 + " too complex - skipping");
			useDefaultEnvelopeChunks = true;
			//return toReturn;								
		}

		boolean in  = false;
		int countIn = 0;
		for (int i = 0; i < var1.getPathLength(); i++) {
			Coordinate coord = var1.getTrajectory().getPositions()[i];
			Point point = gf.createPoint(coord);
			if (intersectionse1se2.contains(point) && !in) {
				in = true;
				if (++countIn > 1) {
					metaCSPLogger.info("Reference path of " + var1 + " enters intersection with " + var2 + " multiple times - skipping");
					useDefaultEnvelopeChunks = true;
					break;
					//return toReturn;					
				}
			}
			if (!intersectionse1se2.contains(point)) {
				in = false;
			}
		}

		double areaDifference = intersectionse1se2.symDifference(intersectionse1se2.getBoundary()).union(se1).getArea()-se1.getArea();
		if (areaDifference > 0.001) {
			metaCSPLogger.info("Intersection " + var1 + " with " + var2 + " seems corrupt (area increased by " + areaDifference + ") - skipping ");
			useDefaultEnvelopeChunks = true;
			//return toReturn;											
		}

		ArrayList<PoseSteering> var1sec1 = new ArrayList<PoseSteering>();
		ArrayList<PoseSteering> var1sec2 = new ArrayList<PoseSteering>();
		ArrayList<PoseSteering> var1sec3 = new ArrayList<PoseSteering>();

		boolean skipSec1 = false;
		boolean skipSec3 = false;

		if (useDefaultEnvelopeChunks) {
			float percentageChunckOne = 0.30f;
			float percentageChunckTwo = 0.40f;
			for (int i = 0; i < var1.getPathLength(); i++) {
				PoseSteering ps = var1.getTrajectory().getPoseSteering()[i];
				if (i < var1.getPathLength()*percentageChunckOne) var1sec1.add(ps);
				else if (i < var1.getPathLength()*(percentageChunckOne+percentageChunckTwo)) var1sec2.add(ps);
				else var1sec3.add(ps);
			}
			metaCSPLogger.info("Using default chunk sizes " + var1sec1.size() + " / " + var1sec2.size() + " / " + var1sec3.size());
		}
		else {	
			for (int i = 0; i < var1.getPathLength(); i++) {
				Coordinate coord = var1.getTrajectory().getPositions()[i];
				PoseSteering ps = var1.getTrajectory().getPoseSteering()[i];
				Point point = gf.createPoint(coord);
				Geometry fp = var1.makeFootprint(ps);
				if (!intersectionse1se2.intersects(fp) && var1sec2.isEmpty()) {
					var1sec1.add(ps);
				}
				else if (intersectionse1se2.intersects(fp)) {
					var1sec2.add(ps);
				}
				else if (!intersectionse1se2.intersects(fp) && !var1sec2.isEmpty()) {
					var1sec3.add(ps);
				}
			}

			//Add to start
			boolean done = false;
			while (!done) {
				try {
					Geometry lastPolySec1 = var1.makeFootprint(var1sec1.get(var1sec1.size()-1));
					if (lastPolySec1.disjoint(se2)) done = true;
					else {
						var1sec2.add(0,var1sec1.get(var1sec1.size()-1));
						var1sec1.remove(var1sec1.size()-1);
						metaCSPLogger.info("Added to start... (1)");
					}
				} catch (IndexOutOfBoundsException e) 
				{ skipSec1 = true; done = true; }
			}
			//If sec1 emptied, remove it
			if (var1sec1.size() < MINIMUM_SIZE) {
				while (var1sec1.size() > 0) {
					var1sec2.add(0,var1sec1.get(var1sec1.size()-1));
					var1sec1.remove(var1sec1.size()-1);
				}
				skipSec1 = true;
			}

			//Add to end
			done = false;
			while (!done) {
				try {
					Geometry firstPolySec3 = var1.makeFootprint(var1sec3.get(0));
					if (firstPolySec3.disjoint(se2)) done = true;
					else {
						var1sec2.add(var1sec3.get(0));
						var1sec3.remove(0);
						//					logger.info("Added to end... (1)");
					}
				} catch (IndexOutOfBoundsException e) { skipSec3 = true; done = true; }
			}
			//If sec3 emptied, remove it
			if (var1sec3.size() < MINIMUM_SIZE) {
				while (var1sec3.size() > 0) {
					var1sec2.add(var1sec3.get(0));
					var1sec3.remove(0);
				}
				skipSec3 = true;
			}

			if (var1sec2.size() < MINIMUM_SIZE) {
				if (var1sec1.size() > MINIMUM_SIZE) {
					var1sec2.add(0,var1sec1.get(var1sec1.size()-1));
					var1sec1.remove(var1sec1.size()-1);
					//				logger.info("Added to start... (2)");
				}
				else if (var1sec3.size() > MINIMUM_SIZE) {
					var1sec2.add(var1sec3.get(0));
					var1sec3.remove(0);				
					//				logger.info("Added to end... (2)");
				}
			}

			if ((skipSec1 && skipSec3) || (!skipSec1 && var1sec1.size() < MINIMUM_SIZE) || (!skipSec3 && var1sec3.size() < MINIMUM_SIZE) || var1sec2.size() < MINIMUM_SIZE) {
				metaCSPLogger.fine("Intersection " + var1 + " with " + var2 + " too small - skipping");
				return toReturn;
			}

		}

		var1.setRefinable(false);
		ArrayList<Trajectory> newTrajectories = new ArrayList<Trajectory>();
		ArrayList<TrajectoryEnvelope> newTrajectoryEnvelopes = new ArrayList<TrajectoryEnvelope>();

		if (!skipSec1) {
			newTrajectories.add(new Trajectory(var1sec1.toArray(new PoseSteering[var1sec1.size()]),var1.getTrajectory().getDts(0, var1sec1.size())));
			newTrajectories.add(new Trajectory(var1sec2.toArray(new PoseSteering[var1sec2.size()]),var1.getTrajectory().getDts(var1sec1.size(), var1sec1.size()+var1sec2.size())));
			if (!skipSec3) {
				newTrajectories.add(new Trajectory(var1sec3.toArray(new PoseSteering[var1sec3.size()]),var1.getTrajectory().getDts(var1sec1.size()+var1sec2.size(),var1.getTrajectory().getPoseSteering().length)));
			}
		}
		else {
			newTrajectories.add(new Trajectory(var1sec2.toArray(new PoseSteering[var1sec2.size()]),var1.getTrajectory().getDts(0, var1sec2.size())));
			if (!skipSec3) {
				newTrajectories.add(new Trajectory(var1sec3.toArray(new PoseSteering[var1sec3.size()]),var1.getTrajectory().getDts(var1sec2.size(),var1.getTrajectory().getPoseSteering().length)));
			}			
		}

		Variable[] newVars = trajectoryEnvelopeSolver.createVariables(newTrajectories.size());
		for (int i = 0; i < newVars.length; i++) {
			TrajectoryEnvelope te = (TrajectoryEnvelope)newVars[i];
			//te.setFootprint(var1.getWidth(), var1.getLength(), var1.getDeltaW(), var1.getDeltaL());
			te.setFootprint(var1.getFootprint());
			//Only for second!
			//			if ((!skipSec1 && i == 1) || (skipSec1 && i == 0)) {
			//				te.setRefinable(false);
			//				refinedWith.get(geometricShapeVariable).add(te);
			//			}
			//			System.out.println("doing i = " + i + " skipsec1: " + skipSec1 + " skipsec3: " + skipSec3);
			te.setTrajectory(newTrajectories.get(i));
			te.setSuperEnvelope(var1);
			te.setRobotID(var1.getRobotID());
			var1.addSubEnvelope(te);
			newTrajectoryEnvelopes.add(te);			
		}

		AllenIntervalConstraint starts = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Starts);
		starts.setFrom(newTrajectoryEnvelopes.get(0));
		starts.setTo(var1);
		toReturn.addConstraint(starts);

		AllenIntervalConstraint finishes = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Finishes);
		finishes.setFrom(newTrajectoryEnvelopes.get(newTrajectoryEnvelopes.size()-1));
		finishes.setTo(var1);
		toReturn.addConstraint(finishes);

		double minTTT12 = 0.0;

		if (!skipSec1) minTTT12 = var1.getTrajectory().getDTs()[var1sec1.size()];
		else minTTT12 = var1.getTrajectory().getDTs()[var1sec2.size()];
		long minTimeToTransition12 = (long)(TrajectoryEnvelope.RESOLUTION*minTTT12);
		AllenIntervalConstraint before1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before, new Bounds(minTimeToTransition12,minTimeToTransition12));
		before1.setFrom(newTrajectoryEnvelopes.get(0));
		before1.setTo(newTrajectoryEnvelopes.get(1));
		toReturn.addConstraint(before1);

		if (newTrajectoryEnvelopes.size() > 2) {
			double minTTT23 = var1.getTrajectory().getDTs()[var1sec1.size()+var1sec2.size()];
			long minTimeToTransition23 = (long)(TrajectoryEnvelope.RESOLUTION*minTTT23);
			AllenIntervalConstraint before2 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before, new Bounds(minTimeToTransition23,minTimeToTransition23));
			before2.setFrom(newTrajectoryEnvelopes.get(1));
			before2.setTo(newTrajectoryEnvelopes.get(2));
			toReturn.addConstraint(before2);
		}

		//		System.out.println("var1sec1 (" + skipSec1 + "): " + var1sec1);
		//		System.out.println("var1sec2: " + var1sec2);
		//		System.out.println("var1sec3 (" + skipSec3 + "): " + var1sec3);
		//		System.out.println("DTs of var1sec2: " + Arrays.toString(var1.getTrajectory().getDts( var1sec2.size(),var1.getTrajectory().getDTs().length-1 )));
		trajectoryEnvelopeSolver.addConstraints(toReturn.getConstraints());

		return toReturn;
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
		
		//subscribe to robot pose topic
		for (int i = 1; i <= NUMBEROFROBOTS; i++) {
			subscribeToRobotReportTopic(i);
		}
		
		setupSolvers();
		
		//Exercises: see in this method
		setupConstraintNetwork();
		
		if (dispatching) setupMonitoring();
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
}


