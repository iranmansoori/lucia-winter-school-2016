package se.oru.aass.lucia2016.test;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.Random;
import java.util.Vector;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ValueOrderingH;
import org.metacsp.framework.Variable;
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
import org.metacsp.utility.logging.MetaCSPLogging;
import org.metacsp.utility.timelinePlotting.TimelinePublisher;
import org.metacsp.utility.timelinePlotting.TimelineVisualizer;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import nav_msgs.OccupancyGrid;
import se.oru.aass.lucia2016.execution.FlapForChaosDispatchingFunction;
import se.oru.aass.lucia2016.meta.RobotAllocationMetaConstraint;
import se.oru.aass.lucia2016.meta.RobotAllocationValOH;
import se.oru.aass.lucia2016.meta.ViewCoordinator;
import se.oru.aass.lucia2016.meta.ViewSchedulingMetaConstraint;
import se.oru.aass.lucia2016.meta.ViewSelectionMetaConstraint;
import se.oru.aass.lucia2016.meta.ViewSelectionValOH;
import se.oru.aass.lucia2016.multi.ViewConstraintSolver;
import se.oru.aass.lucia2016.multi.ViewVariable;
import se.oru.aass.lucia2016.utility.Convertor;
import se.oru.aass.lucia2016.utility.FootPrintFactory;
import se.oru.aass.lucia2016.utility.Lucia16RegionOfInterest;
import uos_active_perception_msgs.GetObservationCameraPoses;
import uos_active_perception_msgs.GetObservationCameraPosesRequest;
import uos_active_perception_msgs.GetObservationCameraPosesResponse;
//import org.metacsp.multi.spatioTemporal.paths.Pose;


public class MultiRobotCoordinator extends AbstractNodeMain {

	private static final long TEMPORAL_RESOLUTION = 1000000;
	private static final long CONTROL_PERIOD = 1000;
	protected static final float INFOGAIN_THRESHOLD = (float) 0.3;
	private static Logger metaCSPLogger = MetaCSPLogging.getLogger(MultiRobotCoordinator.class);
	private ConnectedNode connectedNode;
	private final String nodeName = "lucia_meta_csp_lecture";
	private ViewCoordinator metaSolver = null;
	private ViewConstraintSolver viewSolver = null;
	private ActivityNetworkSolver ans = null;
	private HashMap<Integer, geometry_msgs.Pose> robotsCurrentPose = new HashMap<Integer, geometry_msgs.Pose>();
	private Object semaphore = new Object();
	private boolean poseReceived = false; 
	private static final int NUMBEROFROBOTS = 3; 
	private boolean solved = false;
	private ConstraintNetworkAnimator animator = null;
	private Vector<Pose> cameraPoses = null;
	private Vector<Float> infoGains = null;
	private int cycle = 0;
	private static final long HORIZON = 10000000;
	private long ORIGIN = 0;
	private long SENSING_DURATION = 5000;
	private OccupancyGrid map = null;
	private FlapForChaosDispatchingFunction[] dfs = null;

	
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
		ans = (ActivityNetworkSolver)((TrajectoryEnvelopeSolver)viewSolver.getConstraintSolvers()[0]).getConstraintSolvers()[0];
		MetaCSPLogging.setLevel(ViewCoordinator.class, Level.FINEST);
		metaSolver.setROSNode(connectedNode);
		metaSolver.setRobotCurrentPose(robotsCurrentPose);
		metaSolver.setTemporalResolution(TEMPORAL_RESOLUTION);
		metaSolver.setMap(map);
				
		//Define and add the meta-constraints:
		//  1. ViewSelectionMetaConstraint: selects view poses to navigate to 
		ViewSelectionMetaConstraint viewSelectionMC = new ViewSelectionMetaConstraint(null, new ViewSelectionValOH());	
		viewSelectionMC.setNumberOfRobots(NUMBEROFROBOTS);
		metaSolver.addMetaConstraint(viewSelectionMC);
		
		//  2. RobotAllocationMetaConstraint: takes care of assigning robots to selected view poses
		RobotAllocationValOH robotSelectionValOH = new RobotAllocationValOH(metaSolver);
		RobotAllocationMetaConstraint robotAllocationMC = new RobotAllocationMetaConstraint(null, robotSelectionValOH);
		metaSolver.addMetaConstraint(robotAllocationMC);
		
		//  3. ViewSchedulingMetaConstraint: ensures that trajectory envelopes do not obstruct FoV of robots in view poses
		ViewSchedulingMetaConstraint viewSchedulingMC = new ViewSchedulingMetaConstraint(null, null);
		metaSolver.addMetaConstraint(viewSchedulingMC);
		//  Note: this meta-constraint has a random value ordering heuristic, so we get some variation even offline
		final Random rand = new Random(Calendar.getInstance().getTimeInMillis());
		viewSchedulingMC.setValOH(new ValueOrderingH() {
			@Override
			public int compare(ConstraintNetwork arg0, ConstraintNetwork arg1) {
				return (rand.nextInt(3)-1);
			}
		});
	}
	
	private void delayEndOfParkingUntilNow() {
		for (Variable var : (viewSolver.getConstraintSolvers()[0]).getVariables()) {
			TrajectoryEnvelope te = (TrajectoryEnvelope)var;
			if (te.getSymbolicVariableActivity().getSymbols()[0].equals("Parking")) {
				AllenIntervalConstraint deadline = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Deadline, new Bounds(getCurrentTime(), APSPSolver.INF));
				deadline.setFrom(te);
				deadline.setTo(te);
				(viewSolver.getConstraintSolvers()[0]).addConstraint(deadline);
			}
		}
	}
	
	// Start threads for online solving, plan dispatching, and execution monitoring
	public void setupMonitoring() {
	
		// The callback that is called at each cycle
		InferenceCallback cb = new InferenceCallback() {	
						
			private int myCycle = cycle++;
			
			@Override
			public void doInference(long timeNow) {
				boolean resolved = true;
				if(poseReceived)
					resolved = metaSolver.backtrack(); 
				if(!resolved) resetAll();
				if (metaSolver.getAddedResolvers().length > 0) {
					solved = true;
					metaCSPLogger.info("== SOLUTION at iteration " + myCycle + " ==");
					for (ConstraintNetwork cn : metaSolver.getAddedResolvers()) {
						for (Constraint con : cn.getConstraints()) {
							metaCSPLogger.info("\t" + con);
						}
					}
					metaCSPLogger.info("== END SOLUTION ==");
					delayEndOfParkingUntilNow();
					//show a timeline
					TimelinePublisher tp = new TimelinePublisher(ans.getConstraintNetwork(), new Bounds(0,60000), true, "Time", "turtlebot1", "turtlebot2", "turtlebot3");
					TimelineVisualizer tv = new TimelineVisualizer(tp);
					tv.startAutomaticUpdate(CONTROL_PERIOD);
					//tp.publish(false, false);

					//Save a copy of the constraint network for inspection/debugging
					ConstraintNetwork.saveConstraintNetwork(viewSolver.getTrajectoryEnvelopeSolver().getConstraintNetwork(), "testingLucia" + myCycle + ".cn");
					metaSolver.clearResolvers();									
				}
				
				boolean allDispatched = false;
				
				if(solved){
					int finishedActs = 0;
					int unSelectedVV = 0; 
					Variable[] tes = viewSolver.getTrajectoryEnvelopeSolver().getVariables();
					for (int i = 0; i < tes.length; i++) {
						TrajectoryEnvelope te = (TrajectoryEnvelope)tes[i];
						if(te.getRobotID() == -1){
							unSelectedVV++;
						}							
						if(te.getTemporalVariable().getLET() < connectedNode.getCurrentTime().totalNsecs()/TEMPORAL_RESOLUTION){
							finishedActs++;
						}							
					}
					metaCSPLogger.info("Progress: " + finishedActs + "/" + (tes.length - unSelectedVV) + " completed");
					if(finishedActs == tes.length - unSelectedVV){						
						allDispatched = true;
					}
				}

				if(allDispatched){
					resetAll();
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
			dfs[i-1] = new FlapForChaosDispatchingFunction("turtlebot"+i, viewSolver.getTrajectoryEnvelopeSolver(), connectedNode);
		}
		animator.addDispatchingFunctions(ans, dfs);		
		
	}
	
	// Reset all stops executing threads and re-instantiates and re-sets-up the solvers; it then asks for a next best view
	private void resetAll() {
		metaCSPLogger.info("Setting up solvers and getting next best views...");
		solved = false;
		poseReceived = false;
		for (FlapForChaosDispatchingFunction df : dfs) {
			df.deleteMessageListener();
		}
		animator.teardown();
		setupSolvers();
		setupMonitoring();
		cameraPoses.clear();
		infoGains.clear();
		synchronized (semaphore) {
			getNextBestView(cameraPoses, infoGains);
		}		
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
		
		setupResetAllService();
		
		subscribeToMapServer();
		
		cameraPoses = new Vector<Pose>();
		infoGains = new Vector<Float>();
		
		setupSolvers();
		setupMonitoring();
		
		synchronized (semaphore) {
			getNextBestView(cameraPoses, infoGains);
		}		
		
	}
	
	// For each next best view obtained from FlapForChaos, make a new ViewVariable with no robot assigned to it
	// (these ViewVariables constitute flaws for the ViewSelectionMetaConstraint)
	private Vector<ViewVariable> createViewVariables(Vector<Pose> cameraPoses, Vector<Float> infoGains) {
		Vector<ViewVariable> ret = new Vector<ViewVariable>();
		Variable[] vars = viewSolver.createVariables(cameraPoses.size());
		for (int i = 0; i < vars.length; i++) {
			ViewVariable vv1 = (ViewVariable)vars[i];
			ret.add(vv1);			
			vv1.getTrajectoryEnvelope().getSymbolicVariableActivity().setSymbolicDomain("Sense");
			Trajectory trajRobot1 = new Trajectory(new Pose[] {cameraPoses.get(i)});
			vv1.getTrajectoryEnvelope().setFootprint(FootPrintFactory.getTurtlebotFootprint());
			vv1.getTrajectoryEnvelope().setTrajectory(trajRobot1);
			vv1.setFOVCoordinates(FootPrintFactory.getFoVCoordinates());
			vv1.setInfoGain(infoGains.get(i));
			AllenIntervalConstraint duration1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(SENSING_DURATION ,APSPSolver.INF));
			duration1.setFrom(vv1);
			duration1.setTo(vv1);
			viewSolver.addConstraint(duration1);
		}
		
		//seting the usage
		ViewSchedulingMetaConstraint viewSchedulingMC = null;
		for (int i = 0; i < metaSolver.getMetaConstraints().length; i++) {			
			if(metaSolver.getMetaConstraints()[i] instanceof ViewSchedulingMetaConstraint){
				viewSchedulingMC = (ViewSchedulingMetaConstraint)metaSolver.getMetaConstraints()[i];
				break;
			}
		}
		viewSchedulingMC.setUsage(ret.toArray(new ViewVariable[ret.size()]));
		return ret;		
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
	
	private void subscribeToMapServer() {
		Subscriber<nav_msgs.OccupancyGrid> mapSubscriber = this.connectedNode.newSubscriber("/turtlebot1/move_base/global_costmap/costmap", nav_msgs.OccupancyGrid._TYPE);
		mapSubscriber.addMessageListener(new MessageListener<nav_msgs.OccupancyGrid>() {
			@Override
			public void onNewMessage(nav_msgs.OccupancyGrid message) {
				metaCSPLogger.info("Got the cost map!");
				map = message;
			}
		});
	}

	private void setupResetAllService() {
		this.connectedNode.newServiceServer("reset", services.reset._TYPE, new ServiceResponseBuilder<services.resetRequest, services.resetResponse>() {
			@Override
			public void build(services.resetRequest request, services.resetResponse response) {
				metaCSPLogger.info("Going for reset NOW!");
				cancelAllRobotMotions();
				resetAll();
				//tell all robots to cancel: send a me
			}
		});
	}
	
	private void cancelAllRobotMotions() {
		for (int i = 1; i <= NUMBEROFROBOTS; i++) {
			final actionlib_msgs.GoalID gd = connectedNode.getTopicMessageFactory().newFromType(actionlib_msgs.GoalID._TYPE);
			gd.setId("");
			final Publisher<actionlib_msgs.GoalID> publisher = connectedNode.newPublisher("/turtlebot" + i + "/move_base/cancel", actionlib_msgs.GoalID._TYPE);
			int counter = 0;
			while (counter < 100) {
				publisher.publish(gd);
				counter++;		
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}
	
	public void getNextBestView(final Vector<Pose> cameraPoses, final Vector<Float> infoGains) {
		
		ServiceClient<GetObservationCameraPosesRequest, GetObservationCameraPosesResponse> serviceClient;
		try {			
			serviceClient = connectedNode.newServiceClient("/turtlebot1/get_observation_camera_poses_reachable", GetObservationCameraPoses._TYPE);			

		} catch (ServiceNotFoundException e) {
			throw new RosRuntimeException(e);
		}
		final GetObservationCameraPosesRequest request = serviceClient.newMessage();
		request.setOmitCvm(true);
		request.setSampleSize(100);
		request.setRaySkip((float)0.8);		
		request.setRoi(Lucia16RegionOfInterest.getBoundnigBoxes(connectedNode));
		
		serviceClient.call(request, new ServiceResponseListener<GetObservationCameraPosesResponse>() {
			
			@Override
			public void onSuccess(GetObservationCameraPosesResponse response) {
				metaCSPLogger.info("successfully called get camera poses service! for the robot ");
				ArrayList<geometry_msgs.Pose> poses = (ArrayList<geometry_msgs.Pose>) response.getCameraPoses();
				float[] ig = response.getInformationGains();
				for (int i = 0; i < ig.length; i++) {
					if(ig[i] > INFOGAIN_THRESHOLD){
						System.out.println(i +": " + poses.get(i).getPosition().getX() + " " + poses.get(i).getPosition().getY());
						cameraPoses.add(new Pose(poses.get(i).getPosition().getX(), poses.get(i).getPosition().getY(), Convertor.getOrientation(poses.get(i).getOrientation())));
						infoGains.add(ig[i]);
					}
				}
				createViewVariables(cameraPoses, infoGains);
				poseReceived = true;
			}

			@Override
			public void onFailure(RemoteException arg0) {
				metaCSPLogger.info("failed to call service!");
			}
		});		
		
	}

}
