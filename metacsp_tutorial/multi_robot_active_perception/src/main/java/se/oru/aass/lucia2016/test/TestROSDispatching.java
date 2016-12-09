package se.oru.aass.lucia2016.test;
import geometry_msgs.PoseStamped;
import geometry_msgs.PoseWithCovarianceStamped;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.Random;
import java.util.Vector;
import java.util.logging.Level;
import java.util.logging.Logger;

import nav_msgs.OccupancyGrid;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ValueOrderingH;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatial.DE9IM.PolygonalDomain;
import org.metacsp.multi.spatioTemporal.paths.Pose;
//import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.sensing.ConstraintNetworkAnimator;
import org.metacsp.sensing.InferenceCallback;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.UI.TrajectoryEnvelopeAnimator;
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
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Subscriber;


import actionlib_msgs.GoalStatus;

import com.vividsolutions.jts.geom.Geometry;

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
import se.oru.aass.lucia2016.utility.ParkingPoseLib;
import se.oru.aass.lucia2016.utility.PathPlanFactory;
import uos_active_perception_msgs.GetObservationCameraPoses;
import uos_active_perception_msgs.GetObservationCameraPosesRequest;
import uos_active_perception_msgs.GetObservationCameraPosesResponse;



public class TestROSDispatching extends AbstractNodeMain {

	private static final int TEMPORAL_RESOLUTION = 1000000;
	private static final int CONTROL_PERIOD = 1000;
	protected static final float INFOGAIN_THRESHOLD = (float) 0.3;
	private static Logger metaCSPLogger = MetaCSPLogging.getLogger(TestROSDispatching.class);
	private static ConnectedNode connectedNode;
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

	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(nodeName);
	}
	
	public long getCurrentTime() {
		return connectedNode.getCurrentTime().totalNsecs()/TEMPORAL_RESOLUTION;
	}
	
	private void setupSolvers() {
		ORIGIN = getCurrentTime();
		metaSolver = new ViewCoordinator(ORIGIN, ORIGIN+HORIZON, 100);
		viewSolver = (ViewConstraintSolver)metaSolver.getConstraintSolvers()[0];
		ans = (ActivityNetworkSolver)((TrajectoryEnvelopeSolver)viewSolver.getConstraintSolvers()[0]).getConstraintSolvers()[0];
		MetaCSPLogging.setLevel(ViewCoordinator.class, Level.FINEST);
		metaSolver.setROSNode(connectedNode);
		metaSolver.setRobotCurrentPose(robotsCurrentPose);
		metaSolver.setTimeNow(getCurrentTime());
		metaSolver.setMap(map);
				
		//adding the meta-constraints
		ViewSelectionMetaConstraint viewSelectionMC = new ViewSelectionMetaConstraint(null, new ViewSelectionValOH());	
		//ViewSelectionMetaConstraint viewSelectionMC = new ViewSelectionMetaConstraint(null, null);		
		viewSelectionMC.setNumberOfRobots(NUMBEROFROBOTS);
		metaSolver.addMetaConstraint(viewSelectionMC);
		
		RobotAllocationValOH robotSelectionValOH = new RobotAllocationValOH(metaSolver);
		RobotAllocationMetaConstraint robotAllocationMC = new RobotAllocationMetaConstraint(null, robotSelectionValOH);
		metaSolver.addMetaConstraint(robotAllocationMC);
		
		ViewSchedulingMetaConstraint viewSchedulingMC = new ViewSchedulingMetaConstraint(null, null);
		metaSolver.addMetaConstraint(viewSchedulingMC);

		final Random rand = new Random(Calendar.getInstance().getTimeInMillis());
		viewSchedulingMC.setValOH(new ValueOrderingH() {
			@Override
			public int compare(ConstraintNetwork arg0, ConstraintNetwork arg1) {
				return (rand.nextInt(3)-1);
			}
		});
	}
	
	public void setupMonitoring() {
		//final TrajectoryEnvelopeAnimator tea = new TrajectoryEnvelopeAnimator("Solution");		
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
					//visualizationTEAnimator(tea);
					metaCSPLogger.info("== SOLUTION at iteration " + myCycle + " ==");
					for (ConstraintNetwork cn : metaSolver.getAddedResolvers()) {
						for (Constraint con : cn.getConstraints()) {
							metaCSPLogger.info("\t" + con);
						}
					}
					metaCSPLogger.info("== END SOLUTION ==");
					//show timeline
					TimelinePublisher tp = new TimelinePublisher(ans.getConstraintNetwork(), new Bounds(0,60000), true, "Time", "turtlebot1", "turtlebot2", "turtlebot3");
					TimelineVisualizer tv = new TimelineVisualizer(tp);
					tv.startAutomaticUpdate(CONTROL_PERIOD);
					//tp.publish(false, false);
					
					//ConstraintNetwork.draw(viewSolver.getTrajectoryEnvelopeSolver().getConstraintNetwork());
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
					System.out.println("FINISHED ACTS: " + finishedActs);
					System.out.println("VARS NUMBER: " + (tes.length - unSelectedVV));
					if(finishedActs == tes.length - unSelectedVV){						
						allDispatched = true;
					}
				}

				if(allDispatched){
					resetAll();
				}
			}


		};
		
		animator = new ConstraintNetworkAnimator(ans, CONTROL_PERIOD, cb){
			@Override
			protected long getCurrentTimeInMillis() {				
				return getCurrentTime();
			}
		};		
		FlapForChaosDispatchingFunction[] dfs = new FlapForChaosDispatchingFunction[NUMBEROFROBOTS];
		for (int i = 1; i <= NUMBEROFROBOTS; i++) {
			dfs[i-1] = new FlapForChaosDispatchingFunction("turtlebot"+i, viewSolver.getTrajectoryEnvelopeSolver(), connectedNode);
		}
		animator.addDispatchingFunctions(ans, dfs);		
		//tea.setConstraintNetworkAnimator(animator);
		
	}
	
	private void resetAll() {
		System.out.println(">>>>> All already dispatched actions have been finished!!!!!!");
		solved = false;
		poseReceived = false;
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
			subscribeToResetTopic(i);
		}
		
		subscribeToMapServer();
		
		cameraPoses = new Vector<Pose>();
		infoGains = new Vector<Float>();
		
		//offline setting for test
		//setCameraPoses(cameraPoses, infoGains);
		//vvs = createViewVariables(cameraPoses, infoGains);
		
		setupSolvers();
		setupMonitoring();
		
		synchronized (semaphore) {
			getNextBestView(cameraPoses, infoGains);
		}		
		
	}
	
	private void visualizationTEAnimator(TrajectoryEnvelopeAnimator tea) {
		tea.setTrajectoryEnvelopes(viewSolver.getTrajectoryEnvelopeSolver().getConstraintNetwork());
		Variable[] vars = viewSolver.getVariables();
		Vector<ViewVariable> vvs = new Vector<ViewVariable>();
		for (int i = 0; i < vars.length; i++) {
			if(vars[i] instanceof ViewVariable){
				ViewVariable vv = ((ViewVariable)vars[i]);
				if(viewSolver.getViewConstraintByVariable(vv) != null){
					vvs.add(vv);
				}			
										
			}
		}		
		Geometry[] gms = new Geometry[vvs.size()]; 
		for (int i1 = 0; i1 < vvs.size(); i1++) {
			gms[i1] = ((PolygonalDomain)vvs.get(i1).getFoV().getDomain()).getGeometry();
		}
		tea.addExtraGeometries(gms);				
	}

	
	private  Vector<ViewVariable> createViewVariables(Vector<Pose> cameraPoses, Vector<Float> infoGains) {
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

	private void setCameraPoses(Vector<Pose> cameraPoses, Vector<Float> infoGains) {
		cameraPoses.add(new Pose(1.05, 4.31, -3.12842980123163));		
		infoGains.add((float)0.8);
		cameraPoses.add(new Pose(6.09, 0.82, 0.551774602413026));
		infoGains.add((float)0.8);
		cameraPoses.add(new Pose(3.55, 2.96, -1.51838576694655));
		infoGains.add((float)0.8);
		cameraPoses.add(new Pose(3.06, 0.72, 0.551774602413026));
		infoGains.add((float)0.8);

	}

	private void subscribeToRobotReportTopic(final int i) {
		Subscriber<geometry_msgs.PoseWithCovarianceStamped> poseFeedback = connectedNode.newSubscriber("/turtlebot" + i + "/amcl_pose", geometry_msgs.PoseWithCovarianceStamped._TYPE);
		poseFeedback.addMessageListener(new MessageListener<geometry_msgs.PoseWithCovarianceStamped>() {
			@Override
			public void onNewMessage(geometry_msgs.PoseWithCovarianceStamped message) {
				//currentPose = message.getPose();
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

	private void subscribeToResetTopic(final int i) {
		
		Subscriber<actionlib_msgs.GoalID> mapSubscriber = this.connectedNode.newSubscriber("/turtlebot" + i + "/move_base/cancel", actionlib_msgs.GoalID._TYPE);
		mapSubscriber.addMessageListener(new MessageListener<actionlib_msgs.GoalID>() {
			@Override
			public void onNewMessage(actionlib_msgs.GoalID message) {
				metaCSPLogger.info("Reset all and start over!");
				resetAll();
			}
		});
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
