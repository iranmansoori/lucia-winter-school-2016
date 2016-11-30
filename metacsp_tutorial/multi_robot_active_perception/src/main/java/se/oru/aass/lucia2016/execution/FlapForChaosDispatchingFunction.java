package se.oru.aass.lucia2016.execution;
import geometry_msgs.PoseStamped;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.Vector;
import java.util.logging.Logger;



import org.metacsp.dispatching.DispatchingFunction;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.utility.logging.MetaCSPLogging;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Subscriber;

import actionlib_msgs.GoalID;
import actionlib_msgs.GoalStatus;




import se.oru.aass.lucia2016.meta.ViewCoordinator;
import se.oru.aass.lucia2016.multi.ViewConstraintSolver;
import se.oru.aass.lucia2016.utility.Convertor;
import services.sendGoal;
import services.sendGoalRequest;
import services.sendGoalResponse;



public class FlapForChaosDispatchingFunction extends DispatchingFunction {

	public static final Vector<SymbolicVariableActivity> executingActs = new Vector<SymbolicVariableActivity>();
	final Logger metaCSPLogger = MetaCSPLogging.getLogger(FlapForChaosDispatchingFunction.class);
	private ConnectedNode node = null;
	private int counter;
	private static int MIN_MESSAGES = 30;
	private boolean isExecuting = false;
	private SymbolicVariableActivity currentAct = null;
	private ViewCoordinator metaSolver = null;
	
	public FlapForChaosDispatchingFunction(String component, ViewCoordinator metaSolver, ConnectedNode rosNode) {
		super(component);
		this.node = rosNode;
		this.metaSolver = metaSolver;
		
		//Subscribe to movebase feedback topic
		if(this.node != null){			
			Subscriber<actionlib_msgs.GoalStatusArray> actionlibFeedback = rosNode.newSubscriber("/" + this.component + "/move_base/status", actionlib_msgs.GoalStatusArray._TYPE);
			
			actionlibFeedback.addMessageListener(new MessageListener<actionlib_msgs.GoalStatusArray>() {
				@Override
				public void onNewMessage(actionlib_msgs.GoalStatusArray message) {
					if (message.getStatusList() != null && !message.getStatusList().isEmpty()) {
						//goalID = message.getStatusList().size()-1;	
						GoalStatus gs = message.getStatusList().get(0);		
						if (isExecuting()) {
							//System.out.println(">>>>>>>>>>>>>>>>>> (" + this.+ ") ACTIONLIB SAYS: " + gs.getStatus());
							if (gs.getStatus() != (byte)1) {
								finishCurrentActivity();
							}
						}
					}
				}
			}, 10);
		}
	}
	
	private boolean isExecuting() {
		return isExecuting;
	}
	
	private  void setExecuting(boolean exec) {
		isExecuting = exec;
	}

	public void finishCurrentActivity() {
		System.out.println(">>>>>>>>>>>>>>>>>>> (" + this.component + ") FINISHED!!!");
		this.finish(currentAct);
		currentAct = null;
		setExecuting(false);
	}
	
	@Override
	public boolean skip(SymbolicVariableActivity act) {
		// TODO Auto-generated method stub
		return false;
	}
	
	
	@Override
	public void dispatch(SymbolicVariableActivity act) {
		System.out.println(">>> " + this.getComponent() + " starts executing " + act);
		currentAct = act;
		executingActs.add(act);		
		if (act.getSymbols()[0].equals("Sense")) {
			final long sleepTime = act.getTemporalVariable().getDuration().min;
			final FlapForChaosDispatchingFunction thisDF = this;
			final SymbolicVariableActivity theAct = act;
			new Thread() {
				public void run() {
					long start = Calendar.getInstance().getTimeInMillis();
					while (Calendar.getInstance().getTimeInMillis() - start < sleepTime) {
						try { Thread.sleep(10); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
					thisDF.finish(theAct);
					executingActs.remove(theAct);
				}
			}.start();
		}
		else{//call ROS service to send the robot			
			//get the trajectory envelope belongs to the paths, and send the last pose to the sendGoal Service
			TrajectoryEnvelopeSolver teSolver = ((ViewConstraintSolver)this.metaSolver.getConstraintSolvers()[0]).getTrajectoryEnvelopeSolver();
			Variable[] vars = teSolver.getVariables();
			for (int i = 0; i < vars.length; i++) {
				TrajectoryEnvelope te = (TrajectoryEnvelope)vars[i];
				if(te.getSymbolicVariableActivity().equals(act)){
					Pose vvPose = te.getTrajectory().getPose()[te.getTrajectory().getPose().length - 1];
					sendGoal(vvPose.getX(), vvPose.getY(), vvPose.getTheta());
					break;
				}					
			}
		}
				
	}

	private void sendGoal(double x, double y, double theta) {
        ServiceClient<sendGoalRequest, sendGoalResponse> serviceClient = null;
		try { serviceClient = node.newServiceClient("/"+this.component+"/sendGoal", sendGoal._TYPE); }
		catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		final sendGoalRequest request = serviceClient.newMessage();
		request.setX(x);
		request.setY(y);
		request.setTheta(theta);
		serviceClient.call(request, new ServiceResponseListener<sendGoalResponse>() {
			@Override
			public void onSuccess(sendGoalResponse arg0) {
				setExecuting(true);					
			}
			
			@Override
			public void onFailure(RemoteException arg0) {
				// TODO Auto-generated method stub
				
			}

		});		
	}
	
}
