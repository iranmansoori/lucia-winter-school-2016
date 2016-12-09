package se.oru.aass.lucia2016.utility;

import geometry_msgs.PoseStamped;
import geometry_msgs.Quaternion;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Vector;
import java.util.logging.Logger;

import nav_msgs.GetPlan;
import nav_msgs.GetPlanRequest;
import nav_msgs.GetPlanResponse;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.utility.logging.MetaCSPLogging;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;





public class PathPlanFactory {
	
	private static Logger metaCSPLogger = MetaCSPLogging.getLogger(PathPlanFactory.class);

	public static void getRobotPathPlanFromROSSerive(final HashMap<Integer, Boolean> robToPathStatus, final HashMap<Integer, Vector<Pose>> robToPathPoses, ConnectedNode connectedNode, final int robotId, PoseStamped start, PoseStamped end) {
		
		ConnectedNode node = connectedNode;
		//final Vector<Pose> ret = new Vector<Pose>();
		ServiceClient<GetPlanRequest, GetPlanResponse> serviceClient;
		try {			
			serviceClient = node.newServiceClient("/turtlebot" + robotId + "/move_base/make_plan", GetPlan._TYPE);
			metaCSPLogger.info("connected to path planner for the robot " + robotId);

		} catch (ServiceNotFoundException e) {
			throw new RosRuntimeException(e);
		}
		final GetPlanRequest request = serviceClient.newMessage();
		request.setStart(start);
		request.setGoal(end);
		
		serviceClient.call(request, new ServiceResponseListener<GetPlanResponse>() {
			
			@Override
			public void onSuccess(GetPlanResponse response) {
				metaCSPLogger.info("successfully called path planner service! for the robot " + robotId);
				ArrayList<PoseStamped> poses = (ArrayList<PoseStamped>) response.getPlan().getPoses();
				//System.out.println("-----------------------------");
				Vector<Pose> ps = new Vector<Pose>();
				for (int i = 0; i < poses.size(); i++) {
					//System.out.println(poses.get(i).getPose().getPosition().getX() + " " + poses.get(i).getPose().getPosition().getY());
					ps.add(new Pose(poses.get(i).getPose().getPosition().getX(), poses.get(i).getPose().getPosition().getY(), 
							Convertor.getOrientation(poses.get(i).getPose().getOrientation())));
				}
				robToPathPoses.put(robotId, ps);
				robToPathStatus.put(robotId, true);
				//System.out.println("-----------------------------");
			}

			@Override
			public void onFailure(RemoteException arg0) {
				metaCSPLogger.info("failed to call service!");
			}
		});		
		
	}
	
	
	
	

}
