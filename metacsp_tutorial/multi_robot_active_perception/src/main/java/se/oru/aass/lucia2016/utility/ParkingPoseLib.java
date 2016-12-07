package se.oru.aass.lucia2016.utility;

import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Random;
import java.util.Vector;

import nav_msgs.OccupancyGrid;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.metacsp.framework.Variable;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.spatial.geometry.Vec2;
import org.ros.node.ConnectedNode;

import se.oru.aass.lucia2016.multi.ViewConstraintSolver;
import se.oru.aass.lucia2016.multi.ViewVariable;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;


public class ParkingPoseLib {
	
	private static int SAMPLE_SIZE = 100;
	private static int MAX_X = 670;
	private static int MAX_Y = 460;
	
	public static Pose getHardCodedMoveAwayParkingPose(int robotID) {
		if(robotID == 1) return new Pose(4.18, 4.16, 0.0);
			//return new Pose(0.42, 4.30, 0.0);
		else if(robotID == 2)
			return new Pose(5.24, 3.68, 0.0);
		else
			return new Pose(6.35, 4.14, 0.0);
	}
	
	
	public static Pose getMoveAwayPose(OccupancyGrid map, Pose currRobPose, ViewConstraintSolver viewSolver, int robotId){
		double minDist = Double.MAX_VALUE;
		Pose minPose = null;
		for(int sample = 0; sample < SAMPLE_SIZE; sample++) {
			Random rand_x = new Random(Calendar.getInstance().getTimeInMillis());
			Random rand_y = new Random(Calendar.getInstance().getTimeInMillis());
			Random rand_theta = new Random(Calendar.getInstance().getTimeInMillis());
			double coord_x = (double)rand_x.nextInt(MAX_X)/100;
			double coord_y = (double)rand_y.nextInt(MAX_Y)/100;
			double coord_theta = (double)rand_theta.nextInt(314)/100;
			
			Geometry geo = TrajectoryEnvelope.getFootprint(FootPrintFactory.getTurtlebotFootprintPolygon(), coord_x, coord_y, coord_theta);			
			Coordinate [] coords = geo.getCoordinates();
			boolean isFree = true;
			for (int i = 0; i < coords.length; i++) {
				if(isPointOccupied(map, coords[i].x, coords[i].y)){
					isFree = false;
				}
			}
			
			if (isFree) {
				Variable[]  vars = viewSolver.getVariables();
				for (int i = 0; i < vars.length; i++) {
					ViewVariable vv = (ViewVariable)vars[i];
					if(vv.getTrajectoryEnvelope().getRobotID() != -1 && vv.getTrajectoryEnvelope().getRobotID() != robotId){
						Geometry shape1 = ((GeometricShapeDomain)vv.getFoV().getDomain()).getGeometry();		
						if(shape1.intersects(geo)){
							isFree = false;
						}
						Geometry shape2 = ((GeometricShapeDomain)vv.getTrajectoryEnvelope().getEnvelopeVariable().getDomain()).getGeometry();		
						if(shape2.intersects(geo)){
							isFree = false;
						}						
					}
				}
			}
			
			if(isFree){
				double dist = Math.sqrt(Math.pow(currRobPose.getX() - coord_x, 2) + Math.pow(currRobPose.getY() - coord_y, 2));			
				if(dist < minDist) {
					minDist = dist;
					minPose = new Pose(coord_x, coord_y, coord_theta);
				}
			}
		}
		if (minPose == null) {
			System.out.println("+++++++++++++++++++++++ RETURNING HARD CODED POSE!!!!");
			return getHardCodedMoveAwayParkingPose(robotId);
		}
		return minPose;		
	}
	
	public static boolean isPointOccupied(OccupancyGrid map, double x, double y) {
		if (x <= 0 || y <= 0) return false;
		
		int width = map.getInfo().getWidth();
		int height = map.getInfo().getHeight();
		float resolution = map.getInfo().getResolution();
		byte[] data = new byte[width * height];
		map.getData().getBytes(0, data);
		int grid_x = (int)((x - map.getInfo().getOrigin().getPosition().getX()) / resolution);
		int grid_y = (int)((y - map.getInfo().getOrigin().getPosition().getY()) / resolution);
//		System.out.println("ORIG: " + x + " " + y);
//		System.out.println("GRID: " + grid_x + " " + grid_y);
//		System.out.println("SAMPLE SHADE: " + data[grid_y*width+grid_x]);
		if (data[grid_y*width+grid_x] < 20) return false;
		return true;
	}

}
