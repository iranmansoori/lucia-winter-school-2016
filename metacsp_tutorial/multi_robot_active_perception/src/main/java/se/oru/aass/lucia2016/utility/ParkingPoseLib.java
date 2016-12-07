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
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.spatial.geometry.Vec2;
import org.ros.node.ConnectedNode;


public class ParkingPoseLib {
	
	public static Pose getHardCodedMoveAwayParkingPose(int robotID) {
		if(robotID == 1) return new Pose(4.18, 4.16, 0.0);
			//return new Pose(0.42, 4.30, 0.0);
		else if(robotID == 2)
			return new Pose(5.24, 3.68, 0.0);
		else
			return new Pose(6.35, 4.14, 0.0);
	}
	
	
	public static void getMoveAwayPose(OccupancyGrid map, int robotID){

		//
		int width = map.getInfo().getWidth();
		int height = map.getInfo().getHeight();
		Random rand = new Random(Calendar.getInstance().getTimeInMillis());		
		int Low = 0;
		int High = width * height;
		int index = rand.nextInt(High-Low) + Low;
		
		//transfer this index to coordinate
		
		
		//geneate positions in a loop
		//make a turtule point
		//check those points whether are free
		
		
		
		
		
		
	}
	
	public static boolean isPointOccupied(OccupancyGrid map, double x, double y) {

		int width = map.getInfo().getWidth();
		int height = map.getInfo().getHeight();

		System.out.println("width and height: " + width + " " + height);
		float resolution = map.getInfo().getResolution();
		System.out.println("resolution: " + resolution);
		byte[] data = new byte[width * height];
		map.getData().getBytes(0, data);
		
		int x_index = (int) (x /resolution ) ;
		int y_index = (int) (y /resolution );
//		if(((((y_index) * width)) +  ((x_index))) < 0 || ((((y_index) * width)) +  ((x_index))) > data.length - 1){
//			
//			continue;
//		}
//		System.out.println(" " + (((y_index) * width)) +  ((x_index)) + " == " + (data.length - 1));
		if (data[ (((y_index) * width)) +  ((x_index))] == 0)
			return false;
		return true;
	}

}
