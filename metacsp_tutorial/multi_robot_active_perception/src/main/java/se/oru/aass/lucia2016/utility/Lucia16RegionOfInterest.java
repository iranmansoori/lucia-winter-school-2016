package se.oru.aass.lucia2016.utility;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import geometry_msgs.Vector3;

import java.util.ArrayList;
import java.util.logging.Logger;

import org.metacsp.utility.logging.MetaCSPLogging;
import org.ros.node.ConnectedNode;
import uos_active_perception_msgs.BoundingBox;

public class Lucia16RegionOfInterest {
	
	private static Logger metaCSPLogger = MetaCSPLogging.getLogger(Lucia16RegionOfInterest.class);

	public static ArrayList<BoundingBox> getBoundnigBoxes(ConnectedNode node){
		ArrayList<BoundingBox> ret = new ArrayList<BoundingBox>();
		BoundingBox bbAreaE = getBBAreaE(node);
		ret.add(bbAreaE);
		BoundingBox bbAreaNW = getBBAreaNW(node);
		ret.add(bbAreaNW);		
		BoundingBox bbAreaSW = getBBAreaSW(node);
		ret.add(bbAreaSW);
		return ret;
	}



	private static BoundingBox getBBAreaE(ConnectedNode node) {
		BoundingBox bb = node.getTopicMessageFactory().newFromType(BoundingBox._TYPE);
		PoseStamped ps = node.getTopicMessageFactory().newFromType(PoseStamped._TYPE);
		Pose pose = node.getTopicMessageFactory().newFromType(Pose._TYPE);
		Point position = node.getTopicMessageFactory().newFromType(Point._TYPE);
		position.setX(5.2);
		position.setY(2.1);
		position.setZ(0.275);
		pose.setPosition(position);
		ps.setPose(pose);
		bb.setPoseStamped(ps);
		Vector3 vc = node.getTopicMessageFactory().newFromType(Vector3._TYPE);
		vc.setX(1.0);
		vc.setY(1.6);
		vc.setZ(0.55);
		bb.setDimensions(vc);
		return bb; 		
	}
	
	private static BoundingBox getBBAreaNW(ConnectedNode node) {
		BoundingBox bb = node.getTopicMessageFactory().newFromType(BoundingBox._TYPE);
		PoseStamped ps = node.getTopicMessageFactory().newFromType(PoseStamped._TYPE);
		Pose pose = node.getTopicMessageFactory().newFromType(Pose._TYPE);
		Point position = node.getTopicMessageFactory().newFromType(Point._TYPE);
		position.setX(2.3);
		position.setY(3.6);
		position.setZ(0.275);
		pose.setPosition(position);
		ps.setPose(pose);
		bb.setPoseStamped(ps);
		Vector3 vc = node.getTopicMessageFactory().newFromType(Vector3._TYPE);
		vc.setX(1.8);
		vc.setY(1.0);
		vc.setZ(0.55);
		bb.setDimensions(vc);
		return bb; 	
	}
	
	private static BoundingBox getBBAreaSW(ConnectedNode node) {
		BoundingBox bb = node.getTopicMessageFactory().newFromType(BoundingBox._TYPE);
		PoseStamped ps = node.getTopicMessageFactory().newFromType(PoseStamped._TYPE);
		Pose pose = node.getTopicMessageFactory().newFromType(Pose._TYPE);
		Point position = node.getTopicMessageFactory().newFromType(Point._TYPE);
		position.setX(1.9);
		position.setY(1.3);
		position.setZ(0.275);
		pose.setPosition(position);
		ps.setPose(pose);
		bb.setPoseStamped(ps);
		Vector3 vc = node.getTopicMessageFactory().newFromType(Vector3._TYPE);
		vc.setX(1.8);
		vc.setY(1.0);
		vc.setZ(0.55);
		bb.setDimensions(vc);
		return bb; 	
	}
	
}
