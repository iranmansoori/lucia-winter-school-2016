package se.oru.aass.lucia2016.utility;

import org.ros.node.ConnectedNode;

import std_msgs.Header;



import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import geometry_msgs.Quaternion;

public class Convertor {

	public static  double getOrientation(Quaternion q) {
		return Math.atan2(2.0*(q.getX()*q.getY()+q.getZ()*q.getW()),1.0-2*(q.getY()*q.getY()+q.getZ()*q.getZ()));
	}
	
	public static PoseStamped getPoseStamped(org.metacsp.multi.spatioTemporal.paths.Pose pose, final ConnectedNode node) {
		Point point1 = node.getTopicMessageFactory().newFromType(Point._TYPE);
		point1.setX(pose.getX());
		point1.setY(pose.getY());
		point1.setZ(0);
		Pose pos1 = node.getTopicMessageFactory().newFromType(Pose._TYPE);
		pos1.setPosition(point1);
		Quaternion quat1 = node.getTopicMessageFactory().newFromType(Quaternion._TYPE);
		quat1.setX(0);
		quat1.setY(0);		
		quat1.setZ(Math.sin(pose.getTheta()/2));
		quat1.setW(Math.cos(pose.getTheta()/2));
		pos1.setOrientation(quat1);
		Header header = node.getTopicMessageFactory().newFromType(Header._TYPE);
		header.setFrameId("map");
		PoseStamped p1  = node.getTopicMessageFactory().newFromType(PoseStamped._TYPE);
		p1.setPose(pos1);
		p1.setHeader(header);
		return p1;
	}

	public static PoseStamped getPoseStamped(geometry_msgs.Pose pose, final ConnectedNode node) {
		Header header = node.getTopicMessageFactory().newFromType(Header._TYPE);
		header.setFrameId("map");
		PoseStamped p  = node.getTopicMessageFactory().newFromType(PoseStamped._TYPE);
		p.setPose(pose);
		p.setHeader(header);
		return p;
	}

}
