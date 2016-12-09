package se.oru.aass.lucia2016.exercises;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Vector;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.Variable;
import org.metacsp.framework.multi.MultiConstraintSolver;
import org.metacsp.framework.multi.MultiVariable;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatial.DE9IM.DE9IMRelationSolver;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.metacsp.time.TimePoint;
import org.metacsp.utility.UI.ConstraintNetworkHierarchyFrame;
import org.metacsp.utility.UI.TrajectoryEnvelopeAnimator;
import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import com.vividsolutions.jts.geom.Coordinate;

import edu.uci.ics.jung.graph.DelegateTree;
import se.oru.aass.lucia2016.utility.Convertor;
import se.oru.aass.lucia2016.utility.FootPrintFactory;
import se.oru.aass.lucia2016.utility.PathPlanFactory;

public class Ex3 {
		
	public static void main(String[] args) {

		TrajectoryEnvelopeSolver solver = new TrajectoryEnvelopeSolver(0, 1000000, 100);
		Variable[] vars = solver.createVariables(4);

		vars[0].setComponent("Robot1");
		TrajectoryEnvelope trajEnvelopeRobot1 = (TrajectoryEnvelope)vars[0];
		trajEnvelopeRobot1.getSymbolicVariableActivity().setSymbolicDomain("move_base");
		Trajectory trajRobot1 = new Trajectory("../paths/path1.path");
		trajEnvelopeRobot1.setFootprint(FootPrintFactory.getTurtlebotFootprint());
		trajEnvelopeRobot1.setTrajectory(trajRobot1);
		trajEnvelopeRobot1.setRobotID(1);
		
		vars[1].setComponent("Robot2");
		TrajectoryEnvelope trajEnvelopeRobot2 = (TrajectoryEnvelope)vars[1];
		trajEnvelopeRobot2.getSymbolicVariableActivity().setSymbolicDomain("move_base");
		Trajectory trajRobot2 = new Trajectory("../paths/path3.path");
		trajEnvelopeRobot2.setFootprint(FootPrintFactory.getTurtlebotFootprint());
		trajEnvelopeRobot2.setTrajectory(trajRobot2);
		trajEnvelopeRobot2.setRobotID(2);

		vars[2].setComponent("Robot1");
		TrajectoryEnvelope parkingRobot1 = (TrajectoryEnvelope)vars[2];
		parkingRobot1.getSymbolicVariableActivity().setSymbolicDomain("parking");
		Trajectory parkingPoseRobot1 = new Trajectory(new Pose[] {trajRobot1.getPose()[0]});
		parkingRobot1.setFootprint(FootPrintFactory.getTurtlebotFootprint());
		parkingRobot1.setTrajectory(parkingPoseRobot1);
		parkingRobot1.setRobotID(1);
		
		vars[3].setComponent("Robot2");
		TrajectoryEnvelope parkingRobot2 = (TrajectoryEnvelope)vars[3];
		parkingRobot2.getSymbolicVariableActivity().setSymbolicDomain("parking");
		Trajectory parkingPoseRobot2 = new Trajectory(new Pose[] {trajRobot2.getPose()[0]});
		parkingRobot2.setFootprint(FootPrintFactory.getTurtlebotFootprint());
		parkingRobot2.setTrajectory(parkingPoseRobot2);
		parkingRobot2.setRobotID(2);

		AllenIntervalConstraint parkingMeetsMoveRobot1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Meets);
		parkingMeetsMoveRobot1.setFrom(parkingRobot1);
		parkingMeetsMoveRobot1.setTo(trajEnvelopeRobot1);
		solver.addConstraint(parkingMeetsMoveRobot1);
		
		AllenIntervalConstraint parkingMeetsMoveRobot2 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Meets);
		parkingMeetsMoveRobot2.setFrom(parkingRobot2);
		parkingMeetsMoveRobot2.setTo(trajEnvelopeRobot2);
		solver.addConstraint(parkingMeetsMoveRobot2);

		AllenIntervalConstraint releaseParkingRobot1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(0,0));
		releaseParkingRobot1.setFrom(parkingRobot1);
		releaseParkingRobot1.setTo(parkingRobot1);
		solver.addConstraint(releaseParkingRobot1);
		
		AllenIntervalConstraint releaseParkingRobot2 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(0,0));
		releaseParkingRobot2.setFrom(parkingRobot2);
		releaseParkingRobot2.setTo(parkingRobot2);
		solver.addConstraint(releaseParkingRobot2);

		AllenIntervalConstraint releaseMoveRobot1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(10000,APSPSolver.INF));
		releaseMoveRobot1.setFrom(trajEnvelopeRobot1);
		releaseMoveRobot1.setTo(trajEnvelopeRobot1);
		solver.addConstraint(releaseMoveRobot1);
		
		AllenIntervalConstraint releaseMoveRobot2 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(20000,APSPSolver.INF));
		releaseMoveRobot2.setFrom(trajEnvelopeRobot2);
		releaseMoveRobot2.setTo(trajEnvelopeRobot2);
		solver.addConstraint(releaseMoveRobot2);

		TrajectoryEnvelopeAnimator tea = new TrajectoryEnvelopeAnimator("TrajecotryEnvelope of " + trajEnvelopeRobot1.getComponent());
		tea.addTrajectoryEnvelopes(parkingRobot1, parkingRobot2, trajEnvelopeRobot1, trajEnvelopeRobot2);
				
		ConstraintNetwork.draw(solver.getConstraintNetwork());
	}
		
}
