package se.oru.aass.lucia2016.exercises;
import java.util.Arrays;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.Variable;
import org.metacsp.framework.multi.MultiConstraintSolver;
import org.metacsp.framework.multi.MultiVariable;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatial.DE9IM.DE9IMRelationSolver;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.metacsp.time.TimePoint;
import org.metacsp.utility.UI.ConstraintNetworkHierarchyFrame;
import org.metacsp.utility.UI.TrajectoryEnvelopeAnimator;

import com.vividsolutions.jts.geom.Coordinate;

import edu.uci.ics.jung.graph.DelegateTree;

public class Ex2 {
	
		
	public static void main(String[] args) {

		TrajectoryEnvelopeSolver solver = new TrajectoryEnvelopeSolver(0, 1000000, 100);
		Variable[] vars = solver.createVariables(2);

		//Small robot, 1.4 (w) x 3.4 (l) centered in the middle
		Coordinate frontLeft = new Coordinate(2.7, 0.7);
		Coordinate frontRight = new Coordinate(2.7, -0.7);
		Coordinate backRight = new Coordinate(-1.7, -0.7);
		Coordinate backLeft = new Coordinate(-1.7, 0.7);

		vars[0].setComponent("Robot1");
		TrajectoryEnvelope trajEnvelopeRobot1 = (TrajectoryEnvelope)vars[0];
		trajEnvelopeRobot1.getSymbolicVariableActivity().setSymbolicDomain("move_base");
		Trajectory trajRobot1 = new Trajectory("../paths/path1.path");
		trajEnvelopeRobot1.setFootprint(backLeft,backRight,frontLeft,frontRight);
		trajEnvelopeRobot1.setTrajectory(trajRobot1);
		trajEnvelopeRobot1.setRobotID(1);
		
		vars[1].setComponent("Robot2");
		TrajectoryEnvelope trajEnvelopeRobot2 = (TrajectoryEnvelope)vars[1];
		trajEnvelopeRobot2.getSymbolicVariableActivity().setSymbolicDomain("move_base");
		Trajectory trajRobot2 = new Trajectory("../paths/path3.path");
		trajEnvelopeRobot2.setFootprint(backLeft,backRight,frontLeft,frontRight);
		trajEnvelopeRobot2.setTrajectory(trajRobot2);
		trajEnvelopeRobot2.setRobotID(2);

		TrajectoryEnvelopeAnimator tea = new TrajectoryEnvelopeAnimator("TrajecotryEnvelope of " + trajEnvelopeRobot1.getComponent());
		tea.addTrajectoryEnvelopes(trajEnvelopeRobot1, trajEnvelopeRobot2);
		
		ConstraintNetwork.draw(solver.getConstraintNetwork());
	}
	
}
