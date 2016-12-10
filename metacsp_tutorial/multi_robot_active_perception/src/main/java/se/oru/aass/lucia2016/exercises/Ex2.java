package se.oru.aass.lucia2016.exercises;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.Variable;
import org.metacsp.framework.multi.MultiConstraintSolver;
import org.metacsp.framework.multi.MultiVariable;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.UI.TrajectoryEnvelopeAnimator;

import com.vividsolutions.jts.geom.Coordinate;

import edu.uci.ics.jung.graph.DelegateTree;

public class Ex2 {
	
		
	public static void main(String[] args) {
		
		TrajectoryEnvelopeSolver solver = new TrajectoryEnvelopeSolver(0, 1000000, 100);
		Variable[] vars = solver.createVariables(2, "Robot1");
		TrajectoryEnvelope trajEnvelopeRobot1 = (TrajectoryEnvelope)vars[0];		
		TrajectoryEnvelope parkingEnvelopeRobot1 = (TrajectoryEnvelope)vars[1];		
		trajEnvelopeRobot1.getSymbolicVariableActivity().setSymbolicDomain("MoveBase");
		parkingEnvelopeRobot1.getSymbolicVariableActivity().setSymbolicDomain("Parking");

		//Small robot, 1.4 (w) x 3.4 (l) centered in the middle
		Coordinate frontLeft = new Coordinate(2.7, 0.7);
		Coordinate frontRight = new Coordinate(2.7, -0.7);
		Coordinate backRight = new Coordinate(-1.7, -0.7);
		Coordinate backLeft = new Coordinate(-1.7, 0.7);

		Trajectory trajRobot1 = new Trajectory("../paths/path1.path");
		trajEnvelopeRobot1.setFootprint(backLeft,backRight,frontLeft,frontRight);
		trajEnvelopeRobot1.setTrajectory(trajRobot1);
		trajEnvelopeRobot1.setRobotID(1);
		
		Trajectory parkingPoseRobot1 = new Trajectory(new Pose[] {trajEnvelopeRobot1.getTrajectory().getPose()[0]});
		parkingEnvelopeRobot1.setFootprint(backLeft,backRight,frontLeft,frontRight);
		parkingEnvelopeRobot1.setTrajectory(parkingPoseRobot1);
		parkingEnvelopeRobot1.setRobotID(1);
		
		//TODO 2: change into meets and see result
		AllenIntervalConstraint before = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before, new Bounds(1,APSPSolver.INF));
		before.setFrom(parkingEnvelopeRobot1);
		before.setTo(trajEnvelopeRobot1);
		solver.addConstraints(before);
		
		//TODO 1: add a release on moving
//		AllenIntervalConstraint release = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(10000,APSPSolver.INF));
//		release.setFrom(trajEnvelopeRobot1);
//		release.setTo(trajEnvelopeRobot1);
//		solver.addConstraints(release);
		
		ConstraintNetwork.draw(solver.getConstraintNetwork());
		
		TrajectoryEnvelopeAnimator tea = new TrajectoryEnvelopeAnimator("TrajecotryEnvelope of " + trajEnvelopeRobot1.getComponent());
		tea.addTrajectoryEnvelopes(parkingEnvelopeRobot1, trajEnvelopeRobot1);

		
	}
	
}
