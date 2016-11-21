package se.oru.aass.lucia2016.test;
import java.util.Arrays;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.meta.spatioTemporal.paths.Map;
import org.metacsp.meta.spatioTemporal.paths.TrajectoryEnvelopeScheduler;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatial.DE9IM.PolygonalDomain;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.UI.TrajectoryEnvelopeAnimator;

import se.oru.aass.lucia2016.multi.ViewConstraintSolver;
import se.oru.aass.lucia2016.multi.ViewVariable;



import com.vividsolutions.jts.geom.Coordinate;

public class TestViewSolver {
	
	public static void main(String[] args) {
		
		
		ViewConstraintSolver solver = new ViewConstraintSolver(0, 1000000, 100);
		Variable[] vars = solver.createVariables(2);
		ViewVariable vv1 = (ViewVariable)vars[0];
		ViewVariable vv2 = (ViewVariable)vars[1];
		

		//footprint
		Coordinate c1 = new Coordinate(1.2, 0.0);
		Coordinate c2 = new Coordinate(0.6, 1.2);
		Coordinate c3 = new Coordinate(-0.6, 1.2);
		Coordinate c4 = new Coordinate(-1.2, 0.0);
		Coordinate c5 = new Coordinate(-0.6, -1.2);
		Coordinate c6 = new Coordinate(0.6, -1.2);

		//set the view cone for the FoV
		Coordinate c7 = new Coordinate(0.0, 0.0);
		Coordinate c8 = new Coordinate(4.0, 4.0);
		Coordinate c9 = new Coordinate(4.0, -4.0);
		
		
		
		Pose initPoseT1 = new Pose(0.030, 0.001, 0.006);
		Pose initPoseT2 = new Pose(4.990, 9.088, -0.49);
		
//		Pose[] obsPose = new Pose[2];
//		obsPose[0] = initPoseT1;
//		obsPose[1] = initPoseT1;
		
//		Trajectory trajRobot1 = new Trajectory("paths/path1.path");		
		Trajectory trajRobot1 = new Trajectory(new Pose[] {initPoseT1});
		vv1.getTrajectoryEnvelope().setFootprint(c1,c2,c3,c4,c5,c6);
		vv1.getTrajectoryEnvelope().setTrajectory(trajRobot1);
		vv1.getTrajectoryEnvelope().setRobotID(1);		
		vv1.getTrajectoryEnvelope().setRefinable(false);
		vv1.setFOVCoordinates(c7,c8,c9);
		

//		Trajectory trajRobot2 = new Trajectory("paths/path3.path");		
		Trajectory trajRobot2 = new Trajectory(new Pose[] {initPoseT2});
		vv2.getTrajectoryEnvelope().setFootprint(c1,c2,c3,c4,c5,c6);
		vv2.getTrajectoryEnvelope().setTrajectory(trajRobot2);
		vv2.getTrajectoryEnvelope().setRobotID(2);	
		vv2.getTrajectoryEnvelope().setRefinable(false);
		vv2.setFOVCoordinates(c7,c8,c9);
		
		AllenIntervalConstraint duration1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(20000,20000));
		duration1.setFrom(vv1);
		duration1.setTo(vv1);
		solver.addConstraint(duration1);

		AllenIntervalConstraint duration2 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(20000,20000));
		duration2.setFrom(vv2);
		duration2.setTo(vv2);
		solver.addConstraint(duration2);
		
		System.out.println(vv1 + " has domain " + vv1.getDomain());
		System.out.println(vv2 + " has domain " + vv2.getDomain());
		
//		System.out.println(vv1 + " has domain " + vv1.getDomain());
//		System.out.println("===================\n== AFTER SOLVING ==\n===================");
//		System.out.println(vv1.getInfo());
//
		TrajectoryEnvelopeAnimator tea = new TrajectoryEnvelopeAnimator("This is a test");
		tea.setTrajectoryEnvelopes(solver.getTrajectoryEnvelopeSolver().getConstraintNetwork());
		tea.addExtraGeometries(((PolygonalDomain)vv1.getFoV().getDomain()).getGeometry(), ((PolygonalDomain)vv2.getFoV().getDomain()).getGeometry());
		
		
	}
	
}
