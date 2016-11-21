package se.oru.aass.lucia2016.test;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ValueOrderingH;
import org.metacsp.framework.Variable;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatial.DE9IM.PolygonalDomain;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.time.Bounds;
import org.metacsp.utility.UI.TrajectoryEnvelopeAnimator;
import org.metacsp.utility.logging.MetaCSPLogging;

import se.oru.aass.lucia2016.meta.RobotAllocationMetaConstraint;
import se.oru.aass.lucia2016.meta.ViewCoordinator;
import se.oru.aass.lucia2016.meta.ViewSchedulingMetaConstraint;
import se.oru.aass.lucia2016.meta.ViewSelectionMetaConstraint;
import se.oru.aass.lucia2016.multi.ViewConstraintSolver;
import se.oru.aass.lucia2016.multi.ViewVariable;

import com.vividsolutions.jts.geom.Coordinate;

public class TestOverallDesign {
	
	private Logger metaCSPLogger = MetaCSPLogging.getLogger(this.getClass());

	public static void main(String[] args) {
		
		
		ViewCoordinator metaSolver = new ViewCoordinator(0, 1000000, 100);
		ViewConstraintSolver solver = (ViewConstraintSolver)metaSolver.getConstraintSolvers()[0];
		MetaCSPLogging.setLevel(ViewCoordinator.class, Level.FINEST);
		
		
		//Here all the views we get fron FLAP4Caios: 
		
		Variable[] vars = solver.createVariables(3);
		ViewVariable vv1 = (ViewVariable)vars[0];
		ViewVariable vv2 = (ViewVariable)vars[1];
		ViewVariable vv3 = (ViewVariable)vars[2];
		//all the poses we get from FLAP4Caios
//		Pose viewPose1 = new Pose(16.757438659668, 16.4915008544922, 0.998125874099799);
//		Pose viewPose2 = new Pose(4.91268301010132, 9.12090492248535, -0.48102);
//		Pose viewPose3 = new Pose(12.8536548614502, -12.7020282745361, -1.5017);
		
		Pose viewPose1 = new Pose(-12.7615280151367, -6.15457534790039, -3.12842980123163);
		Pose viewPose2 = new Pose(-14.5186862945557, -2.39040231704712, -1.51838576694655);
		Pose viewPose3 = new Pose(-19.23854637146, -6.94821929931641, 0.551774602413026);
		
		
		//set the view cone for the FoV
		Coordinate c7 = new Coordinate(0.0, 0.0);
		Coordinate c8 = new Coordinate(6.0, 8.0);
		Coordinate c9 = new Coordinate(6.0, -8.0);
		


		
		//this is the turtlebot footprint coordinate
		Coordinate c1 = new Coordinate(1.2, 0.0);
		Coordinate c2 = new Coordinate(0.6, 1.2);
		Coordinate c3 = new Coordinate(-0.6, 1.2);
		Coordinate c4 = new Coordinate(-1.2, 0.0);
		Coordinate c5 = new Coordinate(-0.6, -1.2);
		Coordinate c6 = new Coordinate(0.6, -1.2);
			
		

		
		Trajectory trajRobot1 = new Trajectory(new Pose[] {viewPose1});
		vv1.getTrajectoryEnvelope().setFootprint(c1,c2,c3,c4,c5,c6);
		vv1.getTrajectoryEnvelope().setTrajectory(trajRobot1);
		vv1.setFOVCoordinates(c7,c8,c9);
		//vv1.setInfoGain

		
		Trajectory trajRobot2 = new Trajectory(new Pose[] {viewPose2});
		vv2.getTrajectoryEnvelope().setFootprint(c1,c2,c3,c4,c5,c6);
		vv2.getTrajectoryEnvelope().setTrajectory(trajRobot2);
		vv2.setFOVCoordinates(c7,c8,c9);


		Trajectory trajRobot3 = new Trajectory(new Pose[] {viewPose3});
		vv3.getTrajectoryEnvelope().setFootprint(c1,c2,c3,c4,c5,c6);
		vv3.getTrajectoryEnvelope().setTrajectory(trajRobot3);
		vv3.setFOVCoordinates(c7,c8,c9);

		
		//RobotInitialPose		
		Pose initPoseT1 = new Pose(0.030, 0.001, 0.006);
		Pose initPoseT2 = new Pose(4.990, 9.088, -0.49);
		Pose initPoseT3 = new Pose(16.789, 16.53, 1.02);
		

		AllenIntervalConstraint duration1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(10000,10000));
		duration1.setFrom(vv1);
		duration1.setTo(vv1);
		solver.addConstraint(duration1);

		AllenIntervalConstraint duration2 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(10000,10000));
		duration2.setFrom(vv2);
		duration2.setTo(vv2);
		solver.addConstraint(duration2);

		AllenIntervalConstraint duration3 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(10000,10000));
		duration3.setFrom(vv3);
		duration3.setTo(vv3);
		solver.addConstraint(duration3);
		
		//adding the meta-constraints
		ViewSelectionMetaConstraint viewSelectionMC = new ViewSelectionMetaConstraint(null, null) ;
		viewSelectionMC.setRobotNumber(3);
		metaSolver.addMetaConstraint(viewSelectionMC);
		
		RobotAllocationMetaConstraint RobotAllocationMC = new RobotAllocationMetaConstraint(null, null);
		metaSolver.addMetaConstraint(RobotAllocationMC);
		
		ViewSchedulingMetaConstraint viewSchedulingMC = new ViewSchedulingMetaConstraint(null, null);
		metaSolver.addMetaConstraint(viewSchedulingMC);
		viewSchedulingMC.setUsage(vv1,vv2,vv3);

		final Random rand = new Random(Calendar.getInstance().getTimeInMillis());
		viewSchedulingMC.setValOH(new ValueOrderingH() {
			@Override
			public int compare(ConstraintNetwork arg0, ConstraintNetwork arg1) {
				return (rand.nextInt(3)-1);
			}
		});
				
		
		metaSolver.backtrack();

		//==============================================================================================================================
		//visualization
		//==============================================================================================================================
		
		
		
		System.out.println(vv1 + " has domain " + vv1.getDomain());
		System.out.println(vv2 + " has domain " + vv2.getDomain());
		
//		System.out.println(vv1 + " has domain " + vv1.getDomain());
//		System.out.println("===================\n== AFTER SOLVING ==\n===================");
//		System.out.println(vv1.getInfo());
//
		TrajectoryEnvelopeAnimator tea = new TrajectoryEnvelopeAnimator("This is a test");
		tea.setTrajectoryEnvelopes(solver.getTrajectoryEnvelopeSolver().getConstraintNetwork());
		tea.addExtraGeometries(((PolygonalDomain)vv1.getFoV().getDomain()).getGeometry(), ((PolygonalDomain)vv2.getFoV().getDomain()).getGeometry(),((PolygonalDomain)vv3.getFoV().getDomain()).getGeometry());
		
	}
	
}
