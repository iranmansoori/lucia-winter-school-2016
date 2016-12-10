package se.oru.aass.lucia2016.utility;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;

public class FootPrintFactory {
	
	
	public static Coordinate[] getTurtlebotFootprint(){
		Coordinate[] ret = new Coordinate[6];		
		ret[0] = new Coordinate(0.18, 0.0);
		ret[1] = new Coordinate(0.09, 0.18);
		ret[2] = new Coordinate(-0.09, 0.18);
		ret[3] = new Coordinate(-0.18, 0.0);
		ret[4] = new Coordinate(-0.09, -0.18);
		ret[5] = new Coordinate(0.09, -0.18);
		
//		ret[0] = new Coordinate(1.2, 0.0);
//		ret[1] = new Coordinate(0.6, 1.2);
//		ret[2] = new Coordinate(-0.6, 1.2);
//		ret[3] = new Coordinate(-1.2, 0.0);
//		ret[4] = new Coordinate(-0.6, -1.2);
//		ret[5] = new Coordinate(0.6, -1.2);
		
		return ret;
	}
	
	public static Polygon getTurtlebotFootprintPolygon(){
		Coordinate[] ret = new Coordinate[7];		
		ret[0] = new Coordinate(0.18, 0.0);
		ret[1] = new Coordinate(0.09, 0.18);
		ret[2] = new Coordinate(-0.09, 0.18);
		ret[3] = new Coordinate(-0.18, 0.0);
		ret[4] = new Coordinate(-0.09, -0.18);
		ret[5] = new Coordinate(0.09, -0.18);
		ret[6] = new Coordinate(0.18, 0.0);
		
//		ret[0] = new Coordinate(1.2, 0.0);
//		ret[1] = new Coordinate(0.6, 1.2);
//		ret[2] = new Coordinate(-0.6, 1.2);
//		ret[3] = new Coordinate(-1.2, 0.0);
//		ret[4] = new Coordinate(-0.6, -1.2);
//		ret[5] = new Coordinate(0.6, -1.2);
		
		GeometryFactory gf = new GeometryFactory();
		return gf.createPolygon(ret);
	}

	public static Coordinate[] getFoVCoordinates(){
		Coordinate[] ret = new Coordinate[3];		
		ret[0] = new Coordinate(0.0, 0.0);
		ret[1] = new Coordinate(1.6, 0.9);
		ret[2] = new Coordinate(1.6, -0.9);
		return ret;
	}
	
	
}
