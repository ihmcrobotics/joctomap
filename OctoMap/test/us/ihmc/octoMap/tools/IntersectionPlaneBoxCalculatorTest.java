package us.ihmc.octoMap.tools;

import static org.junit.Assert.*;

import java.util.List;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.random.RandomTools;

public class IntersectionPlaneBoxCalculatorTest
{
   private static final int NUMBER_OF_ITERATIONS = 10000;
   private static final double EPS = 1.0e-7;

   @Test
   public void testRandomNormals() throws Exception
   {
      Random random = new Random(3424L);

      double cubeSize = 1.0;
      Point3d cubeCenter = new Point3d();
      IntersectionPlaneBoxCalculator calculator = new IntersectionPlaneBoxCalculator();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Vector3d planeNormal = RandomTools.generateRandomVector(random, 1.0);
         Point3d pointOnPlane = cubeCenter;
         calculator.setCube(cubeSize, cubeCenter);
         calculator.setPlane(pointOnPlane, planeNormal);
         List<Point3d> intersections = calculator.computeIntersections();

         for (int j = 0; j < intersections.size(); j++)
         {
            Point3d intersection = intersections.get(j);
            Vector3d sub = new Vector3d();
            sub.sub(intersection, pointOnPlane);
            assertEquals("Intersection is not on plane", 0.0, sub.dot(planeNormal), EPS);

            Vector3d v0 = new Vector3d();
            Vector3d v1 = new Vector3d();
            Vector3d v3 = new Vector3d();
            Point3d nextIntersection = intersections.get((j + 1) % intersections.size());
            Point3d previousIntersection = intersections.get(j == 0 ? intersections.size() - 1 : j - 1);
            v0.sub(intersection, nextIntersection);
            v1.sub(intersection, previousIntersection);
            v3.cross(v0, v1);
            assertTrue("Intersetions are not properly ordered", v3.dot(planeNormal) > 0.0);
         }
      }
   }

   @Test
   public void testRandomNormalsAndPointOnPlane() throws Exception
   {
      Random random = new Random(34424L);

      double cubeSize = 1.0;
      Point3d cubeCenter = new Point3d();
      IntersectionPlaneBoxCalculator calculator = new IntersectionPlaneBoxCalculator();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Vector3d planeNormal = RandomTools.generateRandomVector(random, 1.0);
         Point3d pointOnPlane = RandomTools.generateRandomPoint(random, 0.5, 0.5, 0.5);
         calculator.setCube(cubeSize, cubeCenter);
         calculator.setPlane(pointOnPlane, planeNormal);
         List<Point3d> intersections = calculator.computeIntersections();

         for (int j = 0; j < intersections.size(); j++)
         {
            Point3d intersection = intersections.get(j);
            Vector3d sub = new Vector3d();
            sub.sub(intersection, pointOnPlane);
            assertEquals("Intersection is not on plane", 0.0, sub.dot(planeNormal), EPS);

            Vector3d v0 = new Vector3d();
            Vector3d v1 = new Vector3d();
            Vector3d v3 = new Vector3d();
            Point3d nextIntersection = intersections.get((j + 1) % intersections.size());
            Point3d previousIntersection = intersections.get(j == 0 ? intersections.size() - 1 : j - 1);
            v0.sub(intersection, nextIntersection);
            v1.sub(intersection, previousIntersection);
            v3.cross(v0, v1);
            assertTrue("Intersetions are not properly ordered", v3.dot(planeNormal) > 0.0);
         }
      }
   }

   @Test
   public void testRandomNormalsPointOnPlaneAndCubeCenters() throws Exception
   {
      Random random = new Random(3424L);

      double cubeSize = 1.0;
      IntersectionPlaneBoxCalculator calculator = new IntersectionPlaneBoxCalculator();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Point3d cubeCenter = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
         Vector3d planeNormal = RandomTools.generateRandomVector(random, 1.0);
         Point3d pointOnPlane = RandomTools.generateRandomPoint(random, 0.5, 0.5, 0.5);
         pointOnPlane.add(cubeCenter);
         calculator.setCube(cubeSize, cubeCenter);
         calculator.setPlane(pointOnPlane, planeNormal);
         List<Point3d> intersections = calculator.computeIntersections();

         for (int j = 0; j < intersections.size(); j++)
         {
            Point3d intersection = intersections.get(j);
            Vector3d sub = new Vector3d();
            sub.sub(intersection, pointOnPlane);
            assertEquals("Intersection is not on plane", 0.0, sub.dot(planeNormal), EPS);

            Vector3d v0 = new Vector3d();
            Vector3d v1 = new Vector3d();
            Vector3d v3 = new Vector3d();
            Point3d nextIntersection = intersections.get((j + 1) % intersections.size());
            Point3d previousIntersection = intersections.get(j == 0 ? intersections.size() - 1 : j - 1);
            v0.sub(intersection, nextIntersection);
            v1.sub(intersection, previousIntersection);
            v3.cross(v0, v1);
            assertTrue("Intersetions are not properly ordered", v3.dot(planeNormal) > 0.0);
         }
      }
   }
}
