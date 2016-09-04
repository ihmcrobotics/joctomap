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
         Point3d pointOnPlane = cubeCenter;
         Vector3d planeNormal = RandomTools.generateRandomVector(random, 1.0);
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
            assertTrue("Intersections are not properly ordered", v3.dot(planeNormal) > 0.0);
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
         Point3d pointOnPlane = RandomTools.generateRandomPoint(random, 0.5, 0.5, 0.5);
         Vector3d planeNormal = RandomTools.generateRandomVector(random, 1.0);
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
            assertTrue("Intersections are not properly ordered", v3.dot(planeNormal) > 0.0);
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
         Point3d pointOnPlane = RandomTools.generateRandomPoint(random, 0.5, 0.5, 0.5);
         Vector3d planeNormal = RandomTools.generateRandomVector(random, 1.0);
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
            if (v3.dot(planeNormal) < 0.0)
            {
               System.err.println("      Point3d cubeCenter = new Point3d" + cubeCenter + ";");
               System.err.println("      Point3d pointOnPlane = new Point3d" + pointOnPlane + ";");
               System.err.println("      Vector3d planeNormal = new Vector3d" + planeNormal + ";");
            }
            assertTrue("Intersections are not properly ordered", v3.dot(planeNormal) > 0.0);
         }
      }
   }

   @Test
   public void testBug1() throws Exception
   {
      double cubeSize = 0.1;
      IntersectionPlaneBoxCalculator calculator = new IntersectionPlaneBoxCalculator();

      Point3d cubeCenter = new Point3d(4.25, 0.9500000000000001, 0.75);
      Point3d pointOnPlane = new Point3d(4.200864791870117, 0.9091876149177551, 0.7372332811355591);
      Vector3d planeNormal = new Vector3d(-0.7001400589942932, -0.7001400589942932, 0.14002801477909088);
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
         System.out.println(v3.dot(planeNormal) > 0.0);
         assertTrue("Intersections are not properly ordered", v3.dot(planeNormal) > 0.0);
      }
   }

   @Test
   public void testBug2() throws Exception
   {
      double cubeSize = 0.1;
      IntersectionPlaneBoxCalculator calculator = new IntersectionPlaneBoxCalculator();

      Point3d cubeCenter = new Point3d(1.35, -1.85, 0.15000000000000002);
      Vector3d planeNormal = new Vector3d(-0.6383859515190125, 0.39992544054985046, 0.6576648950576782);
      Point3d pointOnPlane = new Point3d(1.3115897178649902, -1.8882930278778076, 0.10343723744153976);
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
         System.out.println(v3.dot(planeNormal) > 0.0);
         assertTrue("Intersections are not properly ordered", v3.dot(planeNormal) > 0.0);
      }
   }

   @Test
   public void testBug3() throws Exception
   {
      double cubeSize = 0.1;
      IntersectionPlaneBoxCalculator calculator = new IntersectionPlaneBoxCalculator();

      Point3d cubeCenter = new Point3d(-0.25, -0.45, -0.05);
      Point3d pointOnPlane = new Point3d(-0.2242894023656845, -0.4647734463214874, -0.0023258039727807045);
      Vector3d planeNormal = new Vector3d(0.20791170661191224, 1.503689309739766E-8, 0.9781475973766547);
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
         System.out.println(v3.dot(planeNormal) > 0.0);
         assertTrue("Intersections are not properly ordered", v3.dot(planeNormal) > 0.0);
      }
   }
}
