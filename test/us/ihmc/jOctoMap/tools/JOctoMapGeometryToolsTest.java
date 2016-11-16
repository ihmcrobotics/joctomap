package us.ihmc.jOctoMap.tools;

import static org.junit.Assert.*;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.jOctoMap.tools.JOctoMapGeometryTools.RayBoxIntersectionResult;

public class JOctoMapGeometryToolsTest
{
   @Test
   public void testDistancceFromPointToLine()
   {
      Random random = new Random(6546354L);

      for (int i = 0; i < 10000; i++)
      {
         Point3d lineStart = JOctoMapRandomTools.generateRandomPoint3d(random, 10.0, 10.0, 10.0);
         Point3d lineEnd = JOctoMapRandomTools.generateRandomPoint3d(random, 10.0, 10.0, 10.0);
         Vector3d lineDirection = new Vector3d();
         lineDirection.sub(lineEnd, lineStart);
         lineDirection.normalize();

         Vector3d randomPerpendicular = JOctoMapRandomTools.generateRandomOrthogonalVector3d(random, lineDirection, true);
         assertEquals(0.0, lineDirection.dot(randomPerpendicular), 1.0e-10);

         double expectedDistance = JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0);
         Point3d pointAtExpectedDistanceFromLine = new Point3d();
         pointAtExpectedDistanceFromLine.scaleAdd(expectedDistance, randomPerpendicular, lineStart);

         double actualDistance = JOctoMapGeometryTools.distanceFromPointToLine(pointAtExpectedDistanceFromLine, lineStart, lineEnd);

         assertEquals(expectedDistance, actualDistance, 1.0e-10);
      }
   }

   @Test
   public void testGetRotationBasedOnNormalGivenReference() throws Exception
   {
      Random random = new Random(3453L);

      for (int i = 0; i < 10000; i++)
      {
         Vector3d referenceNormal = JOctoMapRandomTools.generateRandomVector3d(random, 1.0);
         Vector3d rotatedNormal = new Vector3d();
         Vector3d expectedRotationAxis = JOctoMapRandomTools.generateRandomOrthogonalVector3d(random, referenceNormal, true);
         assertEquals(0.0, referenceNormal.dot(expectedRotationAxis), 1.0e-10);

         double expectedRotationAngle = JOctoMapRandomTools.generateRandomDouble(random, 0.0, Math.PI);

         AxisAngle4d expectedRotation = new AxisAngle4d(expectedRotationAxis, expectedRotationAngle);
         Matrix3d rotationMatrix = new Matrix3d();
         rotationMatrix.set(expectedRotation);
         rotationMatrix.transform(referenceNormal, rotatedNormal);

         AxisAngle4d actualRotation = new AxisAngle4d();
         JOctoMapGeometryTools.getRotationBasedOnNormal(actualRotation, rotatedNormal, referenceNormal);

         assertTrue("Expected: " + expectedRotation + ", actual: " + actualRotation, expectedRotation.epsilonEquals(actualRotation, 1.0e-10));
      }

      Vector3d referenceNormal = JOctoMapRandomTools.generateRandomVector3d(random, 1.0);
      Vector3d rotatedNormal = new Vector3d(referenceNormal);
      AxisAngle4d expectedRotation = new AxisAngle4d(1.0, 0.0, 0.0, 0.0);
      AxisAngle4d actualRotation = new AxisAngle4d();
      JOctoMapGeometryTools.getRotationBasedOnNormal(actualRotation, rotatedNormal, referenceNormal);
      assertTrue("Expected: " + expectedRotation + ", actual: " + actualRotation, expectedRotation.epsilonEquals(actualRotation, 1.0e-10));

      rotatedNormal.negate();
      expectedRotation.setAngle(Math.PI);
      JOctoMapGeometryTools.getRotationBasedOnNormal(actualRotation, rotatedNormal, referenceNormal);
      assertTrue("Expected: " + expectedRotation + ", actual: " + actualRotation, expectedRotation.epsilonEquals(actualRotation, 1.0e-10));
   }

   @Test
   public void testGetRotationBasedOnNormalWithDefaultReference() throws Exception
   {
      Random random = new Random(3453L);
      Vector3d referenceNormal = new Vector3d(0.0, 0.0, 1.0);

      for (int i = 0; i < 10000; i++)
      {
         Vector3d rotatedNormal = new Vector3d();
         Vector3d expectedRotationAxis = JOctoMapRandomTools.generateRandomOrthogonalVector3d(random, referenceNormal, true);
         assertEquals(0.0, referenceNormal.dot(expectedRotationAxis), 1.0e-10);

         double expectedRotationAngle = JOctoMapRandomTools.generateRandomDouble(random, 0.0, Math.PI);

         AxisAngle4d expectedRotation = new AxisAngle4d(expectedRotationAxis, expectedRotationAngle);
         Matrix3d rotationMatrix = new Matrix3d();
         rotationMatrix.set(expectedRotation);
         rotationMatrix.transform(referenceNormal, rotatedNormal);

         AxisAngle4d actualRotation = new AxisAngle4d();
         JOctoMapGeometryTools.getRotationBasedOnNormal(actualRotation, rotatedNormal);
         assertTrue("Expected: " + expectedRotation + ", actual: " + actualRotation, expectedRotation.epsilonEquals(actualRotation, 1.0e-10));
         actualRotation = JOctoMapGeometryTools.getRotationBasedOnNormal(rotatedNormal);
         assertTrue("Expected: " + expectedRotation + ", actual: " + actualRotation, expectedRotation.epsilonEquals(actualRotation, 1.0e-10));
      }

      Vector3d rotatedNormal = new Vector3d(referenceNormal);
      AxisAngle4d expectedRotation = new AxisAngle4d(1.0, 0.0, 0.0, 0.0);
      AxisAngle4d actualRotation = new AxisAngle4d();
      JOctoMapGeometryTools.getRotationBasedOnNormal(actualRotation, rotatedNormal, referenceNormal);
      assertTrue("Expected: " + expectedRotation + ", actual: " + actualRotation, expectedRotation.epsilonEquals(actualRotation, 1.0e-10));
      actualRotation = JOctoMapGeometryTools.getRotationBasedOnNormal(rotatedNormal);
      assertTrue("Expected: " + expectedRotation + ", actual: " + actualRotation, expectedRotation.epsilonEquals(actualRotation, 1.0e-10));

      rotatedNormal.negate();
      expectedRotation.setAngle(Math.PI);
      JOctoMapGeometryTools.getRotationBasedOnNormal(actualRotation, rotatedNormal, referenceNormal);
      assertTrue("Expected: " + expectedRotation + ", actual: " + actualRotation, expectedRotation.epsilonEquals(actualRotation, 1.0e-10));
      actualRotation = JOctoMapGeometryTools.getRotationBasedOnNormal(rotatedNormal);
      assertTrue("Expected: " + expectedRotation + ", actual: " + actualRotation, expectedRotation.epsilonEquals(actualRotation, 1.0e-10));
   }

   @Test
   public void testRayBoxIntersectionWithRayOriginatingFromInsideTheBox() throws Exception
   {
      Random random = new Random(32452L);

      // Test with ray originating from inside the box
      for (int i = 0; i < 10000; i++)
      {
         Vector3d size = JOctoMapRandomTools.generateRandomVector3d(random, 10.0, 10.0, 10.0);
         size.absolute();
         Point3d min = JOctoMapRandomTools.generateRandomPoint3d(random, 10.0, 10.0, 10.0);
         Point3d max = new Point3d();
         max.add(min, size);

         Point3d rayOrigin = JOctoMapRandomTools.generateRandomPoint3d(random, min, max);

         if (i == 0)
            rayOrigin.interpolate(min, max, 0.5);

         Vector3d rayDirection = new Vector3d();

         PointsOnEachBoxFace randomPointsOnEachBoxFace = generateRandomPointsOnEachBoxFace(random, min, max);
         RayBoxIntersectionResult intersectionResult;

         for (Point3d expectedExitingIntersection : randomPointsOnEachBoxFace.toArray())
         {
            rayDirection.sub(expectedExitingIntersection, rayOrigin);
            rayDirection.normalize();

            intersectionResult = JOctoMapGeometryTools.rayBoxIntersection(min, max, rayOrigin, rayDirection);
            assertNotNull(intersectionResult);
            assertNull(intersectionResult.getEnteringIntersection());
            assertTrue(expectedExitingIntersection.epsilonEquals(intersectionResult.getExitingIntersection(), 1.0e-10));
         }

         BoxCorners boxCorners = extractBoxCorners(min, max);

         for (Point3d expectedExitingIntersection : boxCorners.toArray())
         {
            rayDirection.sub(expectedExitingIntersection, rayOrigin);
            rayDirection.normalize();

            intersectionResult = JOctoMapGeometryTools.rayBoxIntersection(min, max, rayOrigin, rayDirection);
            assertNotNull(intersectionResult);
            assertNull(intersectionResult.getEnteringIntersection());
            assertTrue(expectedExitingIntersection.epsilonEquals(intersectionResult.getExitingIntersection(), 1.0e-10));
         }

         BoxFaceCenters boxFaceCenters = extractFaceCenters(min, max);

         for (Point3d expectedExitingIntersection : boxFaceCenters.toArray())
         {
            rayDirection.sub(expectedExitingIntersection, rayOrigin);
            rayDirection.normalize();

            intersectionResult = JOctoMapGeometryTools.rayBoxIntersection(min, max, rayOrigin, rayDirection);
            assertNotNull(intersectionResult);
            assertNull(intersectionResult.getEnteringIntersection());
            assertTrue(expectedExitingIntersection.epsilonEquals(intersectionResult.getExitingIntersection(), 1.0e-10));
         }

         Point3d expectedExitingIntersection = new Point3d();

         // Test simple cases
         expectedExitingIntersection.set(rayOrigin);
         rayDirection.set(1.0, 0.0, 0.0);
         expectedExitingIntersection.setX(max.getX());
         intersectionResult = JOctoMapGeometryTools.rayBoxIntersection(min, max, rayOrigin, rayDirection);
         assertNotNull(intersectionResult);
         assertNull(intersectionResult.getEnteringIntersection());
         assertTrue(expectedExitingIntersection.epsilonEquals(intersectionResult.getExitingIntersection(), 1.0e-10));

         expectedExitingIntersection.set(rayOrigin);
         rayDirection.set(-1.0, 0.0, 0.0);
         expectedExitingIntersection.setX(min.getX());
         intersectionResult = JOctoMapGeometryTools.rayBoxIntersection(min, max, rayOrigin, rayDirection);
         assertNotNull(intersectionResult);
         assertNull(intersectionResult.getEnteringIntersection());
         assertTrue(expectedExitingIntersection.epsilonEquals(intersectionResult.getExitingIntersection(), 1.0e-10));

         expectedExitingIntersection.set(rayOrigin);
         rayDirection.set(0.0, 1.0, 0.0);
         expectedExitingIntersection.setY(max.getY());
         intersectionResult = JOctoMapGeometryTools.rayBoxIntersection(min, max, rayOrigin, rayDirection);
         assertNotNull(intersectionResult);
         assertNull(intersectionResult.getEnteringIntersection());
         assertTrue(expectedExitingIntersection.epsilonEquals(intersectionResult.getExitingIntersection(), 1.0e-10));

         expectedExitingIntersection.set(rayOrigin);
         rayDirection.set(0.0, -1.0, 0.0);
         expectedExitingIntersection.setY(min.getY());
         intersectionResult = JOctoMapGeometryTools.rayBoxIntersection(min, max, rayOrigin, rayDirection);
         assertNotNull(intersectionResult);
         assertNull(intersectionResult.getEnteringIntersection());
         assertTrue(expectedExitingIntersection.epsilonEquals(intersectionResult.getExitingIntersection(), 1.0e-10));

         expectedExitingIntersection.set(rayOrigin);
         rayDirection.set(0.0, 0.0, 1.0);
         expectedExitingIntersection.setZ(max.getZ());
         intersectionResult = JOctoMapGeometryTools.rayBoxIntersection(min, max, rayOrigin, rayDirection);
         assertNotNull(intersectionResult);
         assertNull(intersectionResult.getEnteringIntersection());
         assertTrue(expectedExitingIntersection.epsilonEquals(intersectionResult.getExitingIntersection(), 1.0e-10));

         expectedExitingIntersection.set(rayOrigin);
         rayDirection.set(0.0, 0.0, -1.0);
         expectedExitingIntersection.setZ(min.getZ());
         intersectionResult = JOctoMapGeometryTools.rayBoxIntersection(min, max, rayOrigin, rayDirection);
         assertNotNull(intersectionResult);
         assertNull(intersectionResult.getEnteringIntersection());
         assertTrue(expectedExitingIntersection.epsilonEquals(intersectionResult.getExitingIntersection(), 1.0e-10));
      }
   }

   @Test
   public void testRayBoxIntersectionWithRayOriginatingFromOutsideTheBoxIntersecting() throws Exception
   {
      Random random = new Random(32452L);

      // Test with ray originating from inside the box
      for (int i = 0; i < 10000; i++)
      {
         Vector3d size = JOctoMapRandomTools.generateRandomVector3d(random, 10.0, 10.0, 10.0);
         size.absolute();
         Point3d min = JOctoMapRandomTools.generateRandomPoint3d(random, 10.0, 10.0, 10.0);
         Point3d max = new Point3d();
         max.add(min, size);

         Point3d rayOrigin = new Point3d();
         Vector3d rayDirection = new Vector3d();

         PointsOnEachBoxFace randomPointsOnEachBoxFace = generateRandomPointsOnEachBoxFace(random, min, max);
         BoxCorners boxCorners = extractBoxCorners(min, max);
         RayBoxIntersectionResult intersectionResult;
         Vector3d error = new Vector3d();

         for (Point3d expectedEnteringIntersection : randomPointsOnEachBoxFace.toArray())
         {
            for (Point3d expectedExitingIntersection : randomPointsOnEachBoxFace.toArray())
            {
               if (expectedEnteringIntersection.equals(expectedExitingIntersection))
                  continue;

               rayDirection.sub(expectedExitingIntersection, expectedEnteringIntersection);
               rayDirection.normalize();

               double s = JOctoMapRandomTools.generateRandomDouble(random, -10.0, 0.0);
               rayOrigin.scaleAdd(s, rayDirection, expectedEnteringIntersection);

               intersectionResult = JOctoMapGeometryTools.rayBoxIntersection(min, max, rayOrigin, rayDirection);
               assertNotNull(intersectionResult);
               error.sub(expectedEnteringIntersection, intersectionResult.getEnteringIntersection());
               assertTrue("Error: " + error, expectedEnteringIntersection.epsilonEquals(intersectionResult.getEnteringIntersection(), 1.0e-8));
               error.sub(expectedExitingIntersection, intersectionResult.getExitingIntersection());
               assertTrue("Error: " + error, expectedExitingIntersection.epsilonEquals(intersectionResult.getExitingIntersection(), 1.0e-8));
            }

            for (Point3d expectedExitingIntersection : boxCorners.toArray())
            {
               if (expectedEnteringIntersection.equals(expectedExitingIntersection))
                  continue;

               rayDirection.sub(expectedExitingIntersection, expectedEnteringIntersection);
               rayDirection.normalize();

               double s = JOctoMapRandomTools.generateRandomDouble(random, -10.0, 0.0);
               rayOrigin.scaleAdd(s, rayDirection, expectedEnteringIntersection);

               intersectionResult = JOctoMapGeometryTools.rayBoxIntersection(min, max, rayOrigin, rayDirection);
               boolean isIntersecting = true;
               if (Math.abs(rayDirection.getX()) < 1.0e-10)
               {
                  isIntersecting &= rayOrigin.getX() < max.getX() - 1.0e-10;
                  isIntersecting &= rayOrigin.getX() > min.getX() + 1.0e-10;
               }
               if (Math.abs(rayDirection.getY()) < 1.0e-10)
               {
                  isIntersecting &= rayOrigin.getY() < max.getY() - 1.0e-10;
                  isIntersecting &= rayOrigin.getY() > min.getY() + 1.0e-10;
               }
               if (Math.abs(rayDirection.getZ()) < 1.0e-10)
               {
                  isIntersecting &= rayOrigin.getZ() < max.getZ() - 1.0e-10;
                  isIntersecting &= rayOrigin.getZ() > min.getZ() + 1.0e-10;
               }
               if (isIntersecting)
               {
                  assertNotNull(intersectionResult);
                  error.sub(expectedEnteringIntersection, intersectionResult.getEnteringIntersection());
                  assertTrue("Error: " + error, expectedEnteringIntersection.epsilonEquals(intersectionResult.getEnteringIntersection(), 1.0e-8));
                  error.sub(expectedExitingIntersection, intersectionResult.getExitingIntersection());
                  assertTrue("Error: " + error, expectedExitingIntersection.epsilonEquals(intersectionResult.getExitingIntersection(), 1.0e-8));
               }
               else
               {
                  assertNull(intersectionResult);
               }
            }
         }

         for (Point3d expectedEnteringIntersection : boxCorners.toArray())
         {
            for (Point3d expectedExitingIntersection : boxCorners.toArray())
            {
               if (expectedEnteringIntersection.equals(expectedExitingIntersection))
                  continue;

               rayDirection.sub(expectedExitingIntersection, expectedEnteringIntersection);
               rayDirection.normalize();

               double s = JOctoMapRandomTools.generateRandomDouble(random, -10.0, 0.0);
               rayOrigin.scaleAdd(s, rayDirection, expectedEnteringIntersection);

               intersectionResult = JOctoMapGeometryTools.rayBoxIntersection(min, max, rayOrigin, rayDirection);
               boolean isIntersecting = true;
               if (Math.abs(rayDirection.getX()) < 1.0e-10)
               {
                  isIntersecting &= rayOrigin.getX() < max.getX() - 1.0e-10;
                  isIntersecting &= rayOrigin.getX() > min.getX() + 1.0e-10;
               }
               if (Math.abs(rayDirection.getY()) < 1.0e-10)
               {
                  isIntersecting &= rayOrigin.getY() < max.getY() - 1.0e-10;
                  isIntersecting &= rayOrigin.getY() > min.getY() + 1.0e-10;
               }
               if (Math.abs(rayDirection.getZ()) < 1.0e-10)
               {
                  isIntersecting &= rayOrigin.getZ() < max.getZ() - 1.0e-10;
                  isIntersecting &= rayOrigin.getZ() > min.getZ() + 1.0e-10;
               }
               if (isIntersecting)
               {
                  assertNotNull(intersectionResult);
                  error.sub(expectedEnteringIntersection, intersectionResult.getEnteringIntersection());
                  assertTrue("Error: " + error, expectedEnteringIntersection.epsilonEquals(intersectionResult.getEnteringIntersection(), 1.0e-8));
                  error.sub(expectedExitingIntersection, intersectionResult.getExitingIntersection());
                  assertTrue("Error: " + error, expectedExitingIntersection.epsilonEquals(intersectionResult.getExitingIntersection(), 1.0e-8));
               }
               else
               {
                  assertNull(intersectionResult);
               }
            }
         }
      }
   }

   @Test
   public void testRayBoxIntersectionWithRayOriginatingFromOutsideTheBoxGoingOppositeDirection() throws Exception
   {
      Random random = new Random(32452L);

      // Test with ray originating from inside the box
      for (int i = 0; i < 10000; i++)
      {
         Vector3d size = JOctoMapRandomTools.generateRandomVector3d(random, 10.0, 10.0, 10.0);
         size.absolute();
         Point3d min = JOctoMapRandomTools.generateRandomPoint3d(random, 10.0, 10.0, 10.0);
         Point3d max = new Point3d();
         max.add(min, size);

         Point3d rayOrigin = new Point3d();
         Vector3d rayDirection = new Vector3d();

         PointsOnEachBoxFace randomPointsOnEachBoxFace = generateRandomPointsOnEachBoxFace(random, min, max);
         BoxCorners boxCorners = extractBoxCorners(min, max);
         RayBoxIntersectionResult intersectionResult;

         for (Point3d expectedEnteringIntersection : randomPointsOnEachBoxFace.toArray())
         {
            for (Point3d expectedExitingIntersection : randomPointsOnEachBoxFace.toArray())
            {
               if (expectedEnteringIntersection.equals(expectedExitingIntersection))
                  continue;

               rayDirection.sub(expectedExitingIntersection, expectedEnteringIntersection);
               rayDirection.negate();
               rayDirection.normalize();

               double s = JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0);
               rayOrigin.scaleAdd(s, rayDirection, expectedEnteringIntersection);

               intersectionResult = JOctoMapGeometryTools.rayBoxIntersection(min, max, rayOrigin, rayDirection);
               assertNull(intersectionResult);
            }

            for (Point3d expectedExitingIntersection : boxCorners.toArray())
            {
               if (expectedEnteringIntersection.equals(expectedExitingIntersection))
                  continue;

               rayDirection.sub(expectedExitingIntersection, expectedEnteringIntersection);
               rayDirection.negate();
               rayDirection.normalize();

               double s = JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0);
               rayOrigin.scaleAdd(s, rayDirection, expectedEnteringIntersection);

               intersectionResult = JOctoMapGeometryTools.rayBoxIntersection(min, max, rayOrigin, rayDirection);
               assertNull(intersectionResult);
            }
         }

         for (Point3d expectedEnteringIntersection : boxCorners.toArray())
         {
            for (Point3d expectedExitingIntersection : boxCorners.toArray())
            {
               if (expectedEnteringIntersection.equals(expectedExitingIntersection))
                  continue;

               rayDirection.sub(expectedExitingIntersection, expectedEnteringIntersection);
               rayDirection.negate();
               rayDirection.normalize();

               double s = JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0);
               rayOrigin.scaleAdd(s, rayDirection, expectedEnteringIntersection);

               intersectionResult = JOctoMapGeometryTools.rayBoxIntersection(min, max, rayOrigin, rayDirection);
               assertNull(intersectionResult);
            }
         }
      }
   }

   public static PointsOnEachBoxFace generateRandomPointsOnEachBoxFace(Random random, Point3d min, Point3d max)
   {
      Point3d pointAtXMin = new Point3d();
      Point3d pointAtXMax = new Point3d();
      Point3d pointAtYMin = new Point3d();
      Point3d pointAtYMax = new Point3d();
      Point3d pointAtZMin = new Point3d();
      Point3d pointAtZMax = new Point3d();

      double xAlpha = JOctoMapRandomTools.generateRandomDouble(random, 0.0, 1.0);
      double yAlpha = JOctoMapRandomTools.generateRandomDouble(random, 0.0, 1.0);
      double zAlpha = JOctoMapRandomTools.generateRandomDouble(random, 0.0, 1.0);

      // Create a random point on the x-min face
      pointAtXMin.setX(min.getX());
      pointAtXMin.setY((1.0 - yAlpha) * min.getY() + yAlpha * max.getY());
      pointAtXMin.setZ((1.0 - zAlpha) * min.getZ() + zAlpha * max.getZ());

      // Create a random point on the x-max face
      pointAtXMax.setX(max.getX());
      pointAtXMax.setY((1.0 - yAlpha) * min.getY() + yAlpha * max.getY());
      pointAtXMax.setZ((1.0 - zAlpha) * min.getZ() + zAlpha * max.getZ());

      // Create a random point on the y-min face
      pointAtYMin.setX((1.0 - xAlpha) * min.getX() + xAlpha * max.getX());
      pointAtYMin.setY(min.getY());
      pointAtYMin.setZ((1.0 - zAlpha) * min.getZ() + zAlpha * max.getZ());

      // Create a random point on the y-max face
      pointAtYMax.setX((1.0 - xAlpha) * min.getX() + xAlpha * max.getX());
      pointAtYMax.setY(max.getY());
      pointAtYMax.setZ((1.0 - zAlpha) * min.getZ() + zAlpha * max.getZ());

      // Create a random point on the z-min face
      pointAtZMin.setX((1.0 - xAlpha) * min.getX() + xAlpha * max.getX());
      pointAtZMin.setY((1.0 - yAlpha) * min.getY() + yAlpha * max.getY());
      pointAtZMin.setZ(min.getZ());

      // Create a random point on the z-max face
      pointAtZMax.setX((1.0 - xAlpha) * min.getX() + xAlpha * max.getX());
      pointAtZMax.setY((1.0 - yAlpha) * min.getY() + yAlpha * max.getY());
      pointAtZMax.setZ(max.getZ());

      return new PointsOnEachBoxFace(pointAtXMin, pointAtXMax, pointAtYMin, pointAtYMax, pointAtZMin, pointAtZMax);
   }

   private static BoxCorners extractBoxCorners(Point3d min, Point3d max)
   {
      Point3d maxXYZ = new Point3d(max.getX(), max.getY(), max.getZ());
      Point3d maxXYminZ = new Point3d(max.getX(), max.getY(), min.getZ());
      Point3d maxXminYZ = new Point3d(max.getX(), min.getY(), min.getZ());
      Point3d maxXZminY = new Point3d(max.getX(), min.getY(), max.getZ());
      Point3d maxYZminX = new Point3d(min.getX(), max.getY(), max.getZ());
      Point3d maxYminXZ = new Point3d(min.getX(), max.getY(), min.getZ());
      Point3d minXYZ = new Point3d(min.getX(), min.getY(), min.getZ());
      Point3d maxZminXY = new Point3d(min.getX(), min.getY(), max.getZ());
      return new BoxCorners(maxXYZ, maxXYminZ, maxXminYZ, maxXZminY, maxYZminX, maxYminXZ, minXYZ, maxZminXY);
   }

   private static BoxFaceCenters extractFaceCenters(Point3d min, Point3d max)
   {
      Point3d boxCenter = new Point3d();
      boxCenter.interpolate(min, max, 0.5);

      Point3d centerAtXMin = new Point3d(boxCenter);
      centerAtXMin.setX(min.getX());
      Point3d centerAtXMax = new Point3d(boxCenter);
      centerAtXMax.setX(max.getX());
      Point3d centerAtYMin = new Point3d(boxCenter);
      centerAtYMin.setY(min.getY());
      Point3d centerAtYMax = new Point3d(boxCenter);
      centerAtYMax.setY(max.getY());
      Point3d centerAtZMin = new Point3d(boxCenter);
      centerAtZMin.setZ(min.getZ());
      Point3d centerAtZMax = new Point3d(boxCenter);
      centerAtZMax.setZ(max.getZ());
      return new BoxFaceCenters(centerAtXMin, centerAtXMax, centerAtYMin, centerAtYMax, centerAtZMin, centerAtZMax);
   }

   public static class PointsOnEachBoxFace
   {
      private final Point3d pointAtXMin;
      private final Point3d pointAtXMax;
      private final Point3d pointAtYMin;
      private final Point3d pointAtYMax;
      private final Point3d pointAtZMin;
      private final Point3d pointAtZMax;

      public PointsOnEachBoxFace(Point3d pointAtXMin, Point3d pointAtXMax, Point3d pointAtYMin, Point3d pointAtYMax, Point3d pointAtZMin, Point3d pointAtZMax)
      {
         this.pointAtXMin = pointAtXMin;
         this.pointAtXMax = pointAtXMax;
         this.pointAtYMin = pointAtYMin;
         this.pointAtYMax = pointAtYMax;
         this.pointAtZMin = pointAtZMin;
         this.pointAtZMax = pointAtZMax;
      }

      public Point3d[] toArray()
      {
         return new Point3d[] {pointAtXMin, pointAtXMax, pointAtYMin, pointAtYMax, pointAtZMin, pointAtZMax};
      }
   }

   private static class BoxCorners
   {
      private final Point3d maxXYZ;
      private final Point3d maxXYminZ;
      private final Point3d maxXminYZ;
      private final Point3d maxXZminY;
      private final Point3d maxYZminX;
      private final Point3d maxYminXZ;
      private final Point3d minXYZ;
      private final Point3d maxZminXY;

      public BoxCorners(Point3d maxXYZ, Point3d maxXYminZ, Point3d maxXminYZ, Point3d maxXZminY, Point3d maxYZminX, Point3d maxYminXZ, Point3d minXYZ,
            Point3d maxZminXY)
      {
         this.maxXYZ = maxXYZ;
         this.maxXYminZ = maxXYminZ;
         this.maxXminYZ = maxXminYZ;
         this.maxXZminY = maxXZminY;
         this.maxYZminX = maxYZminX;
         this.maxYminXZ = maxYminXZ;
         this.minXYZ = minXYZ;
         this.maxZminXY = maxZminXY;
      }

      public Point3d[] toArray()
      {
         return new Point3d[] {maxXYZ, maxXYminZ, maxXminYZ, maxXZminY, maxYZminX, maxYminXZ, minXYZ, maxZminXY};
      }
   }

   private static class BoxFaceCenters
   {
      private final Point3d centerAtXMin;
      private final Point3d centerAtXMax;
      private final Point3d centerAtYMin;
      private final Point3d centerAtYMax;
      private final Point3d centerAtZMin;
      private final Point3d centerAtZMax;

      public BoxFaceCenters(Point3d centerAtXMin, Point3d centerAtXMax, Point3d centerAtYMin, Point3d centerAtYMax, Point3d centerAtZMin, Point3d centerAtZMax)
      {
         this.centerAtXMin = centerAtXMin;
         this.centerAtXMax = centerAtXMax;
         this.centerAtYMin = centerAtYMin;
         this.centerAtYMax = centerAtYMax;
         this.centerAtZMin = centerAtZMin;
         this.centerAtZMax = centerAtZMax;
      }

      public Point3d[] toArray()
      {
         return new Point3d[] {centerAtXMin, centerAtXMax, centerAtYMin, centerAtYMax, centerAtZMin, centerAtZMax};
      }
   }
}
