package us.ihmc.jOctoMap.tools;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

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
   public void testGetAxisAngleFromFirstToSecondVector() throws Exception
   {
      Random random = new Random(1176L);
      for (int i = 0; i < 1000; i++)
      {
         Vector3d firstVector = JOctoMapRandomTools.generateRandomVector3d(random, JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));
         double expectedAngle = JOctoMapRandomTools.generateRandomDouble(random, 0.0, Math.PI);
         Vector3d expectedAxis = JOctoMapRandomTools.generateRandomOrthogonalVector3d(random, firstVector, true);
         AxisAngle4d expectedAxisAngle = new AxisAngle4d(expectedAxis, expectedAngle);
         Matrix3d rotationMatrix = new Matrix3d();
         rotationMatrix.set(expectedAxisAngle);

         Vector3d secondVector = new Vector3d();
         rotationMatrix.transform(firstVector, secondVector);
         secondVector.scale(JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));

         AxisAngle4d actualAxisAngle = new AxisAngle4d();
         JOctoMapGeometryTools.getAxisAngleFromFirstToSecondVector(firstVector, secondVector, actualAxisAngle);

         Vector3d actualAxis = new Vector3d(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

         assertEquals(1.0, actualAxis.length(), 1.0e-12);
         assertEquals(0.0, actualAxis.dot(firstVector), 1.0e-12);
         assertEquals(0.0, actualAxis.dot(secondVector), 1.0e-12);

         assertEquals(0.0, expectedAxis.dot(firstVector), 1.0e-12);
         assertEquals(0.0, expectedAxis.dot(secondVector), 1.0e-12);

         if (actualAxisAngle.getAngle() * expectedAxisAngle.getAngle() < 0.0)
         {
            expectedAxis.negate();
            expectedAngle = -expectedAngle;
            expectedAxisAngle.set(expectedAxis, expectedAngle);
         }

         try
         {
            assertTrue(expectedAxisAngle.epsilonEquals(actualAxisAngle, 1.0e-12));
         }
         catch (AssertionError e)
         {
            throw new AssertionError("expected:\n<" + expectedAxisAngle + ">\n but was:\n<" + actualAxisAngle + ">");
         }
      }

      // Test close to 0.0
      for (int i = 0; i < 1000; i++)
      {
         Vector3d firstVector = JOctoMapRandomTools.generateRandomVector3d(random, JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));
         double expectedAngle = JOctoMapRandomTools.generateRandomDouble(random, 0.0001, 0.001);
         if (random.nextBoolean())
            expectedAngle = -expectedAngle;
         Vector3d expectedAxis = JOctoMapRandomTools.generateRandomOrthogonalVector3d(random, firstVector, true);
         AxisAngle4d expectedAxisAngle = new AxisAngle4d(expectedAxis, expectedAngle);
         Matrix3d rotationMatrix = new Matrix3d();
         rotationMatrix.set(expectedAxisAngle);

         Vector3d secondVector = new Vector3d();
         rotationMatrix.transform(firstVector, secondVector);
         secondVector.scale(JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));

         AxisAngle4d actualAxisAngle = new AxisAngle4d();
         JOctoMapGeometryTools.getAxisAngleFromFirstToSecondVector(firstVector, secondVector, actualAxisAngle);

         Vector3d actualAxis = new Vector3d(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

         assertEquals(1.0, actualAxis.length(), 1.0e-12);
         // Can not be as accurate as we get closer to 0.0
         assertEquals(0.0, actualAxis.dot(firstVector), 1.0e-9);
         assertEquals(0.0, actualAxis.dot(secondVector), 1.0e-9);

         assertEquals(0.0, expectedAxis.dot(firstVector), 1.0e-12);
         assertEquals(0.0, expectedAxis.dot(secondVector), 1.0e-12);

         if (actualAxisAngle.getAngle() * expectedAxisAngle.getAngle() < 0.0)
         {
            expectedAxis.negate();
            expectedAngle = -expectedAngle;
            expectedAxisAngle.set(expectedAxis, expectedAngle);
         }

         try
         {
            // Can not be as accurate as we get closer to 0.0
            assertTrue(expectedAxisAngle.epsilonEquals(actualAxisAngle, 1.0e-9));
         }
         catch (AssertionError e)
         {
            throw new AssertionError("expected:\n<" + expectedAxisAngle + ">\n but was:\n<" + actualAxisAngle + ">");
         }
      }

      // Test close to Math.PI
      for (int i = 0; i < 1000; i++)
      {
         Vector3d referenceNormal = JOctoMapRandomTools.generateRandomVector3d(random, JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));
         double expectedAngle = JOctoMapRandomTools.generateRandomDouble(random, 0.00001, 0.001);
         if (random.nextBoolean())
            expectedAngle = -expectedAngle;
         expectedAngle += Math.PI;
         Vector3d expectedAxis = JOctoMapRandomTools.generateRandomOrthogonalVector3d(random, referenceNormal, true);
         AxisAngle4d expectedAxisAngle = new AxisAngle4d(expectedAxis, expectedAngle);
         Matrix3d rotationMatrix = new Matrix3d();
         rotationMatrix.set(expectedAxisAngle);

         Vector3d rotatedNormal = new Vector3d();
         rotationMatrix.transform(referenceNormal, rotatedNormal);
         rotatedNormal.scale(JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));

         AxisAngle4d actualAxisAngle = new AxisAngle4d();
         JOctoMapGeometryTools.getAxisAngleFromFirstToSecondVector(referenceNormal, rotatedNormal, actualAxisAngle);

         Vector3d actualAxis = new Vector3d(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

         assertEquals(1.0, actualAxis.length(), 1.0e-12);
         // Can not be as accurate as we get closer to Math.PI
         assertEquals(0.0, actualAxis.dot(referenceNormal), 1.0e-9);
         assertEquals(0.0, actualAxis.dot(rotatedNormal), 1.0e-9);

         assertEquals(0.0, expectedAxis.dot(referenceNormal), 1.0e-12);
         assertEquals(0.0, expectedAxis.dot(rotatedNormal), 1.0e-12);
         if (actualAxisAngle.getAngle() * expectedAxisAngle.getAngle() < 0.0)
         {
            expectedAxis.negate();
            expectedAngle = -expectedAngle;
            expectedAxisAngle.set(expectedAxis, expectedAngle);
         }

         if (Math.abs(expectedAxisAngle.getAngle() + actualAxisAngle.getAngle()) > 2.0 * Math.PI - 0.1)
         {
            // Here the sign of the axis does not matter.
            if (expectedAxis.dot(actualAxis) < 0.0)
               expectedAxis.negate();
            // Can not be as accurate as we get closer to Math.PI
            assertTrue(expectedAxis.epsilonEquals(actualAxis, 1.0e-9));
         }
         else
         {
            try
            {
               assertTrue(expectedAxisAngle.epsilonEquals(actualAxisAngle, 1.0e-12));
            }
            catch (AssertionError e)
            {
               throw new AssertionError("expected:\n<" + expectedAxisAngle + ">\n but was:\n<" + actualAxisAngle + ">");
            }
         }
      }

      // Test exactly at 0.0
      for (int i = 0; i < 1000; i++)
      {
         Vector3d referenceNormal = JOctoMapRandomTools.generateRandomVector3d(random, JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));
         Vector3d rotatedNormal = new Vector3d(referenceNormal);
         rotatedNormal.scale(JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));
         double expectedAngle = 0.0;
         Vector3d expectedAxis = new Vector3d(1.0, 0.0, 0.0);
         AxisAngle4d expectedAxisAngle = new AxisAngle4d(expectedAxis, expectedAngle);

         AxisAngle4d actualAxisAngle = new AxisAngle4d();
         JOctoMapGeometryTools.getAxisAngleFromFirstToSecondVector(referenceNormal, rotatedNormal, actualAxisAngle);

         Vector3d actualAxis = new Vector3d(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

         assertEquals(1.0, actualAxis.length(), 1.0e-12);

         if (actualAxisAngle.getAngle() * expectedAxisAngle.getAngle() < 0.0)
         {
            expectedAxis.negate();
            expectedAngle = -expectedAngle;
            expectedAxisAngle.set(expectedAxis, expectedAngle);
         }

         try
         {
            assertTrue(expectedAxisAngle.epsilonEquals(actualAxisAngle, 1.0e-12));
         }
         catch (AssertionError e)
         {
            throw new AssertionError("expected:\n<" + expectedAxisAngle + ">\n but was:\n<" + actualAxisAngle + ">");
         }
      }

      // Test exactly at Math.PI
      for (int i = 0; i < 1000; i++)
      {
         Vector3d referenceNormal = JOctoMapRandomTools.generateRandomVector3d(random, JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));
         Vector3d rotatedNormal = new Vector3d();
         rotatedNormal.negate(referenceNormal);
         rotatedNormal.scale(JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));
         double expectedAngle = Math.PI;
         Vector3d expectedAxis = new Vector3d(1.0, 0.0, 0.0);
         AxisAngle4d expectedAxisAngle = new AxisAngle4d(expectedAxis, expectedAngle);

         AxisAngle4d actualAxisAngle = new AxisAngle4d();
         JOctoMapGeometryTools.getAxisAngleFromFirstToSecondVector(referenceNormal, rotatedNormal, actualAxisAngle);

         Vector3d actualAxis = new Vector3d(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

         assertEquals(1.0, actualAxis.length(), 1.0e-12);

         if (actualAxisAngle.getAngle() * expectedAxisAngle.getAngle() < 0.0)
         {
            expectedAxis.negate();
            expectedAngle = -expectedAngle;
            expectedAxisAngle.set(expectedAxis, expectedAngle);
         }

         try
         {
            assertTrue(expectedAxisAngle.epsilonEquals(actualAxisAngle, 1.0e-12));
         }
         catch (AssertionError e)
         {
            throw new AssertionError("expected:\n<" + expectedAxisAngle + ">\n but was:\n<" + actualAxisAngle + ">");
         }
      }
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
