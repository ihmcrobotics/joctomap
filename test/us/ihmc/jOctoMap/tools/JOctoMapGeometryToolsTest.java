package us.ihmc.jOctoMap.tools;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.axisAngle.AxisAngle;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.tuple3D.Point3D;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.jOctoMap.tools.JOctoMapGeometryTools.RayBoxIntersectionResult;

public class JOctoMapGeometryToolsTest
{
   @Test
   public void testGetAxisAngleFromFirstToSecondVector() throws Exception
   {
      Random random = new Random(1176L);
      for (int i = 0; i < 1000; i++)
      {
         Vector3D firstVector = JOctoMapRandomTools.generateRandomVector3D(random, JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));
         double expectedAngle = JOctoMapRandomTools.generateRandomDouble(random, 0.0, Math.PI);
         Vector3D expectedAxis = JOctoMapRandomTools.generateRandomOrthogonalVector3D(random, firstVector, true);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);
         RotationMatrix rotationMatrix = new RotationMatrix(expectedAxisAngle);

         Vector3D secondVector = new Vector3D();
         rotationMatrix.transform(firstVector, secondVector);
         secondVector.scale(JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));

         AxisAngle actualAxisAngle = new AxisAngle();
         JOctoMapGeometryTools.getAxisAngleFromFirstToSecondVector(firstVector, secondVector, actualAxisAngle);

         Vector3D actualAxis = new Vector3D(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

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
         Vector3D firstVector = JOctoMapRandomTools.generateRandomVector3D(random, JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));
         double expectedAngle = JOctoMapRandomTools.generateRandomDouble(random, 0.0001, 0.001);
         if (random.nextBoolean())
            expectedAngle = -expectedAngle;
         Vector3D expectedAxis = JOctoMapRandomTools.generateRandomOrthogonalVector3D(random, firstVector, true);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(expectedAxisAngle);

         Vector3D secondVector = new Vector3D();
         rotationMatrix.transform(firstVector, secondVector);
         secondVector.scale(JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));

         AxisAngle actualAxisAngle = new AxisAngle();
         JOctoMapGeometryTools.getAxisAngleFromFirstToSecondVector(firstVector, secondVector, actualAxisAngle);

         Vector3D actualAxis = new Vector3D(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

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
         Vector3D referenceNormal = JOctoMapRandomTools.generateRandomVector3D(random, JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));
         double expectedAngle = JOctoMapRandomTools.generateRandomDouble(random, 0.00001, 0.001);
         if (random.nextBoolean())
            expectedAngle = -expectedAngle;
         expectedAngle += Math.PI;
         Vector3D expectedAxis = JOctoMapRandomTools.generateRandomOrthogonalVector3D(random, referenceNormal, true);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(expectedAxisAngle);

         Vector3D rotatedNormal = new Vector3D();
         rotationMatrix.transform(referenceNormal, rotatedNormal);
         rotatedNormal.scale(JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));

         AxisAngle actualAxisAngle = new AxisAngle();
         JOctoMapGeometryTools.getAxisAngleFromFirstToSecondVector(referenceNormal, rotatedNormal, actualAxisAngle);

         Vector3D actualAxis = new Vector3D(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

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
         Vector3D referenceNormal = JOctoMapRandomTools.generateRandomVector3D(random, JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));
         Vector3D rotatedNormal = new Vector3D(referenceNormal);
         rotatedNormal.scale(JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));
         double expectedAngle = 0.0;
         Vector3D expectedAxis = new Vector3D(1.0, 0.0, 0.0);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);

         AxisAngle actualAxisAngle = new AxisAngle();
         JOctoMapGeometryTools.getAxisAngleFromFirstToSecondVector(referenceNormal, rotatedNormal, actualAxisAngle);

         Vector3D actualAxis = new Vector3D(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

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
         Vector3D referenceNormal = JOctoMapRandomTools.generateRandomVector3D(random, JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));
         Vector3D rotatedNormal = new Vector3D();
         rotatedNormal.setAndNegate(referenceNormal);
         rotatedNormal.scale(JOctoMapRandomTools.generateRandomDouble(random, 0.0, 10.0));
         double expectedAngle = Math.PI;
         Vector3D expectedAxis = new Vector3D(1.0, 0.0, 0.0);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);

         AxisAngle actualAxisAngle = new AxisAngle();
         JOctoMapGeometryTools.getAxisAngleFromFirstToSecondVector(referenceNormal, rotatedNormal, actualAxisAngle);

         Vector3D actualAxis = new Vector3D(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

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
         Vector3D size = JOctoMapRandomTools.generateRandomVector3D(random, 10.0, 10.0, 10.0);
         size.absolute();
         Point3D min = JOctoMapRandomTools.generateRandomPoint3D(random, 10.0, 10.0, 10.0);
         Point3D max = new Point3D();
         max.add(min, size);

         Point3D rayOrigin = JOctoMapRandomTools.generateRandomPoint3D(random, min, max);

         if (i == 0)
            rayOrigin.interpolate(min, max, 0.5);

         Vector3D rayDirection = new Vector3D();

         PointsOnEachBoxFace randomPointsOnEachBoxFace = generateRandomPointsOnEachBoxFace(random, min, max);
         RayBoxIntersectionResult intersectionResult;

         for (Point3D expectedExitingIntersection : randomPointsOnEachBoxFace.toArray())
         {
            rayDirection.sub(expectedExitingIntersection, rayOrigin);
            rayDirection.normalize();

            intersectionResult = JOctoMapGeometryTools.rayBoxIntersection(min, max, rayOrigin, rayDirection);
            assertNotNull(intersectionResult);
            assertNull(intersectionResult.getEnteringIntersection());
            assertTrue(expectedExitingIntersection.epsilonEquals(intersectionResult.getExitingIntersection(), 1.0e-10));
         }

         BoxCorners boxCorners = extractBoxCorners(min, max);

         for (Point3D expectedExitingIntersection : boxCorners.toArray())
         {
            rayDirection.sub(expectedExitingIntersection, rayOrigin);
            rayDirection.normalize();

            intersectionResult = JOctoMapGeometryTools.rayBoxIntersection(min, max, rayOrigin, rayDirection);
            assertNotNull(intersectionResult);
            assertNull(intersectionResult.getEnteringIntersection());
            assertTrue(expectedExitingIntersection.epsilonEquals(intersectionResult.getExitingIntersection(), 1.0e-10));
         }

         BoxFaceCenters boxFaceCenters = extractFaceCenters(min, max);

         for (Point3D expectedExitingIntersection : boxFaceCenters.toArray())
         {
            rayDirection.sub(expectedExitingIntersection, rayOrigin);
            rayDirection.normalize();

            intersectionResult = JOctoMapGeometryTools.rayBoxIntersection(min, max, rayOrigin, rayDirection);
            assertNotNull(intersectionResult);
            assertNull(intersectionResult.getEnteringIntersection());
            assertTrue(expectedExitingIntersection.epsilonEquals(intersectionResult.getExitingIntersection(), 1.0e-10));
         }

         Point3D expectedExitingIntersection = new Point3D();

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
         Vector3D size = JOctoMapRandomTools.generateRandomVector3D(random, 10.0, 10.0, 10.0);
         size.absolute();
         Point3D min = JOctoMapRandomTools.generateRandomPoint3D(random, 10.0, 10.0, 10.0);
         Point3D max = new Point3D();
         max.add(min, size);

         Point3D rayOrigin = new Point3D();
         Vector3D rayDirection = new Vector3D();

         PointsOnEachBoxFace randomPointsOnEachBoxFace = generateRandomPointsOnEachBoxFace(random, min, max);
         BoxCorners boxCorners = extractBoxCorners(min, max);
         RayBoxIntersectionResult intersectionResult;
         Vector3D error = new Vector3D();

         for (Point3D expectedEnteringIntersection : randomPointsOnEachBoxFace.toArray())
         {
            for (Point3D expectedExitingIntersection : randomPointsOnEachBoxFace.toArray())
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

            for (Point3D expectedExitingIntersection : boxCorners.toArray())
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

         for (Point3D expectedEnteringIntersection : boxCorners.toArray())
         {
            for (Point3D expectedExitingIntersection : boxCorners.toArray())
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
         Vector3D size = JOctoMapRandomTools.generateRandomVector3D(random, 10.0, 10.0, 10.0);
         size.absolute();
         Point3D min = JOctoMapRandomTools.generateRandomPoint3D(random, 10.0, 10.0, 10.0);
         Point3D max = new Point3D();
         max.add(min, size);

         Point3D rayOrigin = new Point3D();
         Vector3D rayDirection = new Vector3D();

         PointsOnEachBoxFace randomPointsOnEachBoxFace = generateRandomPointsOnEachBoxFace(random, min, max);
         BoxCorners boxCorners = extractBoxCorners(min, max);
         RayBoxIntersectionResult intersectionResult;

         for (Point3D expectedEnteringIntersection : randomPointsOnEachBoxFace.toArray())
         {
            for (Point3D expectedExitingIntersection : randomPointsOnEachBoxFace.toArray())
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

            for (Point3D expectedExitingIntersection : boxCorners.toArray())
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

         for (Point3D expectedEnteringIntersection : boxCorners.toArray())
         {
            for (Point3D expectedExitingIntersection : boxCorners.toArray())
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

   public static PointsOnEachBoxFace generateRandomPointsOnEachBoxFace(Random random, Point3D min, Point3D max)
   {
      Point3D pointAtXMin = new Point3D();
      Point3D pointAtXMax = new Point3D();
      Point3D pointAtYMin = new Point3D();
      Point3D pointAtYMax = new Point3D();
      Point3D pointAtZMin = new Point3D();
      Point3D pointAtZMax = new Point3D();

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

   private static BoxCorners extractBoxCorners(Point3D min, Point3D max)
   {
      Point3D maxXYZ = new Point3D(max.getX(), max.getY(), max.getZ());
      Point3D maxXYminZ = new Point3D(max.getX(), max.getY(), min.getZ());
      Point3D maxXminYZ = new Point3D(max.getX(), min.getY(), min.getZ());
      Point3D maxXZminY = new Point3D(max.getX(), min.getY(), max.getZ());
      Point3D maxYZminX = new Point3D(min.getX(), max.getY(), max.getZ());
      Point3D maxYminXZ = new Point3D(min.getX(), max.getY(), min.getZ());
      Point3D minXYZ = new Point3D(min.getX(), min.getY(), min.getZ());
      Point3D maxZminXY = new Point3D(min.getX(), min.getY(), max.getZ());
      return new BoxCorners(maxXYZ, maxXYminZ, maxXminYZ, maxXZminY, maxYZminX, maxYminXZ, minXYZ, maxZminXY);
   }

   private static BoxFaceCenters extractFaceCenters(Point3D min, Point3D max)
   {
      Point3D boxCenter = new Point3D();
      boxCenter.interpolate(min, max, 0.5);

      Point3D centerAtXMin = new Point3D(boxCenter);
      centerAtXMin.setX(min.getX());
      Point3D centerAtXMax = new Point3D(boxCenter);
      centerAtXMax.setX(max.getX());
      Point3D centerAtYMin = new Point3D(boxCenter);
      centerAtYMin.setY(min.getY());
      Point3D centerAtYMax = new Point3D(boxCenter);
      centerAtYMax.setY(max.getY());
      Point3D centerAtZMin = new Point3D(boxCenter);
      centerAtZMin.setZ(min.getZ());
      Point3D centerAtZMax = new Point3D(boxCenter);
      centerAtZMax.setZ(max.getZ());
      return new BoxFaceCenters(centerAtXMin, centerAtXMax, centerAtYMin, centerAtYMax, centerAtZMin, centerAtZMax);
   }

   public static class PointsOnEachBoxFace
   {
      private final Point3D pointAtXMin;
      private final Point3D pointAtXMax;
      private final Point3D pointAtYMin;
      private final Point3D pointAtYMax;
      private final Point3D pointAtZMin;
      private final Point3D pointAtZMax;

      public PointsOnEachBoxFace(Point3D pointAtXMin, Point3D pointAtXMax, Point3D pointAtYMin, Point3D pointAtYMax, Point3D pointAtZMin, Point3D pointAtZMax)
      {
         this.pointAtXMin = pointAtXMin;
         this.pointAtXMax = pointAtXMax;
         this.pointAtYMin = pointAtYMin;
         this.pointAtYMax = pointAtYMax;
         this.pointAtZMin = pointAtZMin;
         this.pointAtZMax = pointAtZMax;
      }

      public Point3D[] toArray()
      {
         return new Point3D[] {pointAtXMin, pointAtXMax, pointAtYMin, pointAtYMax, pointAtZMin, pointAtZMax};
      }
   }

   private static class BoxCorners
   {
      private final Point3D maxXYZ;
      private final Point3D maxXYminZ;
      private final Point3D maxXminYZ;
      private final Point3D maxXZminY;
      private final Point3D maxYZminX;
      private final Point3D maxYminXZ;
      private final Point3D minXYZ;
      private final Point3D maxZminXY;

      public BoxCorners(Point3D maxXYZ, Point3D maxXYminZ, Point3D maxXminYZ, Point3D maxXZminY, Point3D maxYZminX, Point3D maxYminXZ, Point3D minXYZ,
            Point3D maxZminXY)
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

      public Point3D[] toArray()
      {
         return new Point3D[] {maxXYZ, maxXYminZ, maxXminYZ, maxXZminY, maxYZminX, maxYminXZ, minXYZ, maxZminXY};
      }
   }

   private static class BoxFaceCenters
   {
      private final Point3D centerAtXMin;
      private final Point3D centerAtXMax;
      private final Point3D centerAtYMin;
      private final Point3D centerAtYMax;
      private final Point3D centerAtZMin;
      private final Point3D centerAtZMax;

      public BoxFaceCenters(Point3D centerAtXMin, Point3D centerAtXMax, Point3D centerAtYMin, Point3D centerAtYMax, Point3D centerAtZMin, Point3D centerAtZMax)
      {
         this.centerAtXMin = centerAtXMin;
         this.centerAtXMax = centerAtXMax;
         this.centerAtYMin = centerAtYMin;
         this.centerAtYMax = centerAtYMax;
         this.centerAtZMin = centerAtZMin;
         this.centerAtZMax = centerAtZMax;
      }

      public Point3D[] toArray()
      {
         return new Point3D[] {centerAtXMin, centerAtXMax, centerAtYMin, centerAtYMax, centerAtZMin, centerAtZMax};
      }
   }
}
