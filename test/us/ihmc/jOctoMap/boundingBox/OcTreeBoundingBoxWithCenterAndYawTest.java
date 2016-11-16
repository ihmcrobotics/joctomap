package us.ihmc.jOctoMap.boundingBox;

import static org.junit.Assert.*;

import java.util.Random;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.tools.JOctoMapGeometryTools.RayBoxIntersectionResult;
import us.ihmc.jOctoMap.tools.JOctoMapGeometryToolsTest;
import us.ihmc.jOctoMap.tools.JOctoMapGeometryToolsTest.PointsOnEachBoxFace;
import us.ihmc.jOctoMap.tools.JOctoMapRandomTools;

public class OcTreeBoundingBoxWithCenterAndYawTest
{
   double resolution = 0.025;
   int treeDepth = 16;
   double yaw = Math.PI / 2;
   Point3d offset = new Point3d(-150.0, 150.0, 0.0);

   Point3d minCoordinate = new Point3d(0.0, 0.0, 0.0);
   Point3d maxCoordinate = new Point3d(100.0, 200.0, 100.0);
   OcTreeKey minKey = new OcTreeKey(32768, 32768, 32768);
   OcTreeKey maxKey = new OcTreeKey(36768, 40768, 36768);

   @Test
   public void isPointInRotatedBoundingBoxTest()
   {
      Point3d pointA = new Point3d(-200.0, 200.0, 50.0);
      OcTreeBoundingBoxWithCenterAndYaw boundingBox = new OcTreeBoundingBoxWithCenterAndYaw(minCoordinate, maxCoordinate, resolution, treeDepth);
      boundingBox.setYaw(yaw);
      boundingBox.setOffset(offset, resolution, treeDepth);

      assertTrue(boundingBox.isInBoundingBox(pointA));
   }

   @Test
   public void isKeyInSimpleBoundingBoxTest()
   {
      OcTreeKey keyA = new OcTreeKey(34000, 39000, 34000);
      OcTreeSimpleBoundingBox boundingBox = new OcTreeSimpleBoundingBox(minKey, maxKey);
      boundingBox.update(resolution, treeDepth);

      assertTrue(boundingBox.isInBoundingBox(keyA));
   }

   @Test
   public void isKeyInRotatedBoundingBoxTest()
   {
      //OcTreeKey key = new OcTreeKey(29940, 35596, 34768); //45 degrees yaw 
      OcTreeKey key = new OcTreeKey(29304, 34768, 34768); //60 degrees yaw 
      OcTreeSimpleBoundingBox simpleBoundingBox = new OcTreeSimpleBoundingBox(minKey, maxKey);
      simpleBoundingBox.update(resolution, treeDepth);

      OcTreeBoundingBoxWithCenterAndYaw boundingBox = new OcTreeBoundingBoxWithCenterAndYaw(simpleBoundingBox, resolution, treeDepth);
      boundingBox.setYaw(Math.PI / 3);

      assertTrue(boundingBox.isInBoundingBox(key));
   }

   @Test
   public void isKeyInRotatedAndOffsetBoundingBoxTest()
   {
      OcTreeKey key = new OcTreeKey(23940, 41596, 34768); //45 degrees yaw and offset (-150.0, 150.0, 0.0)
      OcTreeSimpleBoundingBox simpleBoundingBox = new OcTreeSimpleBoundingBox(minKey, maxKey);
      simpleBoundingBox.update(resolution, treeDepth);

      OcTreeBoundingBoxWithCenterAndYaw boundingBox = new OcTreeBoundingBoxWithCenterAndYaw(simpleBoundingBox, resolution, treeDepth);
      boundingBox.setYaw(Math.PI / 4);
      boundingBox.setOffset(offset, resolution, treeDepth);

      assertTrue(boundingBox.isInBoundingBox(key));
   }

   @Test
   public void testRayIntersectionWithRayOriginatingFromInside() throws Exception
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

         Vector3d offset = JOctoMapRandomTools.generateRandomVector3d(random, 10.0);
         double yaw = JOctoMapRandomTools.generateRandomDouble(random, Math.PI);
         Matrix4d transform = new Matrix4d();
         transform.rotZ(yaw);
         transform.setTranslation(offset);

         OcTreeBoundingBoxWithCenterAndYaw bbx = new OcTreeBoundingBoxWithCenterAndYaw();
         bbx.setLocalMinMaxCoordinates(min, max);
         bbx.setOffset(offset);
         bbx.setYaw(yaw);

         Point3d rayOriginLocal = JOctoMapRandomTools.generateRandomPoint3d(random, min, max);

         if (i == 0)
            rayOriginLocal.interpolate(min, max, 0.5);

         Point3d rayOriginWorld = new Point3d();
         transform.transform(rayOriginLocal, rayOriginWorld);

         PointsOnEachBoxFace randomPointsOnEachBoxFace = JOctoMapGeometryToolsTest.generateRandomPointsOnEachBoxFace(random, min, max);
         RayBoxIntersectionResult intersectionResultInWorld;

         for (Point3d expectedExitingIntersectionLocal : randomPointsOnEachBoxFace.toArray())
         {
            Point3d expectedExitingIntersectionWorld = new Point3d();
            transform.transform(expectedExitingIntersectionLocal, expectedExitingIntersectionWorld);

            Vector3d rayDirectionLocal = new Vector3d();
            rayDirectionLocal.sub(expectedExitingIntersectionLocal, rayOriginLocal);
            rayDirectionLocal.normalize();
            Vector3d rayDirectionWorld = new Vector3d();
            transform.transform(rayDirectionLocal, rayDirectionWorld);

            intersectionResultInWorld = bbx.rayIntersection(rayOriginWorld, rayDirectionWorld);
            assertNotNull(intersectionResultInWorld);
            assertNull(intersectionResultInWorld.getEnteringIntersection());
            assertTrue(expectedExitingIntersectionWorld.epsilonEquals(intersectionResultInWorld.getExitingIntersection(), 1.0e-10));
         }
      }
   }
}
