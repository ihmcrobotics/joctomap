package us.ihmc.jOctoMap.boundingBox;

import static us.ihmc.robotics.Assert.assertNotNull;
import static us.ihmc.robotics.Assert.assertNull;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
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
   Point3D offset = new Point3D(-150.0, 150.0, 0.0);

   Point3D minCoordinate = new Point3D(0.0, 0.0, 0.0);
   Point3D maxCoordinate = new Point3D(100.0, 200.0, 100.0);
   OcTreeKey minKey = new OcTreeKey(32768, 32768, 32768);
   OcTreeKey maxKey = new OcTreeKey(36768, 40768, 36768);

   @Test
   public void isPointInRotatedBoundingBoxTest()
   {
      Point3D pointA = new Point3D(-200.0, 200.0, 50.0);
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
         Vector3D size = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         size.absolute();
         Point3D min = JOctoMapRandomTools.generateRandomPoint3D(random, 10.0, 10.0, 10.0);
         Point3D max = new Point3D();
         max.add(min, size);

         Vector3D offset = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 10.0);
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.getRotation().setToYawOrientation(yaw);
         transform.getTranslation().set(offset);

         OcTreeBoundingBoxWithCenterAndYaw bbx = new OcTreeBoundingBoxWithCenterAndYaw();
         bbx.setLocalMinMaxCoordinates(min, max);
         bbx.setOffset(offset);
         bbx.setYaw(yaw);

         Point3D rayOriginLocal = JOctoMapRandomTools.generateRandomPoint3D(random, min, max);

         if (i == 0)
            rayOriginLocal.interpolate(min, max, 0.5);

         Point3D rayOriginWorld = new Point3D();
         transform.transform(rayOriginLocal, rayOriginWorld);

         PointsOnEachBoxFace randomPointsOnEachBoxFace = JOctoMapGeometryToolsTest.generateRandomPointsOnEachBoxFace(random, min, max);
         RayBoxIntersectionResult intersectionResultInWorld;

         for (Point3D expectedExitingIntersectionLocal : randomPointsOnEachBoxFace.toArray())
         {
            Point3D expectedExitingIntersectionWorld = new Point3D();
            transform.transform(expectedExitingIntersectionLocal, expectedExitingIntersectionWorld);

            Vector3D rayDirectionLocal = new Vector3D();
            rayDirectionLocal.sub(expectedExitingIntersectionLocal, rayOriginLocal);
            rayDirectionLocal.normalize();
            Vector3D rayDirectionWorld = new Vector3D();
            transform.transform(rayDirectionLocal, rayDirectionWorld);

            intersectionResultInWorld = bbx.rayIntersection(rayOriginWorld, rayDirectionWorld);
            assertNotNull(intersectionResultInWorld);
            assertNull(intersectionResultInWorld.getEnteringIntersection());
            assertTrue(expectedExitingIntersectionWorld.epsilonEquals(intersectionResultInWorld.getExitingIntersection(), 1.0e-10));
         }
      }
   }
}
