package us.ihmc.octoMap;

import static org.junit.Assert.*;

import java.util.Map.Entry;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;

import org.junit.Test;

import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.OccupancyOcTreeNode;
import us.ihmc.octoMap.ocTree.OccupancyOcTree;
import us.ihmc.octoMap.pointCloud.PointCloud;

public class ChangedKeysTest
{

   @Test
   public void test()
   {
      fail("Not yet implemented");
   }

   @Test
   public void test1() throws Exception
   {
      OccupancyOcTree tree = new OccupancyOcTree(0.05);
      tree.enableChangeDetection(true);

      Point3d origin = new Point3d(0.01f, 0.01f, 0.02f);
      Point3d point_on_surface = new Point3d(4.01f, 0.01f, 0.01f);
      tree.insertRay(origin, point_on_surface);
      printChanges(tree);
      tree.updateNode(new Point3d(2.01f, 0.01f, 0.01f), 2.0f);
      printChanges(tree);
      tree.updateNode(new Point3d(2.01f, 0.01f, 0.01f), -2.0f);
      printChanges(tree);

      System.out.println("generating spherical scan at " + origin + " ...");
      Matrix3d yaw = new Matrix3d();
      Matrix3d pitch = new Matrix3d();

      for (int i = -100; i < 101; i++)
      {
         PointCloud cloud = new PointCloud();
         for (int j = -100; j < 101; j++)
         {
            Point3d rotated = new Point3d(point_on_surface);
            yaw.rotZ(Math.toRadians(i * 0.5));
            pitch.rotY(Math.toRadians(j * 0.5));
            Matrix3d rotation = new Matrix3d();
            rotation.mul(yaw, pitch);
            rotation.transform(rotated);
            cloud.add(rotated);
         }

         // insert in global coordinates:
         tree.insertPointCloud(cloud, origin);
      }

      printChanges(tree);

      System.out.println("done.");
   }

   void printChanges(OccupancyOcTree tree)
   {
      int changedOccupied = 0;
      int changedFree = 0;
      int actualOccupied = 0;
      int actualFree = 0;
      int missingChanged = 0;

      tree.expand();

      // iterate through the changed nodes
      for (Entry<OcTreeKeyReadOnly, Boolean> entrySet : tree.getChangedKeys().entrySet())
      {
         OccupancyOcTreeNode node = tree.search(entrySet.getKey());
         if (node != null)
         {
            if (tree.isNodeOccupied(node))
            {
               changedOccupied += 1;
            }
            else
            {
               changedFree += 1;
            }
         }
         else
         {
            missingChanged += 1;
         }
      }

      // iterate through the entire tree
      for (OccupancyOcTreeNode node : tree)
      {
         if (!node.hasAtLeastOneChild())
         {
            if (tree.isNodeOccupied(node))
            {
               actualOccupied += 1;
            }
            else
            {
               actualFree += 1;
            }
         }
      }

      System.out.println("change detection: " + changedOccupied + " occ; " + changedFree + " free; " + missingChanged + " missing");
      System.out.println("actual: " + actualOccupied + " occ; " + actualFree + " free; ");

      tree.prune();
   }

}
