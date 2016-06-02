package us.ihmc.octoMap.tools;

import static org.junit.Assert.*;

import java.util.Random;

import javax.vecmath.Point3d;

import org.junit.Test;

import us.ihmc.octoMap.iterators.OcTreeSuperNode;
import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.node.OccupancyOcTreeNode;
import us.ihmc.octoMap.ocTree.OcTree;

public class OcTreeSearchToolsTest
{
   @Test
   public void testSearchWithKey() throws Exception
   {
      Random random = new Random(54564L);
      double resolution = 0.02;
      OcTree ocTree = new OcTree(resolution);
      int treeDepth = ocTree.getTreeDepth();

      int keyMax = OcTreeKeyTools.computeMaximumKey(treeDepth);

      for (int i = 0; i < 100000; i++)
      {
         OcTreeKey randomKey = new OcTreeKey(random, keyMax);
         ocTree.updateNode(randomKey, true);
      }

      for (OcTreeSuperNode<OccupancyOcTreeNode> superNode : ocTree)
      {
         int nodeDepth = superNode.getDepth();
         OcTreeKey key = superNode.getKey();
         OccupancyOcTreeNode expectedNode = superNode.getNode();

         if (nodeDepth == treeDepth)
         {
            OccupancyOcTreeNode actualNode = ocTree.search(key);
            assertEquals(expectedNode, actualNode);
         }

         OccupancyOcTreeNode actualNode = ocTree.search(key, nodeDepth);
         assertEquals(expectedNode, actualNode);
      }
   }

   @Test
   public void testSearchWithCoordinate() throws Exception
   {
      Random random = new Random(54564L);
      double resolution = 0.02;
      OcTree ocTree = new OcTree(resolution);
      int treeDepth = ocTree.getTreeDepth();

      int keyMax = OcTreeKeyTools.computeMaximumKey(treeDepth);

      for (int i = 0; i < 100000; i++)
      {
         OcTreeKey randomKey = new OcTreeKey(random, keyMax);
         ocTree.updateNode(randomKey, true);
      }

      for (OcTreeSuperNode<OccupancyOcTreeNode> superNode : ocTree)
      {
         int nodeDepth = superNode.getDepth();
         OcTreeKey key = superNode.getKey();
         Point3d nodeCoordinate = OcTreeKeyConversionTools.keyToCoordinate(key, nodeDepth, resolution, treeDepth);
         OccupancyOcTreeNode expectedNode = superNode.getNode();

         if (nodeDepth == treeDepth)
         {
            OccupancyOcTreeNode actualNode = ocTree.search(nodeCoordinate);
            assertEquals(expectedNode, actualNode);
         }

         OccupancyOcTreeNode actualNode = ocTree.search(nodeCoordinate, nodeDepth);
         assertEquals(expectedNode, actualNode);
      }
   }

   @Test
   public void test()
   {
      fail("Not yet implemented");
   }

}
