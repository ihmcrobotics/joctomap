package us.ihmc.jOctoMap.tools;

import static org.junit.Assert.*;

import java.util.Random;

import javax.vecmath.Point3d;

import org.junit.Test;

import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.node.OccupancyOcTreeNode;
import us.ihmc.jOctoMap.ocTree.OccupancyOcTree;
import us.ihmc.jOctoMap.tools.OcTreeKeyConversionTools;

public class OcTreeSearchToolsTest
{
   @Test
   public void testSearchWithKey() throws Exception
   {
      Random random = new Random(54564L);
      double resolution = 0.02;
      OccupancyOcTree ocTree = new OccupancyOcTree(resolution);
      int treeDepth = ocTree.getTreeDepth();

      for (int i = 0; i < 100000; i++)
      {
         OcTreeKey randomKey = new OcTreeKey(random, treeDepth);
         ocTree.updateNode(randomKey, true);
      }

      for (OccupancyOcTreeNode node : ocTree)
      {
         int nodeDepth = node.getDepth();
         OcTreeKey key = new OcTreeKey();
         node.getKey(key);
         OccupancyOcTreeNode expectedNode = node;

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
      OccupancyOcTree ocTree = new OccupancyOcTree(resolution);
      int treeDepth = ocTree.getTreeDepth();

      for (int i = 0; i < 100000; i++)
      {
         OcTreeKey randomKey = new OcTreeKey(random, treeDepth);
         ocTree.updateNode(randomKey, true);
      }

      for (OccupancyOcTreeNode node : ocTree)
      {
         int nodeDepth = node.getDepth();
         OcTreeKey key = new OcTreeKey();
         node.getKey(key);
         Point3d nodeCoordinate = OcTreeKeyConversionTools.keyToCoordinate(key, nodeDepth, resolution, treeDepth);
         OccupancyOcTreeNode expectedNode = node;

         if (nodeDepth == treeDepth)
         {
            OccupancyOcTreeNode actualNode = ocTree.search(nodeCoordinate);
            assertEquals(expectedNode, actualNode);
         }

         OccupancyOcTreeNode actualNode = ocTree.search(nodeCoordinate, nodeDepth);
         assertEquals(expectedNode, actualNode);
      }
   }
}
