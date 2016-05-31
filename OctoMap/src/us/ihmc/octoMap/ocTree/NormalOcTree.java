package us.ihmc.octoMap.ocTree;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.node.OcTreeNodeTools;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOccupancyOcTreeBase;
import us.ihmc.octoMap.tools.IntersectionPlaneBoxCalculator;
import us.ihmc.octoMap.tools.OcTreeCoordinateConversionTools;
import us.ihmc.octoMap.tools.OcTreeKeyTools;

public class NormalOcTree extends AbstractOccupancyOcTreeBase<NormalOcTreeNode>
{
   private final IntersectionPlaneBoxCalculator intersectionPlaneBoxCalculator = new IntersectionPlaneBoxCalculator();

   public NormalOcTree(double resolution)
   {
      super(resolution);
   }

   public void updateNormals()
   {
      if (root != null)
      {
         OcTreeKey rootKey = new OcTreeKey(treeMaximumValue, treeMaximumValue, treeMaximumValue);
         updateNormalsRecursively(root, rootKey, 0);
      }
   }

   private void updateNormalsRecursively(NormalOcTreeNode node, OcTreeKey nodeKey, int depth)
   {
      if (depth > treeDepth)
         throw new RuntimeException("Something went wrong.");
      if (node == null)
         throw new RuntimeException("The given node is null.");

      if (!isNodeOccupied(node))
      {
         node.setNormal(null);
         return;
      }

      if (node.hasAtLeastOneChild())
      {
         int childDepth = depth + 1;
         int centerOffsetKey = treeMaximumValue >> childDepth;

         for (int childIndex = 0; childIndex < 8; childIndex++)
         {
            if (OcTreeNodeTools.nodeChildExists(node, childIndex))
            {
               OcTreeKey childKey = OcTreeKeyTools.computeChildKey(childIndex, centerOffsetKey, nodeKey);
               NormalOcTreeNode childNode = OcTreeNodeTools.getNodeChild(node, childIndex);
               updateNormalsRecursively(childNode, childKey, childDepth);
            }
         }
         node.updateNormalChildren();
      }
      else
      {
         List<Vector3d> normals = new ArrayList<>();
         getNormals(nodeKey, normals);
         if (!normals.isEmpty())
         {
            Vector3d averageNormal = new Vector3d();
            for (Vector3d normal : normals)
            {
               averageNormal.add(normal);
            }
            averageNormal.scale(1.0 / (double) normals.size());
            averageNormal.normalize();
            node.setNormal(averageNormal);
         }
         else
         {
            node.setNormal(null);
            return;
         }
      }
      
      if (node.getNormal() != null)
      {
         Point3d center = keyToCoord(nodeKey);
         double computeNodeSize = getNodeSize(depth);
         intersectionPlaneBoxCalculator.setCube(computeNodeSize, center);
         intersectionPlaneBoxCalculator.setPlane(center, node.getNormal());
         List<Point3d> intersections = new ArrayList<>();
         intersectionPlaneBoxCalculator.computeIntersections(intersections);
         node.setPlane(intersections);
      }
   }

   @Override
   protected NormalOcTreeNode createRootNode()
   {
      return new NormalOcTreeNode();
   }
}
