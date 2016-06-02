package us.ihmc.octoMap.ocTree;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.apache.commons.math3.stat.descriptive.moment.Variance;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.node.OcTreeNodeTools;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOccupancyOcTreeBase;
import us.ihmc.octoMap.tools.IntersectionPlaneBoxCalculator;
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
         OcTreeKey rootKey = OcTreeKeyTools.getRootKey(treeDepth);
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

         for (int childIndex = 0; childIndex < 8; childIndex++)
         {
            if (OcTreeNodeTools.nodeChildExists(node, childIndex))
            {
               OcTreeKey childKey = OcTreeKeyTools.computeChildKey(childIndex, nodeKey, childDepth, treeDepth);
               NormalOcTreeNode childNode = OcTreeNodeTools.getNodeChild(node, childIndex);
               updateNormalsRecursively(childNode, childKey, childDepth);
            }
         }
         node.updateNormalChildren();
      }
      else
      {
         node.setNormal(computeNodeNormal(nodeKey, true));
      }
      
      if (node.getNormal() != null)
      {
         Point3d center = keyToCoordinate(nodeKey);
         double computeNodeSize = getNodeSize(depth);
         intersectionPlaneBoxCalculator.setCube(computeNodeSize, center);
         intersectionPlaneBoxCalculator.setPlane(center, node.getNormal());
         node.setPlane(intersectionPlaneBoxCalculator.computeIntersections());
      }
      else
      {
         node.setPlane(null);
      }
   }

   public Vector3d computeNodeNormal(OcTreeKey key, boolean unknownStatus)
   {
      NormalOcTreeNode node = search(key);
      if (node == null)
         return null;

      Vector3d normal = new Vector3d();
      OcTreeKey currentKey = new OcTreeKey();
      NormalOcTreeNode currentNode;

      int kxOffset = 0;
      int kyOffset = 0;
      int kzOffset = 0;

      for (kxOffset = -1; kxOffset <= 1; kxOffset++)
      {
         for (kyOffset = -1; kyOffset <= 1; kyOffset++)
         {
            for (kzOffset = -1; kzOffset <= 1; kzOffset++)
            {
               currentKey.k[0] = key.k[0] + kxOffset;
               currentKey.k[1] = key.k[1] + kyOffset;
               currentKey.k[2] = key.k[2] + kzOffset;
               currentNode = search(currentKey);

               boolean nodeExists = currentNode == null;
               boolean isOccupied = !nodeExists && isNodeOccupied(currentNode);
               boolean isUnknownConsideredOccupied = nodeExists && unknownStatus;
               
               if (isOccupied || isUnknownConsideredOccupied)
               {
                  normal.x -= kxOffset;
                  normal.y -= kyOffset;
                  normal.z -= kzOffset;
               }
            }
         }
      }

      double lengthSquared = normal.lengthSquared();
      if (lengthSquared > 1.0e-3)
      {
         normal.scale(1.0 / Math.sqrt(lengthSquared));
         return normal;
      }
      else
      {
         return null;
      }
   }

   public double computeNodeNeighborNormalDifference(OcTreeKey key, int depth)
   {
      NormalOcTreeNode node = search(key, depth);
      if (node == null || !node.isNormalSet())
         return Double.NaN;

      Vector3d normal = node.getNormal();
      Vector3d meanNormal = new Vector3d();
      OcTreeKey currentKey = new OcTreeKey();
      NormalOcTreeNode currentNode;

      int kxOffset = 0;
      int kyOffset = 0;
      int kzOffset = 0;
      int count = 0;

      double meanDifference = 0.0;
      Variance var = new Variance();

      for (kxOffset = -1; kxOffset <= 1; kxOffset++)
      {
         for (kyOffset = -1; kyOffset <= 1; kyOffset++)
         {
            for (kzOffset = -1; kzOffset <= 1; kzOffset++)
            {
               if (kxOffset == 0 && kyOffset == 0 && kzOffset == 0)
                  continue;

               currentKey.k[0] = key.k[0] + kxOffset;
               currentKey.k[1] = key.k[1] + kyOffset;
               currentKey.k[2] = key.k[2] + kzOffset;
               currentNode = search(currentKey, depth);

               if (currentNode == null || !currentNode.isNormalSet())
                  continue;

               meanNormal.add(currentNode.getNormal());
               meanDifference += 1.0 - Math.abs(currentNode.getNormal().dot(normal));
               var.increment(1.0 - Math.abs(currentNode.getNormal().dot(normal)));
//               meanDifference = Math.max(meanDifference, 1.0 - Math.abs(currentNode.getNormal().dot(normal)));
               count++;
            }
         }
      }

      if (count == 0)
         return Double.NaN;

      meanNormal.scale(1.0 / count);
      meanDifference = 1.0 - Math.abs(meanNormal.dot(normal));

//      meanDifference /= count;
      return meanDifference;
//      return var.getResult();
   }

   @Override
   protected NormalOcTreeNode createRootNode()
   {
      return new NormalOcTreeNode();
   }
}
