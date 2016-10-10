package us.ihmc.octoMap.tools;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.ThreadLocalRandom;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import gnu.trove.map.hash.TDoubleObjectHashMap;
import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyList;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.tools.OcTreeNearestNeighborTools.NeighborActionRule;

public abstract class NormalEstimationTools
{
   public static void computeNodeNormalRansac(NormalOcTreeNode root, OcTreeKeyReadOnly key, double searchRadius, double maxDistanceFromPlane, int treeDepth)
   {
      NormalOcTreeNode currentNode = OcTreeSearchTools.search(root, key, treeDepth);
      computeNodeNormalRansac(root, currentNode, searchRadius, maxDistanceFromPlane);
   }

   public static void computeNodeNormalRansac(NormalOcTreeNode root, NormalOcTreeNode currentNode, double searchRadius, double maxDistanceFromPlane)
   {
      if (!currentNode.isHitLocationSet() || !currentNode.isNormalSet())
      {
         currentNode.resetNormal();
         return;
      }

      List<NormalOcTreeNode> neighbors = new ArrayList<>();
      NeighborActionRule<NormalOcTreeNode> collectNodeCentersRule = new NeighborActionRule<NormalOcTreeNode>()
      {
         @Override
         public void doActionOnNeighbor(NormalOcTreeNode node)
         {
            if (currentNode != node)
               neighbors.add(node);
         }
      };

      OcTreeNearestNeighborTools.findRadiusNeighbors(root, currentNode, searchRadius, collectNodeCentersRule);

      computeNormalRansac(currentNode, maxDistanceFromPlane, neighbors);
   }

   public static void computeNodeNormalRansac(OcTreeKey currentNodeKey, Map<OcTreeKey, NormalOcTreeNode> keyToNodeMap,
         double searchRadius, double maxDistanceFromPlane, double resolution, int treeDepth)
   {
      NormalOcTreeNode currentNode = keyToNodeMap.get(currentNodeKey);

      if (!currentNode.isHitLocationSet() || !currentNode.isNormalSet())
      {
         currentNode.resetNormal();
         return;
      }

      List<NormalOcTreeNode> neighbors = new ArrayList<>();

      List<OcTreeKey> cachedNeighborKeyOffsets = getCachedNeighborKeyOffsets(searchRadius, resolution, treeDepth);

      OcTreeKey currentKey = new OcTreeKey();

      for (int i = 0; i < cachedNeighborKeyOffsets.size(); i++)
      {
         currentKey.add(currentNodeKey, cachedNeighborKeyOffsets.get(i));
         NormalOcTreeNode neighborNode = keyToNodeMap.get(currentKey);

         if (neighborNode != null && neighborNode.isHitLocationSet())
            neighbors.add(neighborNode);
      }

      computeNormalRansac(currentNode, maxDistanceFromPlane, neighbors);
   }

   private static final TDoubleObjectHashMap<OcTreeKeyList> neighborOffsetsCached = new TDoubleObjectHashMap<>(1);

   public static OcTreeKeyList getCachedNeighborKeyOffsets(double searchRadius, double resolution, int treeDepth)
   {
      OcTreeKeyList cachedNeighborOffsets = neighborOffsetsCached.get(searchRadius);
      if (cachedNeighborOffsets == null)
      {
         cachedNeighborOffsets = new OcTreeKeyList();
         neighborOffsetsCached.put(searchRadius, cachedNeighborOffsets);
         OcTreeKeyTools.computeNeighborKeyOffsets(treeDepth, resolution, treeDepth, searchRadius, cachedNeighborOffsets);
      }
      return cachedNeighborOffsets;
   }

   private static void computeNormalRansac(NormalOcTreeNode currentNode, double maxDistanceFromPlane, List<NormalOcTreeNode> neighbors)
   {
      Vector3d normalCandidate = new Vector3d();
      Random random = ThreadLocalRandom.current();
      Point3d[] randomDraw = {new Point3d(), new Point3d()};
      Vector3d currentNodeNormal = new Vector3d();
      Point3d currentNodeHitLocation = new Point3d();
      Vector3d currentNodeToNeighbor = new Vector3d();

      currentNode.getNormal(currentNodeNormal);
      currentNode.getHitLocation(currentNodeHitLocation);

      // Need to be recomputed as the neighbors may have changed
      double nodeNormalQuality = 0.0;
      int nodeNumberOfPoints = 0;

      for (int i = 0; i < neighbors.size(); i++)
      {
         NormalOcTreeNode neighbor = neighbors.get(i);

         currentNodeToNeighbor.set(neighbor.getHitLocationX(), neighbor.getHitLocationY(), neighbor.getHitLocationZ());
         currentNodeToNeighbor.sub(currentNodeHitLocation);
         double distanceFromPlane = Math.abs(currentNodeNormal.dot(currentNodeToNeighbor));
         if (distanceFromPlane <= maxDistanceFromPlane)
         {
            nodeNormalQuality += distanceFromPlane;
            nodeNumberOfPoints++;
         }
      }

      if (nodeNumberOfPoints == 0)
         nodeNormalQuality = Double.POSITIVE_INFINITY;
      else
         nodeNormalQuality /= nodeNumberOfPoints;

      boolean hasNormalBeenUpdatedAtLeastOnce = false;
      do
      {
         int index = 0;

         while (index < 2)
         {
            // Did not find two other points. Give up for now.
            if (neighbors.isEmpty())
               return;

            int nextInt = random.nextInt(neighbors.size());
            NormalOcTreeNode neighbor = neighbors.get(nextInt);
            randomDraw[index++].set(neighbor.getHitLocationX(), neighbor.getHitLocationY(), neighbor.getHitLocationZ());
            neighbors.remove(nextInt);
         }

         double v1_x = randomDraw[0].getX() - currentNodeHitLocation.getX();
         double v1_y = randomDraw[0].getY() - currentNodeHitLocation.getY();
         double v1_z = randomDraw[0].getZ() - currentNodeHitLocation.getZ();

         double v2_x = randomDraw[1].getX() - currentNodeHitLocation.getX();
         double v2_y = randomDraw[1].getY() - currentNodeHitLocation.getY();
         double v2_z = randomDraw[1].getZ() - currentNodeHitLocation.getZ();

         normalCandidate.setX(v1_y * v2_z - v1_z * v2_y);
         normalCandidate.setY(v2_x * v1_z - v2_z * v1_x);
         normalCandidate.setZ(v1_x * v2_y - v1_y * v2_x);
         normalCandidate.normalize();

         float candidateNormalQuality = 0.0f;
         int candidateNumberOfPoints = 2; // The two points picked randomly are exactly on the plane

         for (int i = 0; i < neighbors.size(); i++)
         {
            NormalOcTreeNode neighbor = neighbors.get(i);
            currentNodeToNeighbor.set(neighbor.getHitLocationX(), neighbor.getHitLocationY(), neighbor.getHitLocationZ());
            currentNodeToNeighbor.sub(currentNodeHitLocation);
            double distanceFromPlane = Math.abs(normalCandidate.dot(currentNodeToNeighbor));
            if (distanceFromPlane < maxDistanceFromPlane)
            {
               candidateNormalQuality += distanceFromPlane;
               candidateNumberOfPoints++;
            }
         }

         candidateNormalQuality /= candidateNumberOfPoints;

         boolean isSimplyBetter = candidateNumberOfPoints >= nodeNumberOfPoints && candidateNormalQuality <= nodeNormalQuality;
         boolean hasLittleLessNodesButIsMuchBetter = candidateNumberOfPoints >= (int) (0.5 * nodeNumberOfPoints)
               && candidateNormalQuality <= 0.75 * nodeNormalQuality;

         if (isSimplyBetter || hasLittleLessNodesButIsMuchBetter)
         {
            if (currentNodeNormal.dot(normalCandidate) < 0.0)
               normalCandidate.negate();

            currentNode.setNormal(normalCandidate);
            currentNode.setNormalQuality(candidateNormalQuality, candidateNumberOfPoints);
            nodeNormalQuality = candidateNormalQuality;
            hasNormalBeenUpdatedAtLeastOnce = true;
         }
      }
      while (!hasNormalBeenUpdatedAtLeastOnce && nodeNormalQuality > 0.005); // TODO Check if necessary, maybe only one iteration when normal is pretty good already.
   }
}
