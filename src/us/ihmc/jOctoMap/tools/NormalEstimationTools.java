package us.ihmc.jOctoMap.tools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.ThreadLocalRandom;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.tools.OcTreeNearestNeighborTools.NeighborActionRule;

public abstract class NormalEstimationTools
{
   public static void computeNodeNormalRansac(NormalOcTreeNode root, OcTreeKeyReadOnly key, NormalEstimationParameters parameters, int treeDepth)
   {
      NormalOcTreeNode currentNode = OcTreeSearchTools.search(root, key, treeDepth);
      computeNodeNormalRansac(root, currentNode, parameters);
   }

   public static void computeNodeNormalRansac(NormalOcTreeNode root, NormalOcTreeNode currentNode, NormalEstimationParameters parameters)
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

      double searchRadius = parameters.getSearchRadius();
      double maxDistanceFromPlane = parameters.getMaxDistanceFromPlane();

      OcTreeNearestNeighborTools.findRadiusNeighbors(root, currentNode, searchRadius, collectNodeCentersRule);

      Random random = ThreadLocalRandom.current();
      Point3d[] randomDraw = {new Point3d(), new Point3d()};
      Vector3d currentNormal = new Vector3d();
      Point3d currentNodeHitLocation = new Point3d();
      Vector3d currentNodeToNeighbor = new Vector3d();

      currentNode.getNormal(currentNormal);
      currentNode.getHitLocation(currentNodeHitLocation);

      // Need to be recomputed as the neighbors may have changed
      double currentAverageDeviation = 0.0;
      int currentConsensus = 0;

      for (int i = 0; i < neighbors.size(); i++)
      {
         NormalOcTreeNode neighbor = neighbors.get(i);

         currentNodeToNeighbor.set(neighbor.getHitLocationX(), neighbor.getHitLocationY(), neighbor.getHitLocationZ());
         currentNodeToNeighbor.sub(currentNodeHitLocation);
         double distanceFromPlane = Math.abs(currentNormal.dot(currentNodeToNeighbor));
         if (distanceFromPlane <= maxDistanceFromPlane)
         {
            currentAverageDeviation += distanceFromPlane;
            currentConsensus++;
         }
      }

      if (currentConsensus == 0)
         currentAverageDeviation = Double.POSITIVE_INFINITY;
      else
         currentAverageDeviation /= currentConsensus;

//      boolean hasNormalBeenUpdatedAtLeastOnce = false;
//      do
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

         Vector3d normalCandidate = JOctoMapGeometryTools.computeNormal(currentNodeHitLocation, randomDraw[0], randomDraw[1]);

         float candidateAverageDeviation = 0.0f;
         int candidateConsensus = 2; // The two points picked randomly are exactly on the plane

         for (int i = 0; i < neighbors.size(); i++)
         {
            NormalOcTreeNode neighbor = neighbors.get(i);
            currentNodeToNeighbor.set(neighbor.getHitLocationX(), neighbor.getHitLocationY(), neighbor.getHitLocationZ());
            currentNodeToNeighbor.sub(currentNodeHitLocation);
            double distanceFromPlane = Math.abs(normalCandidate.dot(currentNodeToNeighbor));
            if (distanceFromPlane < maxDistanceFromPlane)
            {
               candidateAverageDeviation += distanceFromPlane;
               candidateConsensus++;
            }
         }

         candidateAverageDeviation /= candidateConsensus;

         double minConsensusRatio = parameters.getMinConsensusRatio();
         double maxAverageDeviationRatio = parameters.getMaxAverageDeviationRatio();

         boolean isSimplyBetter = candidateConsensus >= currentConsensus && candidateAverageDeviation <= currentAverageDeviation;
         boolean hasSmallerConsensusButIsMuchBetter = candidateConsensus >= (int) (minConsensusRatio * currentConsensus)
               && candidateAverageDeviation <= maxAverageDeviationRatio * currentAverageDeviation;

         if (isSimplyBetter || hasSmallerConsensusButIsMuchBetter)
         {
            if (currentNormal.dot(normalCandidate) < 0.0)
               normalCandidate.negate();

            currentNode.setNormal(normalCandidate);
            currentNode.setNormalQuality(candidateAverageDeviation, candidateConsensus);
            currentAverageDeviation = candidateAverageDeviation;
//            hasNormalBeenUpdatedAtLeastOnce = true;
         }
      }
//      while (!hasNormalBeenUpdatedAtLeastOnce && currentAverageDeviation > 0.005);// TODO Review the approach. It is pretty time consuming for large datasets.
   }
}
