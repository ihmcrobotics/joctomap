package us.ihmc.jOctoMap.tools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.ThreadLocalRandom;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.apache.commons.lang3.mutable.MutableInt;
import org.apache.commons.math3.stat.descriptive.moment.Variance;

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

      List<NormalOcTreeNode> neighbors = searchNeighbors(root, currentNode, parameters);

      double maxDistanceFromPlane = parameters.getMaxDistanceFromPlane();

      if (neighbors.size() < 2)
         return;

      Random random = ThreadLocalRandom.current();
      Point3d[] randomDraw = {new Point3d(), new Point3d()};
      Vector3d currentNormal = new Vector3d();
      Point3d currentNodeHitLocation = new Point3d();

      currentNode.getNormal(currentNormal);
      currentNode.getHitLocation(currentNodeHitLocation);

      // Need to be recomputed as the neighbors may have changed
      MutableInt currentConsensus = new MutableInt();
      MutableDouble currentVariance = new MutableDouble();
      computeNormalConsensusAndVariance(currentNodeHitLocation, currentNormal, neighbors, maxDistanceFromPlane, currentVariance, currentConsensus);

//      boolean hasNormalBeenUpdatedAtLeastOnce = false;
//      do
      {
         int[] randomIndices = random.ints(0, neighbors.size()).distinct().limit(2).toArray();
         neighbors.get(randomIndices[0]).getHitLocation(randomDraw[0]);
         neighbors.get(randomIndices[1]).getHitLocation(randomDraw[1]);

         Vector3d normalCandidate = JOctoMapGeometryTools.computeNormal(currentNodeHitLocation, randomDraw[0], randomDraw[1]);

         MutableInt candidateConsensus = new MutableInt();
         MutableDouble candidateVariance = new MutableDouble();
         computeNormalConsensusAndVariance(currentNodeHitLocation, normalCandidate, neighbors, maxDistanceFromPlane, candidateVariance, candidateConsensus);

         double minConsensusRatio = parameters.getMinConsensusRatio();
         double maxAverageDeviationRatio = parameters.getMaxAverageDeviationRatio();

         boolean isSimplyBetter = candidateConsensus.intValue() >= currentConsensus.intValue() && candidateVariance.doubleValue() <= currentVariance.doubleValue();
         boolean hasSmallerConsensusButIsMuchBetter = candidateConsensus.intValue() >= (int) (minConsensusRatio * currentConsensus.intValue())
               && candidateVariance.doubleValue() <= maxAverageDeviationRatio * currentVariance.doubleValue();

         if (isSimplyBetter || hasSmallerConsensusButIsMuchBetter)
         {
            if (currentNormal.dot(normalCandidate) < 0.0)
               normalCandidate.negate();

            currentNode.setNormal(normalCandidate);
            currentNode.setNormalQuality(candidateVariance.floatValue(), candidateConsensus.intValue());
            currentVariance = candidateVariance;
//            hasNormalBeenUpdatedAtLeastOnce = true;
         }
      }
//      while (!hasNormalBeenUpdatedAtLeastOnce && currentAverageDeviation > 0.005);// TODO Review the approach. It is pretty time consuming for large datasets.
   }

   private static List<NormalOcTreeNode> searchNeighbors(NormalOcTreeNode root, NormalOcTreeNode currentNode, NormalEstimationParameters parameters)
   {
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

      OcTreeNearestNeighborTools.findRadiusNeighbors(root, currentNode, searchRadius, collectNodeCentersRule);
      return neighbors;
   }

   private static void computeNormalConsensusAndVariance(Point3d pointOnPlane, Vector3d planeNormal, Iterable<NormalOcTreeNode> neighbors, double maxDistanceFromPlane, MutableDouble varianceToPack, MutableInt consensusToPack)
   {
      Variance variance = new Variance();
      consensusToPack.setValue(0);

      Vector3d toNeighborHitLocation = new Vector3d();

      for (NormalOcTreeNode neighbor : neighbors)
      {
         toNeighborHitLocation.set(neighbor.getHitLocationX(), neighbor.getHitLocationY(), neighbor.getHitLocationZ());
         toNeighborHitLocation.sub(pointOnPlane);
         double distanceFromPlane = Math.abs(planeNormal.dot(toNeighborHitLocation));
         if (distanceFromPlane <= maxDistanceFromPlane)
         {
            variance.increment(distanceFromPlane);
            consensusToPack.increment();
         }
      }

      if (consensusToPack.intValue() == 0)
         varianceToPack.setValue(Double.POSITIVE_INFINITY);
      else
         varianceToPack.setValue(variance.getResult());
   }
}
