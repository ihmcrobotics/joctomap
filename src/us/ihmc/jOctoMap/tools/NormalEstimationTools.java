package us.ihmc.jOctoMap.tools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.ThreadLocalRandom;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.apache.commons.lang3.mutable.MutableInt;
import org.apache.commons.math3.stat.descriptive.moment.Variance;
import org.ejml.alg.dense.decomposition.svd.SvdImplicitQrDecompose_D64;
import org.ejml.data.DenseMatrix64F;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.SingularOps;

import us.ihmc.geometry.tuple3D.Point3D;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.geometry.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;
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

      if (neighbors.size() < 2)
         return;

      double maxDistanceFromPlane = parameters.getMaxDistanceFromPlane();
      Vector3D currentNormal = currentNode.getNormalCopy();
      Point3D currentNodeHitLocation = currentNode.getHitLocationCopy();

      // Need to be recomputed as the neighbors may have changed
      MutableInt currentConsensus = new MutableInt();
      MutableDouble currentVariance = new MutableDouble();
      computeNormalConsensusAndVariance(currentNodeHitLocation, currentNormal, neighbors, maxDistanceFromPlane, currentVariance, currentConsensus);

      for (int iteration = 0; iteration < parameters.getNumberOfIterations(); iteration++)
      {
         Vector3D candidateNormal = computeNormalFromTwoRandomNeighbors(neighbors, currentNodeHitLocation);
         if (parameters.isLeastSquaresEstimationEnabled())
            candidateNormal = refineNormalWithLeastSquares(currentNodeHitLocation, candidateNormal, maxDistanceFromPlane, neighbors);

         MutableInt candidateConsensus = new MutableInt();
         MutableDouble candidateVariance = new MutableDouble();
         computeNormalConsensusAndVariance(currentNodeHitLocation, candidateNormal, neighbors, maxDistanceFromPlane, candidateVariance, candidateConsensus);

         peekBestNormal(currentNode, currentNormal, currentVariance, currentConsensus, candidateNormal, candidateVariance, candidateConsensus, parameters);
      }
   }

   private static boolean peekBestNormal(NormalOcTreeNode node, Vector3DReadOnly currentNormal, MutableDouble currentVariance, MutableInt currentConsensus,
         Vector3DBasics candidateNormal, MutableDouble candidateVariance, MutableInt candidateConsensus, NormalEstimationParameters parameters)
   {
      if (isCandidateNormalBetter(currentVariance, currentConsensus, candidateVariance, candidateConsensus, parameters))
      {
         if (currentNormal.dot(candidateNormal) < 0.0)
            candidateNormal.negate();

         node.setNormal(candidateNormal);
         node.setNormalQuality(candidateVariance.floatValue(), candidateConsensus.intValue());
         currentConsensus.setValue(candidateConsensus);
         currentVariance.setValue(candidateVariance);
         return true;
      }
      return false;
   }

   private static boolean isCandidateNormalBetter(MutableDouble currentVariance, MutableInt currentConsensus, MutableDouble candidateVariance,
         MutableInt candidateConsensus, NormalEstimationParameters parameters)
   {
      double minConsensusRatio = parameters.getMinConsensusRatio();
      double maxAverageDeviationRatio = parameters.getMaxAverageDeviationRatio();

      boolean isBetter = candidateConsensus.intValue() >= currentConsensus.intValue() && candidateVariance.doubleValue() <= currentVariance.doubleValue();
      if (isBetter)
         return true;

      boolean hasSmallerConsensusButIsMuchBetter = candidateConsensus.intValue() >= (int) (minConsensusRatio * currentConsensus.intValue())
            && candidateVariance.doubleValue() <= maxAverageDeviationRatio * currentVariance.doubleValue();
      return hasSmallerConsensusButIsMuchBetter;
   }

   private static Vector3D computeNormalFromTwoRandomNeighbors(List<NormalOcTreeNode> neighbors, Point3DReadOnly currentNodeHitLocation)
   {
      Random random = ThreadLocalRandom.current();
      Point3D[] randomHitLocations = random.ints(0, neighbors.size())
                                           .distinct()
                                           .limit(2)
                                           .mapToObj(neighbors::get)
                                           .map(NormalOcTreeNode::getHitLocationCopy)
                                           .toArray(Point3D[]::new);

      Vector3D normalCandidate = JOctoMapGeometryTools.computeNormal(currentNodeHitLocation, randomHitLocations[0], randomHitLocations[1]);
      return normalCandidate;
   }

   private static Vector3D refineNormalWithLeastSquares(Point3DReadOnly pointOnPlane, Vector3DReadOnly ransacNormal, double maxDistanceFromPlane, List<NormalOcTreeNode> neighbors)
   {
      IncrementalCovariance3D covarianceCalulator = new IncrementalCovariance3D();

      Vector3D toNeighborHitLocation = new Vector3D();

      for (NormalOcTreeNode neighbor : neighbors)
      {
         toNeighborHitLocation.set(neighbor.getHitLocationX(), neighbor.getHitLocationY(), neighbor.getHitLocationZ());
         toNeighborHitLocation.sub(pointOnPlane);
         double distanceFromPlane = Math.abs(ransacNormal.dot(toNeighborHitLocation));
         if (distanceFromPlane <= maxDistanceFromPlane)
            covarianceCalulator.addDataPoint(neighbor.getHitLocationX(), neighbor.getHitLocationY(), neighbor.getHitLocationZ());
      }

      SingularValueDecomposition<DenseMatrix64F> svd = new SvdImplicitQrDecompose_D64(true, false, true, false);
      svd.decompose(covarianceCalulator.getCovariance());
      DenseMatrix64F v = svd.getV(null, false);
      SingularOps.descendingOrder(null, false, svd.getW(null), v, false);
      
      Vector3D refinedNormal = new Vector3D(v.get(0, 2), v.get(1, 2), v.get(2, 2));
      refinedNormal.normalize();
      return refinedNormal;
   }

   private static List<NormalOcTreeNode> searchNeighbors(NormalOcTreeNode root, NormalOcTreeNode currentNode, NormalEstimationParameters parameters)
   {
      List<NormalOcTreeNode> neighbors = new ArrayList<>();
      NeighborActionRule<NormalOcTreeNode> collectNeighborsRule = new NeighborActionRule<NormalOcTreeNode>()
      {
         @Override
         public void doActionOnNeighbor(NormalOcTreeNode node)
         {
            if (currentNode != node)
               neighbors.add(node);
         }
      };

      double searchRadius = parameters.getSearchRadius();

      OcTreeNearestNeighborTools.findRadiusNeighbors(root, currentNode, searchRadius, collectNeighborsRule);
      return neighbors;
   }

   private static void computeNormalConsensusAndVariance(Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal, Iterable<NormalOcTreeNode> neighbors,
         double maxDistanceFromPlane, MutableDouble varianceToPack, MutableInt consensusToPack)
   {
      Variance variance = new Variance();
      consensusToPack.setValue(0);

      Vector3D toNeighborHitLocation = new Vector3D();

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
