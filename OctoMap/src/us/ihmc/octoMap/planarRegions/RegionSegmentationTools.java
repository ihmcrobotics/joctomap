package us.ihmc.octoMap.planarRegions;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.stream.Collectors;

import org.apache.commons.lang3.mutable.MutableBoolean;

import us.ihmc.octoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.octoMap.exceptions.PlanarRegionSegmentationException;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.ocTree.implementations.PlanarRegionSegmentationParameters;
import us.ihmc.octoMap.tools.OcTreeNearestNeighborTools;
import us.ihmc.octoMap.tools.OcTreeNearestNeighborTools.NeighborActionRule;

public class RegionSegmentationTools
{

   public static List<PlanarRegion> mergePlanarRegionsIfPossible(NormalOcTreeNode root, List<PlanarRegion> inputRegions, PlanarRegionSegmentationParameters parameters)
   {
      List<PlanarRegion> mergedRegions = new ArrayList<>();
      while (!inputRegions.isEmpty())
      {
         PlanarRegion candidateForMergeOtherRegions = inputRegions.get(0);
         Map<Boolean, List<PlanarRegion>> mergeableAndNonMergeableGroups = inputRegions.subList(1, inputRegions.size()).parallelStream()
                     // Group each region according to the result of areRegionsMergeable.
                    .collect(Collectors.groupingBy(other -> areRegionsMergeable(root, candidateForMergeOtherRegions, other, parameters)));

         // Merge all the mergeable regions onto the candidate.
         mergeableAndNonMergeableGroups.getOrDefault(true, Collections.emptyList()).forEach(candidateForMergeOtherRegions::merge);
         // All non mergeable regions, used for the next iteration.
         inputRegions = mergeableAndNonMergeableGroups.getOrDefault(false, Collections.emptyList());
         // We're done with candidate, put it in the output list.
         mergedRegions.add(candidateForMergeOtherRegions);
      }
      return mergedRegions;
   }

   public static boolean areRegionsMergeable(NormalOcTreeNode root, PlanarRegion currentRegion, PlanarRegion potentialRegionToMerge,
         PlanarRegionSegmentationParameters parameters)
   {
      if (currentRegion == potentialRegionToMerge)
         throw new PlanarRegionSegmentationException("Problem Houston.");

      double maxDistanceFromPlane = parameters.getMaxDistanceFromPlane();

      if (currentRegion.absoluteOrthogonalDistance(potentialRegionToMerge.getOrigin()) > maxDistanceFromPlane)
         return false;

      double dotThreshold = Math.cos(parameters.getMaxAngleFromPlane());

      if (currentRegion.absoluteDot(potentialRegionToMerge) < dotThreshold)
         return false;

      double searchRadius = parameters.getSearchRadius();
      int otherRegionId;
      PlanarRegion regionToNavigate;
      if (potentialRegionToMerge.getNumberOfNodes() < currentRegion.getNumberOfNodes())
      {
         regionToNavigate = potentialRegionToMerge;
         otherRegionId = currentRegion.getId();
      }
      else
      {
         regionToNavigate = currentRegion;
         otherRegionId = potentialRegionToMerge.getId();
      }

      return regionToNavigate.nodeStream()
                             .filter(node -> isNodeInOtherRegionNeighborhood(root, node, otherRegionId, searchRadius))
                             .findFirst()
                             .isPresent();
   }

   public static boolean isNodeInOtherRegionNeighborhood(NormalOcTreeNode root, NormalOcTreeNode nodeFromOneRegion, int otherRegionId, double searchRadius)
   {
      if (!nodeFromOneRegion.isPartOfRegion())
         return false; //throw new PlanarRegionSegmentationException("Problem Houston.");

      MutableBoolean foundNeighborFromOtherRegion = new MutableBoolean(false);

      NeighborActionRule<NormalOcTreeNode> actionRule = new NeighborActionRule<NormalOcTreeNode>()
      {
         @Override
         public void doActionOnNeighbor(NormalOcTreeNode node)
         {
            if (node.getRegionId() == otherRegionId)
               foundNeighborFromOtherRegion.setTrue();
         }

         @Override
         public boolean earlyAbort()
         {
            return foundNeighborFromOtherRegion.booleanValue();
         }
      };
      OcTreeNearestNeighborTools.findRadiusNeighbors(root, nodeFromOneRegion, searchRadius, actionRule);
      return foundNeighborFromOtherRegion.booleanValue();
   }

   public static boolean isNodePartOfRegion(NormalOcTreeNode node, PlanarRegion planarRegion, double maxDistanceFromPlane, double dotThreshold)
   {
      double absoluteOrthogonalDistance = planarRegion.absoluteOrthogonalDistance(node);
      if (absoluteOrthogonalDistance > maxDistanceFromPlane)
         return false;
   
      double absoluteDot = planarRegion.absoluteDotWithNodeNormal(node);
      return absoluteDot > dotThreshold;
   }

   public static void addNodeToExplorationIfPossible(NormalOcTreeNode neighborNode, int regionId, Deque<NormalOcTreeNode> exploration)
   {
      if (neighborNode.getHasBeenCandidateForRegion() == regionId || neighborNode.isPartOfRegion())
         return;
      if (!neighborNode.isNormalSet() || !neighborNode.isHitLocationSet())
         return;
      neighborNode.setHasBeenCandidateForRegion(regionId);
      exploration.add(neighborNode);
   }

   public static PlanarRegion createNewPlanarRegion(NormalOcTreeNode root, NormalOcTreeNode seedNode, int regionId, OcTreeBoundingBoxInterface boundingBox, PlanarRegionSegmentationParameters parameters)
   {
      double dotThreshold = Math.cos(parameters.getMaxAngleFromPlane());
      double searchRadius = parameters.getSearchRadius();
      double maxDistanceFromPlane = parameters.getMaxDistanceFromPlane();
   
      PlanarRegion planarRegion = new PlanarRegion(regionId);
      planarRegion.addNode(seedNode);
   
      ArrayDeque<NormalOcTreeNode> nodesToExplore = new ArrayDeque<>();
      NeighborActionRule<NormalOcTreeNode> extendSearchRule = neighborNode -> addNodeToExplorationIfPossible(neighborNode, regionId, nodesToExplore);
      OcTreeNearestNeighborTools.findRadiusNeighbors(root, seedNode, searchRadius, extendSearchRule);
   
      while (!nodesToExplore.isEmpty())
      {
         NormalOcTreeNode currentNode = nodesToExplore.poll();
         if (boundingBox != null && !boundingBox.isInBoundingBox(currentNode.getX(), currentNode.getY(), currentNode.getZ()))
            continue;
   
         if (isNodePartOfRegion(currentNode, planarRegion, maxDistanceFromPlane, dotThreshold))
         {
            planarRegion.addNode(currentNode);
            OcTreeNearestNeighborTools.findRadiusNeighbors(root, currentNode, searchRadius, extendSearchRule);
         }
      }
   
      return planarRegion;
   }

   public static void growPlanarRegion(NormalOcTreeNode root, PlanarRegion planarRegion, OcTreeBoundingBoxInterface boundingBox, PlanarRegionSegmentationParameters parameters)
   {
      double dotThreshold = Math.cos(parameters.getMaxAngleFromPlane());
      double searchRadius = parameters.getSearchRadius();
      double maxDistanceFromPlane = parameters.getMaxDistanceFromPlane();
   
      ArrayDeque<NormalOcTreeNode> nodesToExplore = new ArrayDeque<>();
   
      int currentPlanarRegionId = planarRegion.getId();
   
      NeighborActionRule<NormalOcTreeNode> extendSearchRule = neighborNode -> addNodeToExplorationIfPossible(neighborNode, currentPlanarRegionId, nodesToExplore);
      planarRegion.nodeStream().forEach(regionNode -> OcTreeNearestNeighborTools.findRadiusNeighbors(root, regionNode, searchRadius, extendSearchRule));
   
      while (!nodesToExplore.isEmpty())
      {
         NormalOcTreeNode currentNode = nodesToExplore.poll();
         if (boundingBox != null && !boundingBox.isInBoundingBox(currentNode.getX(), currentNode.getY(), currentNode.getZ()))
            continue;
   
         if (isNodePartOfRegion(currentNode, planarRegion, maxDistanceFromPlane, dotThreshold))
         {
            planarRegion.addNode(currentNode);
            OcTreeNearestNeighborTools.findRadiusNeighbors(root, currentNode, searchRadius, extendSearchRule);
         }
      }
   }

   public static void updatePlanarRegions(NormalOcTreeNode root, OcTreeBoundingBoxInterface boundingBox, PlanarRegionSegmentationParameters parameters, List<PlanarRegion> planarRegions)
   {
      double dotThreshold = Math.cos(parameters.getMaxAngleFromPlane());
      double maxDistanceFromPlane = parameters.getMaxDistanceFromPlane();
   
      for (PlanarRegion planarRegion : planarRegions)
      {
         boolean removedAtLeastOneNode = false;
   
         for (int i = planarRegion.getNumberOfNodes() - 1; i >= 0; i--)
         {
            NormalOcTreeNode node = planarRegion.getNode(i);
            // Let's not update nodes that are outside the bounding box as the OcTree should be frozen there.
            if (boundingBox != null && !boundingBox.isInBoundingBox(node.getX(), node.getY(), node.getZ()))
               continue;
   
            // Removes the nodes if: 1- node has been deleted (normal has been reset), 2- the node is physically not part of the region anymore.
            if (!node.isNormalSet() || !isNodePartOfRegion(node, planarRegion, maxDistanceFromPlane, dotThreshold))
            {
               planarRegion.removeNode(i);
               node.resetRegionId();
               removedAtLeastOneNode = true;
            }
         }
   
         if (removedAtLeastOneNode)
            planarRegion.recomputeNormalAndOrigin();
      }
   
      planarRegions.stream().forEach(region -> growPlanarRegion(root, region, boundingBox, parameters));
   }

   public static List<PlanarRegion> searchNewPlanarRegions(NormalOcTreeNode root, OcTreeBoundingBoxInterface boundingBox, PlanarRegionSegmentationParameters parameters, Random random, List<NormalOcTreeNode> leafNodes)
   {
      List<PlanarRegion> newPlanarRegions = new ArrayList<>();
   
      float minNormalQuality = (float) parameters.getMinNormalQuality();
   
      for (NormalOcTreeNode node : leafNodes)
      {
         if (node.isPartOfRegion() || !node.isNormalSet())
            continue;
         if (node.getNormalAverageDeviation() > minNormalQuality)
            continue;
   
         int regionId = random.nextInt(Integer.MAX_VALUE);
         PlanarRegion planarRegion = createNewPlanarRegion(root, node, regionId, boundingBox, parameters);
   
         if (planarRegion.getNumberOfNodes() < 10)
            planarRegion.nodeStream().forEach(nodeToReset -> nodeToReset.resetRegionId());
         else
            newPlanarRegions.add(planarRegion);
      }
   
      return newPlanarRegions;
   }

}
