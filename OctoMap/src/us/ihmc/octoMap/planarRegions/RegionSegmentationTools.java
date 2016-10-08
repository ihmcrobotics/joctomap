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
         mergeableAndNonMergeableGroups.getOrDefault(true, Collections.emptyList()).forEach(candidateForMergeOtherRegions::addNodesFromOtherRegion);
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
         //throw new PlanarRegionSegmentationException("Problem Houston.");
         // TODO This is a weird case that happens sometimes, need to figure out what's wrong.
         return false;

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
      if (!node.isNormalSet())
         return false;

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
         if (!isNodeInBoundingBox(currentNode, boundingBox))
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
         if (!isNodeInBoundingBox(currentNode, boundingBox))
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
      planarRegions.parallelStream().forEach(region -> removeBadNodesFromRegion(boundingBox, parameters, region));
      planarRegions.stream().forEach(region -> growPlanarRegion(root, region, boundingBox, parameters));
   }

   private static void removeBadNodesFromRegion(OcTreeBoundingBoxInterface boundingBox, PlanarRegionSegmentationParameters parameters,
         PlanarRegion planarRegion)
   {
      double dotThreshold = Math.cos(parameters.getMaxAngleFromPlane());
      double maxDistanceFromPlane = parameters.getMaxDistanceFromPlane();

      List<NormalOcTreeNode> nodesToRemove = planarRegion.nodeStream()
            // Let's not update nodes that are outside the bounding box as the OcTree should be frozen there.
            .filter(node -> isNodeInBoundingBox(node, boundingBox))
            // Group nodes by whether they are part of the region or not.
            .collect(Collectors.groupingBy(node -> !isNodePartOfRegion(node, planarRegion, maxDistanceFromPlane, dotThreshold)))
            // Keep only the nodes that should be removed.
            .getOrDefault(true, Collections.emptyList());

      planarRegion.removeNodesAndUpdate(nodesToRemove);
   }

   private static boolean isNodeInBoundingBox(NormalOcTreeNode node, OcTreeBoundingBoxInterface boundingBox)
   {
      return boundingBox == null || boundingBox.isInBoundingBox(node.getX(), node.getY(), node.getZ());
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
            planarRegion.clearRegion();
         else
            newPlanarRegions.add(planarRegion);
      }

      return newPlanarRegions;
   }

}
