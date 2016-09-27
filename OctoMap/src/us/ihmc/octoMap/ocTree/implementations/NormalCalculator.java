package us.ihmc.octoMap.ocTree.implementations;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.tools.OcTreeKeyConversionTools;
import us.ihmc.octoMap.tools.OcTreeNearestNeighborTools;
import us.ihmc.octoMap.tools.OcTreeNearestNeighborTools.NeighborActionRule;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class NormalCalculator
{
   private final Random random = new Random(1651L);

   private NormalOcTreeNode currentNode;
   private final Vector3d currentNodeNormal = new Vector3d();
   private final Point3d currentNodeCenter = new Point3d();

   private final Vector3d normalCandidate = new Vector3d();
   private final Point3d[] randomDraw = {new Point3d(), new Point3d()};
   private final Vector3d nodeCenterToNeighborCenter = new Vector3d();

   private final RecyclingArrayList<Point3d> tempNeighborCenters = new RecyclingArrayList<>(Point3d.class);
   private final NeighborActionRule<NormalOcTreeNode> collectNodeCentersRule = new NeighborActionRule<NormalOcTreeNode>()
   {
      @Override
      public void doActionOnNeighbor(NormalOcTreeNode node, OcTreeKeyReadOnly nodeKey)
      {
         if (currentNode != node)
            node.getCenter(tempNeighborCenters.add());
      }
   };

   private NormalOcTreeNode root;
   private double resolution;
   private int treeDepth;

   public NormalCalculator()
   {
   }

   public void setOcTreeParameters(NormalOcTreeNode root, double resolution, int treeDepth)
   {
      this.root = root;
      this.resolution = resolution;
      this.treeDepth = treeDepth;
   }

   public void computeNodeNormalRansac(NormalOcTreeNode node, OcTreeKeyReadOnly key, double searchRadius, double maxDistanceFromPlane)
   {
      if (!node.isCenterSet() || !node.isNormalSet())
      {
         node.resetNormal();
         return;
      }

      currentNode = node;
      currentNode.getNormal(currentNodeNormal);
      currentNode.getCenter(currentNodeCenter);

      // Need to be recomputed as the neighbors may have changed
      double nodeNormalQuality = 0.0;
      int nodeNumberOfPoints = 0;

      Point3d coord = OcTreeKeyConversionTools.keyToCoordinate(key, resolution, treeDepth);
      tempNeighborCenters.clear();
      OcTreeNearestNeighborTools.findRadiusNeighbors(root, coord, searchRadius, collectNodeCentersRule, resolution, treeDepth);

      for (int i = 0; i < tempNeighborCenters.size(); i++)
      {
         Point3d neighborCenter = tempNeighborCenters.get(i);

         nodeCenterToNeighborCenter.sub(currentNodeCenter, neighborCenter);
         double distanceFromPlane = Math.abs(currentNodeNormal.dot(nodeCenterToNeighborCenter));
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

      normalCandidate.set(0.0, 0.0, 0.0);

      boolean hasNormalBeenUpdatedAtLeastOnce = false;
      do 
      {
         int index = 0;

         while (index < 2)
         {
            // Did not find two other points. Give up for now.
            if (tempNeighborCenters.isEmpty())
               return;

            int nextInt = random.nextInt(tempNeighborCenters.size());
            randomDraw[index++].set(tempNeighborCenters.get(nextInt));
            tempNeighborCenters.fastRemove(nextInt);
         }

         double v1_x = randomDraw[0].getX() - currentNodeCenter.getX();
         double v1_y = randomDraw[0].getY() - currentNodeCenter.getY();
         double v1_z = randomDraw[0].getZ() - currentNodeCenter.getZ();

         double v2_x = randomDraw[1].getX() - currentNodeCenter.getX();
         double v2_y = randomDraw[1].getY() - currentNodeCenter.getY();
         double v2_z = randomDraw[1].getZ() - currentNodeCenter.getZ();

         normalCandidate.setX(v1_y * v2_z - v1_z * v2_y);
         normalCandidate.setY(v2_x * v1_z - v2_z * v1_x);
         normalCandidate.setZ(v1_x * v2_y - v1_y * v2_x);
         normalCandidate.normalize();

         float candidateNormalQuality = 0.0f;
         int candidateNumberOfPoints = 2; // The two points picked randomly are exactly on the plane

         for (int i = 0; i < tempNeighborCenters.size(); i++)
         {
            nodeCenterToNeighborCenter.sub(currentNodeCenter, tempNeighborCenters.get(i));
            double distanceFromPlane = Math.abs(normalCandidate.dot(nodeCenterToNeighborCenter));
            if (distanceFromPlane < maxDistanceFromPlane)
            {
               candidateNormalQuality += distanceFromPlane;
               candidateNumberOfPoints++;
            }
         }

         candidateNormalQuality /= candidateNumberOfPoints;

         boolean isSimplyBetter = candidateNumberOfPoints >= nodeNumberOfPoints && candidateNormalQuality <= nodeNormalQuality;
         boolean hasLittleLessNodesButIsMuchBetter = candidateNumberOfPoints >= (int) (0.75 * nodeNumberOfPoints)
               && candidateNormalQuality <= 0.5 * nodeNormalQuality;

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
