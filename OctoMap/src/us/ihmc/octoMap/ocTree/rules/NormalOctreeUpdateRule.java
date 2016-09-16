package us.ihmc.octoMap.ocTree.rules;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.occupancy.OccupancyParametersReadOnly;

public class NormalOctreeUpdateRule extends UpdateOccupancyRule<NormalOcTreeNode>
{
   private double alphaHitLocationUpdate;
   private final Point3d hitLocation = new Point3d();
   private final Point3d sensorLocation = new Point3d();
   private final Vector3d initialGuessNormal = new Vector3d();

   public NormalOctreeUpdateRule(OccupancyParametersReadOnly occupancyParameters)
   {
      super(occupancyParameters);
   }

   public void setAlphaHitLocationUpdate(double alphaHitLocationUpdate)
   {
      this.alphaHitLocationUpdate = alphaHitLocationUpdate;
   }

   public void setHitLocation(Tuple3d sensorLocation, Tuple3d hitLocation)
   {
      this.sensorLocation.set(sensorLocation);
      this.hitLocation.set(hitLocation);
   }

   public void setHitLocation(Tuple3f sensorLocation, Tuple3f hitLocation)
   {
      this.sensorLocation.set(sensorLocation);
      this.hitLocation.set(hitLocation);
   }

   @Override
   public void updateLeaf(NormalOcTreeNode leafToUpdate, OcTreeKeyReadOnly leafKey, boolean nodeJustCreated)
   {
      super.updateLeaf(leafToUpdate, leafKey, nodeJustCreated);

      leafToUpdate.updateCenter(hitLocation, alphaHitLocationUpdate);

      if (!leafToUpdate.isNormalSet())
      {
         initialGuessNormal.sub(sensorLocation, hitLocation);
         initialGuessNormal.normalize();
         leafToUpdate.setNormal(initialGuessNormal);
         leafToUpdate.setNormalQuality(Float.POSITIVE_INFINITY);
      }
   }

   @Override
   public void updateInnerNode(NormalOcTreeNode innerNodeToUpdate)
   {
      super.updateInnerNode(innerNodeToUpdate);
   }
}