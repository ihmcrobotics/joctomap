package us.ihmc.octoMap.ocTree.rules;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.ocTree.rules.interfaces.UpdateRule;
import us.ihmc.octoMap.occupancy.OccupancyParametersReadOnly;
import us.ihmc.octoMap.occupancy.OccupancyTools;

public class NormalOctreeUpdateRule implements UpdateRule<NormalOcTreeNode>
{
   private double alphaHitLocationUpdate;
   private final Point3d hitLocation = new Point3d();
   private final Point3d sensorLocation = new Point3d();
   private final Vector3d initialGuessNormal = new Vector3d();

   private float updateLogOdds = Float.NaN;
   private final OccupancyParametersReadOnly parameters;

   public NormalOctreeUpdateRule(OccupancyParametersReadOnly occupancyParameters)
   {
      this.parameters = occupancyParameters;
   }

   public void setUpdateLogOdds(float updateLogOdds)
   {
      this.updateLogOdds = updateLogOdds;
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
      OccupancyTools.updateNodeLogOdds(parameters, leafToUpdate, updateLogOdds);

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
      innerNodeToUpdate.updateOccupancyChildren();
   }
}
