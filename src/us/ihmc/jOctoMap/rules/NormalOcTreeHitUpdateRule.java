package us.ihmc.jOctoMap.rules;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3d;

import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.occupancy.OccupancyParametersReadOnly;
import us.ihmc.jOctoMap.rules.interfaces.UpdateRule;
import us.ihmc.jOctoMap.tools.OccupancyTools;

public class NormalOcTreeHitUpdateRule implements UpdateRule<NormalOcTreeNode>
{
   private final Point3d hitLocation = new Point3d();
   private long updateWeight = 1L;
   private final Point3d sensorLocation = new Point3d();
   private final Vector3d initialGuessNormal = new Vector3d();
   private final Vector3d nodeNormal = new Vector3d();

   private float updateLogOdds = Float.NaN;
   private final OccupancyParametersReadOnly parameters;

   public NormalOcTreeHitUpdateRule(OccupancyParametersReadOnly occupancyParameters)
   {
      this.parameters = occupancyParameters;
   }

   public void setUpdateLogOdds(float updateLogOdds)
   {
      this.updateLogOdds = updateLogOdds;
   }

   public void setHitLocation(Tuple3d sensorLocation, Tuple3d hitLocation)
   {
      this.sensorLocation.set(sensorLocation);
      this.hitLocation.set(hitLocation);
      setHitUpdateWeight(1L);
   }

   public void setHitLocation(Tuple3f sensorLocation, Tuple3f hitLocation)
   {
      this.sensorLocation.set(sensorLocation);
      this.hitLocation.set(hitLocation);
      setHitUpdateWeight(1L);
   }

   public void setHitUpdateWeight(long updateWeight)
   {
      this.updateWeight = updateWeight;
   }

   @Override
   public void updateLeaf(NormalOcTreeNode leafToUpdate, OcTreeKeyReadOnly leafKey, boolean nodeJustCreated)
   {
      OccupancyTools.updateNodeLogOdds(parameters, leafToUpdate, updateLogOdds);

      leafToUpdate.updateHitLocation(hitLocation, updateWeight);

      if (!leafToUpdate.isNormalSet())
      {
         initialGuessNormal.sub(sensorLocation, hitLocation);
         initialGuessNormal.normalize();
         leafToUpdate.setNormal(initialGuessNormal);
         leafToUpdate.setNormalQuality(Float.POSITIVE_INFINITY, 0);
      }
      else // TODO review normal flips.
      {
         initialGuessNormal.sub(sensorLocation, hitLocation);
         leafToUpdate.getNormal(nodeNormal);
         if (nodeNormal.dot(initialGuessNormal) < 0.0)
            leafToUpdate.negateNormal();
      }
   }

   @Override
   public void updateInnerNode(NormalOcTreeNode innerNodeToUpdate)
   {
      innerNodeToUpdate.updateOccupancyChildren();
      innerNodeToUpdate.updateHitLocationChildren();
   }
}
