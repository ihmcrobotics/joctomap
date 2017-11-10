package us.ihmc.jOctoMap.rules;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.occupancy.OccupancyParametersReadOnly;
import us.ihmc.jOctoMap.rules.interfaces.UpdateRule;
import us.ihmc.jOctoMap.tools.OccupancyTools;

public class NormalOcTreeHitUpdateRule implements UpdateRule<NormalOcTreeNode>
{
   private final Point3D hitLocation = new Point3D();
   private long updateWeight = 1L;
   private long maximumNumberOfHits = Long.MAX_VALUE;
   private final Point3D sensorLocation = new Point3D();
   private final Vector3D initialGuessNormal = new Vector3D();
   private final Vector3D nodeNormal = new Vector3D();

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

   public void setHitLocation(Tuple3DReadOnly sensorLocation, Tuple3DReadOnly hitLocation)
   {
      this.sensorLocation.set(sensorLocation);
      this.hitLocation.set(hitLocation);
      setHitUpdateWeight(1L);
   }

   public void setHitUpdateWeight(long updateWeight)
   {
      this.updateWeight = updateWeight;
   }

   public void setMaximumNumberOfHits(long maximumNumberOfHits)
   {
      this.maximumNumberOfHits = maximumNumberOfHits;
   }

   @Override
   public void updateLeaf(NormalOcTreeNode leafToUpdate, OcTreeKeyReadOnly leafKey, boolean nodeJustCreated)
   {
      OccupancyTools.updateNodeLogOdds(parameters, leafToUpdate, updateLogOdds);

      leafToUpdate.updateHitLocation(hitLocation, updateWeight, maximumNumberOfHits);

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
