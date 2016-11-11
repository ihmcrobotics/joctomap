package us.ihmc.jOctoMap.rules;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.occupancy.OccupancyParametersReadOnly;
import us.ihmc.jOctoMap.rules.interfaces.UpdateRule;
import us.ihmc.jOctoMap.tools.OccupancyTools;

public class NormalOcTreeNodeIntegrateRule implements UpdateRule<NormalOcTreeNode>
{
   private final Point3d hitLocation = new Point3d();
   private final Point3d sensorLocation = new Point3d();
   private final Vector3d initialGuessNormal = new Vector3d();
   private final Vector3d nodeNormal = new Vector3d();

   private NormalOcTreeNode nodeToIntegrate;

   private float updateLogOdds = Float.NaN;
   private final OccupancyParametersReadOnly parameters;

   public NormalOcTreeNodeIntegrateRule(OccupancyParametersReadOnly occupancyParameters)
   {
      this.parameters = occupancyParameters;
   }

   public void setUpdateLogOdds(float updateLogOdds)
   {
      this.updateLogOdds = updateLogOdds;
   }

   public void setHitLocation(Tuple3d sensorLocation, NormalOcTreeNode nodeToIntegrate)
   {
      this.sensorLocation.set(sensorLocation);
      this.nodeToIntegrate = nodeToIntegrate;
   }

   @Override
   public void updateLeaf(NormalOcTreeNode leafToUpdate, OcTreeKeyReadOnly leafKey, boolean nodeJustCreated)
   {
      OccupancyTools.updateNodeLogOdds(parameters, leafToUpdate, updateLogOdds);

      leafToUpdate.updateHitLocation(nodeToIntegrate);

      // FIXME incoherent with NormalOcTree.insertNormalOcTree because the transform ends up not being used.
      nodeToIntegrate.getHitLocation(hitLocation);

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
