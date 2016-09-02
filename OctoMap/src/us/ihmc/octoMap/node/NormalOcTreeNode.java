package us.ihmc.octoMap.node;

import javax.vecmath.Vector3d;

import us.ihmc.octoMap.ocTree.PlanarRegion;

public class NormalOcTreeNode extends AbstractOccupancyOcTreeNode<NormalOcTreeNode>
{
   private float normalX = Float.NaN;
   private float normalY = Float.NaN;
   private float normalZ = Float.NaN;
   private int regionId = PlanarRegion.NO_REGION_ID;

   public NormalOcTreeNode()
   {
   }

   @Override
   public void copyData(NormalOcTreeNode other)
   {
      super.copyData(other);
      normalX = other.normalX;
      normalY = other.normalY;
      normalZ = other.normalZ;
   }

   @Override
   public void allocateChildren()
   {
      children = new NormalOcTreeNode[8];
   }

   @Override
   public NormalOcTreeNode create()
   {
      return new NormalOcTreeNode();
   }

   @Override
   public void clear()
   {
      super.resetLogOdds();
      resetNormal();
      resetRegionId();
   }

   public void resetRegionId()
   {
      regionId = PlanarRegion.NO_REGION_ID;
   }

   public void updateNormalChildren()
   {
      if (children == null)
      {
         resetNormal();
         return;
      }

      normalX = 0.0f;
      normalY = 0.0f;
      normalZ = 0.0f;
      int count = 0;

      for (int i = 0; i < 8; i++)
      {
         NormalOcTreeNode child = children[i];

         if (child != null && child.isNormalSet())
         {
            normalX += child.normalX;
            normalY += child.normalY;
            normalZ += child.normalZ;
            count++;
         }
      }

      if (count == 0)
      {
         resetNormal();
         return;
      }

      float invNorm = (float) (1.0 / Math.sqrt(normalX * normalX + normalY * normalY + normalZ * normalZ));
      normalX *= invNorm;
      normalY *= invNorm;
      normalZ *= invNorm;
   }

   public void getNormal(Vector3d normalToPack)
   {
      normalToPack.set(normalX, normalY, normalZ);
   }

   public void setNormal(Vector3d normal)
   {
      normalX = (float) normal.getX();
      normalY = (float) normal.getY();
      normalZ = (float) normal.getZ();
   }

   public void resetNormal()
   {
      normalX = Float.NaN;
      normalY = Float.NaN;
      normalZ = Float.NaN;
   }

   public boolean isNormalSet()
   {
      return !Float.isNaN(normalX) && !Float.isNaN(normalY) && !Float.isNaN(normalZ);
   }

   public boolean isPartOfRegion()
   {
      return regionId != PlanarRegion.NO_REGION_ID;
   }

   public void setRegionId(int regionId)
   {
      this.regionId = regionId;
   }

   public int getRegionId()
   {
      return regionId;
   }
}
