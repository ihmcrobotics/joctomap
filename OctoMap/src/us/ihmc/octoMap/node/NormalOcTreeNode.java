package us.ihmc.octoMap.node;

import javax.vecmath.Vector3d;

import us.ihmc.octoMap.ocTree.PlanarRegion;

public class NormalOcTreeNode extends AbstractOccupancyOcTreeNode<NormalOcTreeNode>
{
   private Vector3d normal = null;
   private int regionId = PlanarRegion.NO_REGION_ID;

   public NormalOcTreeNode()
   {
   }

   @Override
   public void copyData(NormalOcTreeNode other)
   {
      super.copyData(other);
      if (other.normal != null)
      {
         if (normal == null)
            normal = new Vector3d();
         normal.set(other.normal);
      }
   }

   @Override
   public void allocateChildren()
   {
      children = new NormalOcTreeNode[8];
   }

   private static long count = 0L;

   @Override
   public NormalOcTreeNode create()
   {
      if (++count % 100000 == 0)
         System.out.println("Number of nodes created: " + count);
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
      setNormal(getAverageChildNormal());
   }

   public Vector3d getAverageChildNormal()
   {
      if (children == null)
         return null;

      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      int count = 0;

      for (int i = 0; i < 8; i++)
      {
         NormalOcTreeNode child = children[i];

         if (child != null && child.isNormalSet())
         {
            Vector3d childNormal = child.normal;
            x += childNormal.getX();
            y += childNormal.getY();
            z += childNormal.getZ();
            count++;
         }
      }

      if (count == 0)
         return null;
      x /= count;
      y /= count;
      z /= count;
      Vector3d normal = new Vector3d(x, y, z);
      normal.normalize();
      return normal;
   }

   public void getNormal(Vector3d normalToPack)
   {
      normalToPack.set(normal);
   }

   public void setNormal(Vector3d normal)
   {
      if (this.normal == null)
         this.normal = new Vector3d();
      this.normal.set(normal);
   }

   public void resetNormal()
   {
      if (normal != null)
         normal.set(Double.NaN, Double.NaN, Double.NaN);
   }

   public boolean isNormalSet()
   {
      return normal != null && !Double.isNaN(normal.getX()) && !Double.isNaN(normal.getY()) && !Double.isNaN(normal.getZ());
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
