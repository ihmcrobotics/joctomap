package us.ihmc.octoMap.node;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.ocTree.PlanarRegion;

public class NormalOcTreeNode extends AbstractOccupancyOcTreeNode<NormalOcTreeNode>
{
   private float normalX = Float.NaN;
   private float normalY = Float.NaN;
   private float normalZ = Float.NaN;
   private float normalQuality = Float.NaN;
   private float centerX = Float.NaN;
   private float centerY = Float.NaN;
   private float centerZ = Float.NaN;
   private int regionId = PlanarRegion.NO_REGION_ID;
   private int hasBeenCandidateForRegion = PlanarRegion.NO_REGION_ID;

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
      normalQuality = other.normalQuality;
      centerX = other.centerX;
      centerY = other.centerY;
      centerZ = other.centerZ;
      regionId = other.regionId;
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

   public void negateNormal()
   {
      normalX = - normalX;
      normalY = - normalY;
      normalZ = - normalZ;
   }

   public void setNormalQuality(float quality)
   {
      normalQuality = quality;
   }

   public float getNormalQuality()
   {
      return normalQuality;
   }

   public void resetNormal()
   {
      normalX = Float.NaN;
      normalY = Float.NaN;
      normalZ = Float.NaN;
      normalQuality = Float.NaN;
   }

   public boolean isNormalSet()
   {
      return !Float.isNaN(normalX) && !Float.isNaN(normalY) && !Float.isNaN(normalZ);
   }

   public boolean isNormalQualitySet()
   {
      return !Float.isNaN(normalQuality);
   }

   public void resetCenter()
   {
      centerX = Float.NaN;
      centerY = Float.NaN;
      centerZ = Float.NaN;
   }

   public boolean isCenterSet()
   {
      return !Float.isNaN(centerX) && !Float.isNaN(centerY) && !Float.isNaN(centerY);
   }

   public void getCenter(Point3d centerToPack)
   {
      centerToPack.set(centerX, centerY, centerZ);
   }

   public void setCenter(Point3d center)
   {
      centerX = (float) center.getX();
      centerY = (float) center.getY();
      centerZ = (float) center.getZ();
   }

   public void updateCenter(Point3d centerUpdate, double alphaUpdate)
   {
      float xUpdate = (float) centerUpdate.getX();
      float yUpdate = (float) centerUpdate.getY();
      float zUpdate = (float) centerUpdate.getZ();
      updateCenter(xUpdate, yUpdate, zUpdate, (float) alphaUpdate);
   }

   public void updateCenter(Point3f centerUpdate, double alphaUpdate)
   {
      float xUpdate = centerUpdate.getX();
      float yUpdate = centerUpdate.getY();
      float zUpdate = centerUpdate.getZ();
      updateCenter(xUpdate, yUpdate, zUpdate, (float) alphaUpdate);
   }

   public void updateCenter(float xUpdate, float yUpdate, float zUpdate, float alphaUpdate)
   {
      if (!isCenterSet())
      {
         centerX = xUpdate;
         centerY = yUpdate;
         centerZ = zUpdate;
      }
      else
      {
         centerX = alphaUpdate * xUpdate + (1.0f - alphaUpdate) * centerX;
         centerY = alphaUpdate * yUpdate + (1.0f - alphaUpdate) * centerY;
         centerZ = alphaUpdate * zUpdate + (1.0f - alphaUpdate) * centerZ;
      }
   }

   public void updateCenterChildren()
   {
      if (children == null)
      {
         resetCenter();
         return;
      }

      centerX = 0.0f;
      centerY = 0.0f;
      centerZ = 0.0f;
      int count = 0;

      for (int i = 0; i < 8; i++)
      {
         NormalOcTreeNode child = children[i];

         if (child != null && child.isCenterSet())
         {
            centerX += child.centerX;
            centerY += child.centerY;
            centerZ += child.centerZ;
            count++;
         }
      }

      if (count == 0)
      {
         resetCenter();
         return;
      }
      double invCount = 1.0 / count;
      centerX *= invCount;
      centerY *= invCount;
      centerZ *= invCount;
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

   public int getHasBeenCandidateForRegion()
   {
      return hasBeenCandidateForRegion;
   }

   public void setHasBeenCandidateForRegion(int hasBeenCandidateForRegion)
   {
      this.hasBeenCandidateForRegion = hasBeenCandidateForRegion;
   }

   public void resetHasBeenCandidateForRegion()
   {
      hasBeenCandidateForRegion = PlanarRegion.NO_REGION_ID;
   }
}
