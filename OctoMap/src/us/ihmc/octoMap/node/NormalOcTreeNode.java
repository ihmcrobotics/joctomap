package us.ihmc.octoMap.node;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.ocTree.implementations.NormalOcTree;
import us.ihmc.octoMap.planarRegions.PlanarRegion;

public class NormalOcTreeNode extends AbstractOccupancyOcTreeNode<NormalOcTreeNode>
{
   private float normalX = Float.NaN;
   private float normalY = Float.NaN;
   private float normalZ = Float.NaN;
   private float normalAverageDeviation = Float.NaN;
   private int normalConsensusSize = 0;
   private float centerX = Float.NaN;
   private float centerY = Float.NaN;
   private float centerZ = Float.NaN;
   private int regionId = PlanarRegion.NO_REGION_ID;
   private int hasBeenCandidateForRegion = PlanarRegion.NO_REGION_ID;

   private long n;
   private float devX, nDevX;
   private float devY, nDevY;
   private float devZ, nDevZ;

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
      normalAverageDeviation = other.normalAverageDeviation;
      normalConsensusSize = other.normalConsensusSize;
      centerX = other.centerX;
      centerY = other.centerY;
      centerZ = other.centerZ;
      regionId = other.regionId;
      hasBeenCandidateForRegion = other.hasBeenCandidateForRegion;

      if (NormalOcTree.UPDATE_NODE_HIT_WITH_AVERAGE)
      {
         n = other.n;
         devX = other.devX;
         devY = other.devY;
         devZ = other.devZ;
         nDevX = other.nDevX;
         nDevY = other.nDevY;
         nDevZ = other.nDevZ;
      }
   }

   @Override
   public void allocateChildren()
   {
      children = new NormalOcTreeNode[8];
   }

   @Override
   public void clear()
   {
      super.resetLogOdds();
      resetNormal();
      resetCenter();
      resetRegionId();
      resetHasBeenCandidateForRegion();
   }

   public void resetNormal()
   {
      normalX = Float.NaN;
      normalY = Float.NaN;
      normalZ = Float.NaN;
      normalAverageDeviation = Float.NaN;
      normalConsensusSize = 0;
   }

   public void resetCenter()
   {
      centerX = Float.NaN;
      centerY = Float.NaN;
      centerZ = Float.NaN;

      if (NormalOcTree.UPDATE_NODE_HIT_WITH_AVERAGE)
      {
         n = 0;
         devX = Float.NaN; nDevX = Float.NaN;
         devY = Float.NaN; nDevY = Float.NaN;
         devZ = Float.NaN; nDevZ = Float.NaN;
      }
   }

   public void resetRegionId()
   {
      regionId = PlanarRegion.NO_REGION_ID;
   }

   public void resetHasBeenCandidateForRegion()
   {
      hasBeenCandidateForRegion = PlanarRegion.NO_REGION_ID;
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

   public void setNormalQuality(float averageDeviation, int consensusSize)
   {
      normalAverageDeviation = averageDeviation;
      normalConsensusSize = consensusSize;
   }

   public float getNormalAverageDeviation()
   {
      return normalAverageDeviation;
   }

   public int getNormalConsensusSize()
   {
      return normalConsensusSize;
   }

   public boolean isNormalSet()
   {
      return !Float.isNaN(normalX) && !Float.isNaN(normalY) && !Float.isNaN(normalZ);
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
      if (NormalOcTree.UPDATE_NODE_HIT_WITH_AVERAGE)
      {
         if (n == 0)
         {
            centerX = 0.0f;
            centerY = 0.0f;
            centerZ = 0.0f;
         }
         n++;
         float n0 = n;
         devX = xUpdate - centerX;
         nDevX = devX / n0;
         centerX += nDevX;

         devY = yUpdate - centerY;
         nDevY = devY / n0;
         centerY += nDevY;

         devZ = zUpdate - centerZ;
         nDevZ = devZ / n0;
         centerZ += nDevZ;
      }
      else
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

   public void updateRegionIdChildren()
   {
      if (!hasAtLeastOneChild())
      {
         resetRegionId();
         return;
      }

      int indexRegionWithHighestCount = -1;
      int highestCount = -1;

      for (int i = 0; i < 8; i++)
      {
         NormalOcTreeNode currentChild = children[i];
         if (currentChild != null && currentChild.isPartOfRegion())
         {
            int currentCount = 1;
            
            for (int j = 0; j < i; j++)
            {
               NormalOcTreeNode other = children[j];
               if (other != null && currentChild.getRegionId() == other.getRegionId())
                  currentCount++;
            }

            if (indexRegionWithHighestCount < 0 || currentCount > highestCount)
            {
               indexRegionWithHighestCount = i;
               highestCount = currentCount;
            }
         }
      }

      if (indexRegionWithHighestCount < 0)
         resetRegionId();
      else
         regionId = children[indexRegionWithHighestCount].regionId;
   }

   public int getHasBeenCandidateForRegion()
   {
      return hasBeenCandidateForRegion;
   }

   public void setHasBeenCandidateForRegion(int hasBeenCandidateForRegion)
   {
      this.hasBeenCandidateForRegion = hasBeenCandidateForRegion;
   }
}
