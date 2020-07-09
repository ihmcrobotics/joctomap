package us.ihmc.jOctoMap.node;

import org.apache.commons.math3.util.Precision;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.jOctoMap.node.baseImplementation.AbstractOccupancyOcTreeNode;
import us.ihmc.jOctoMap.pointCloud.PointCloud;

public class NormalOcTreeNode extends AbstractOccupancyOcTreeNode<NormalOcTreeNode>
{
   private float normalX = Float.NaN;
   private float normalY = Float.NaN;
   private float normalZ = Float.NaN;
   private float normalAverageDeviation = Float.NaN;
   private int normalConsensusSize = 0;
   private float hitLocationX = Float.NaN;
   private float hitLocationY = Float.NaN;
   private float hitLocationZ = Float.NaN;

   /**
    * Timestamp recorded when this node was last hit. Can be used to identify node that have not been
    * hit for a long time and potentially prune them.
    */
   private long lastHitTimestamp = PointCloud.UNDEFINED_TIMESTAMP;

   private long numberOfHits;

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
      hitLocationX = other.hitLocationX;
      hitLocationY = other.hitLocationY;
      hitLocationZ = other.hitLocationZ;
      numberOfHits = other.numberOfHits;
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
      resetHitLocation();
   }

   public void resetNormal()
   {
      normalX = Float.NaN;
      normalY = Float.NaN;
      normalZ = Float.NaN;
      normalAverageDeviation = Float.NaN;
      normalConsensusSize = 0;
   }

   public void resetHitLocation()
   {
      hitLocationX = Float.NaN;
      hitLocationY = Float.NaN;
      hitLocationZ = Float.NaN;
      numberOfHits = 0;
      lastHitTimestamp = PointCloud.UNDEFINED_TIMESTAMP;
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

   public void getNormal(Vector3DBasics normalToPack)
   {
      normalToPack.set(normalX, normalY, normalZ);
   }

   public Vector3D getNormalCopy()
   {
      return new Vector3D(normalX, normalY, normalZ);
   }

   public void setNormal(Vector3DReadOnly normal)
   {
      normalX = (float) normal.getX();
      normalY = (float) normal.getY();
      normalZ = (float) normal.getZ();
   }

   public void negateNormal()
   {
      normalX = -normalX;
      normalY = -normalY;
      normalZ = -normalZ;
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

   public boolean isHitLocationSet()
   {
      return !Float.isNaN(hitLocationX) && !Float.isNaN(hitLocationY) && !Float.isNaN(hitLocationZ);
   }

   public void getHitLocation(Point3DBasics hitLocationToPack)
   {
      hitLocationToPack.set(hitLocationX, hitLocationY, hitLocationZ);
   }

   public Point3D getHitLocationCopy()
   {
      return new Point3D(hitLocationX, hitLocationY, hitLocationZ);
   }

   public void updateHitLocation(Point3DReadOnly centerUpdate)
   {
      updateHitLocation(centerUpdate, 1L);
   }

   public void updateHitLocation(Point3DReadOnly centerUpdate, long updateWeight)
   {
      updateHitLocation(centerUpdate, updateWeight, Long.MAX_VALUE);
   }

   public void updateHitLocation(Point3DReadOnly centerUpdate, long updateWeight, long maximumNumberOfHits)
   {
      updateHitLocation(centerUpdate, updateWeight, maximumNumberOfHits, PointCloud.UNDEFINED_TIMESTAMP);
   }

   public void updateHitLocation(Point3DReadOnly centerUpdate, long updateWeight, long maximumNumberOfHits, long currentTimestamp)
   {
      float xUpdate = (float) centerUpdate.getX();
      float yUpdate = (float) centerUpdate.getY();
      float zUpdate = (float) centerUpdate.getZ();
      updateHitLocation(xUpdate, yUpdate, zUpdate, updateWeight, maximumNumberOfHits, currentTimestamp);
   }

   public void updateHitLocation(float xUpdate, float yUpdate, float zUpdate)
   {
      updateHitLocation(xUpdate, yUpdate, zUpdate, 1L, Long.MAX_VALUE, PointCloud.UNDEFINED_TIMESTAMP);
   }

   private void updateHitLocation(float xUpdate, float yUpdate, float zUpdate, long updateWeight, long maximumNumberOfHits, long currentTimestamp)
   {
      if (numberOfHits == 0)
      {
         hitLocationX = 0.0f;
         hitLocationY = 0.0f;
         hitLocationZ = 0.0f;
      }

      numberOfHits += updateWeight;
      numberOfHits = Math.min(numberOfHits, maximumNumberOfHits);
      double nInv = (double) updateWeight / (double) numberOfHits;
      hitLocationX += (xUpdate - hitLocationX) * nInv;
      hitLocationY += (yUpdate - hitLocationY) * nInv;
      hitLocationZ += (zUpdate - hitLocationZ) * nInv;
      // In case the new timestamp is either undefined or older we don't update it.
      if (currentTimestamp != PointCloud.UNDEFINED_TIMESTAMP && currentTimestamp > lastHitTimestamp)
         lastHitTimestamp = currentTimestamp;
   }

   public void updateHitLocationChildren()
   {
      if (children == null)
      {
         resetHitLocation();
         return;
      }

      hitLocationX = 0.0f;
      hitLocationY = 0.0f;
      hitLocationZ = 0.0f;
      int count = 0;

      for (int i = 0; i < 8; i++)
      {
         NormalOcTreeNode child = children[i];

         if (child != null && child.isHitLocationSet())
         {
            hitLocationX += child.hitLocationX;
            hitLocationY += child.hitLocationY;
            hitLocationZ += child.hitLocationZ;
            count++;
         }
      }

      if (count == 0)
      {
         resetHitLocation();
         return;
      }
      double invCount = 1.0 / count;
      hitLocationX *= invCount;
      hitLocationY *= invCount;
      hitLocationZ *= invCount;
   }

   public double getHitLocationX()
   {
      return hitLocationX;
   }

   public double getHitLocationY()
   {
      return hitLocationY;
   }

   public double getHitLocationZ()
   {
      return hitLocationZ;
   }

   public long getNumberOfHits()
   {
      return numberOfHits;
   }

   public double getNormalX()
   {
      return normalX;
   }

   public double getNormalY()
   {
      return normalY;
   }

   public double getNormalZ()
   {
      return normalZ;
   }

   public boolean isLastHitTimestampDefined()
   {
      return lastHitTimestamp != PointCloud.UNDEFINED_TIMESTAMP;
   }

   public long getLastHitTimestamp()
   {
      return lastHitTimestamp;
   }

   @Override
   protected boolean epsilonEqualsInternal(NormalOcTreeNode other, double epsilon)
   {
      if (!Precision.equals(normalX, other.normalX, epsilon))
         return false;
      if (!Precision.equals(normalY, other.normalY, epsilon))
         return false;
      if (!Precision.equals(normalZ, other.normalZ, epsilon))
         return false;
      if (!Precision.equals(hitLocationX, other.hitLocationX, epsilon))
         return false;
      if (!Precision.equals(hitLocationY, other.hitLocationY, epsilon))
         return false;
      if (!Precision.equals(hitLocationZ, other.hitLocationZ, epsilon))
         return false;
      return super.epsilonEqualsInternal(other, epsilon);
   }
}
