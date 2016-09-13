package us.ihmc.octoMap.boundingBox;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;

public class OcTreeBoundingBoxWithCenterAndYaw implements OcTreeBoundingBoxInterface
{
   private final Point3d centerMetric = new Point3d();
   private final Vector3d halfSizeMetric = new Vector3d();
   private final OcTreeKey centerKey = new OcTreeKey();
   private final OcTreeKey halfSizeKey = new OcTreeKey();
   private double sinYaw = 0.0, cosYaw = 1.0;

   private boolean centerMetricDirtyBit = false;
   private boolean halfSizeMetricDirtyBit = false;
   private boolean centerKeyDirtyBit = false;
   private boolean halfSizeKeyDirtyBit = false;

   public OcTreeBoundingBoxWithCenterAndYaw()
   {
   }

   public OcTreeBoundingBoxWithCenterAndYaw(OcTreeBoundingBoxWithCenterAndYaw other)
   {
      set(other);
   }

   public OcTreeBoundingBoxWithCenterAndYaw(Point3d minCoordinate, Point3d maxCoordinate)
   {
      set(minCoordinate, maxCoordinate);
   }

   public void set(OcTreeBoundingBoxWithCenterAndYaw other)
   {
      centerMetric.set(other.centerMetric);
      halfSizeMetric.set(other.halfSizeMetric);
      centerKey.set(other.centerKey);
      halfSizeKey.set(other.halfSizeKey);
      sinYaw = other.sinYaw;
      cosYaw = other.cosYaw;

      centerMetricDirtyBit = other.centerMetricDirtyBit;
      halfSizeMetricDirtyBit = other.halfSizeMetricDirtyBit;
      centerKeyDirtyBit = other.centerKeyDirtyBit;
      halfSizeKeyDirtyBit = other.halfSizeKeyDirtyBit;
   }

   public void set(Point3d minCoordinate, Point3d maxCoordinate)
   {
      centerMetric.interpolate(minCoordinate, maxCoordinate, 0.5);
      halfSizeMetric.sub(maxCoordinate, minCoordinate);
      halfSizeMetric.scale(0.5);
      sinYaw = 0.0;
      cosYaw = 1.0;

      centerMetricDirtyBit = false;
      halfSizeMetricDirtyBit = false;
      centerKeyDirtyBit = true;
      halfSizeKeyDirtyBit = true;
   }

   public void set(OcTreeKeyReadOnly minKey, OcTreeKeyReadOnly maxKey)
   {
      for (int i = 0; i < 3; i++)
      {
         int halfSize = (maxKey.getKey(i) - minKey.getKey(i)) / 2;
         halfSizeKey.setKey(i, halfSize);
      }
      centerKey.add(minKey, halfSizeKey);

      sinYaw = 0.0;
      cosYaw = 1.0;

      centerKeyDirtyBit = false;
      halfSizeKeyDirtyBit = false;
      centerMetricDirtyBit = true;
      halfSizeMetricDirtyBit = true;
   }

   public void setCenter(Point3d center)
   {
      centerMetric.set(center);

      centerMetricDirtyBit = false;
      centerKeyDirtyBit = true;
   }

   public void setHalfSize(Vector3d halfSize)
   {
      halfSizeMetric.set(halfSize);

      halfSizeMetricDirtyBit = false;
      halfSizeKeyDirtyBit = true;
   }

   public void setYaw(double yaw)
   {
      sinYaw = Math.sin(yaw);
      cosYaw = Math.cos(yaw);
   }

   @Override
   public boolean isInBoundingBox(double x, double y, double z)
   {
      if (centerMetricDirtyBit || halfSizeMetricDirtyBit)
         throw new RuntimeException("The bounding box coordinates are not up to date.");

      double zLocal = z - centerMetric.getZ();

      if (Math.abs(zLocal) > halfSizeMetric.getZ())
         return false;

      double xLocal = (x - centerMetric.getX()) * cosYaw - (y - centerMetric.getY()) * sinYaw;
      double yLocal = (x - centerMetric.getX()) * sinYaw + (y - centerMetric.getY()) * cosYaw;

      if (Math.abs(xLocal) > halfSizeMetric.getX())
         return false;

      if (Math.abs(yLocal) > halfSizeMetric.getY())
         return false;

      return true;
   }

   @Override
   public boolean isInBoundingBox(OcTreeKeyReadOnly candidate)
   {
      if (centerKeyDirtyBit || halfSizeKeyDirtyBit)
         throw new RuntimeException("The bounding box keys are not up to date.");

      int zKeyLocal = candidate.getKey(2) - centerKey.getKey(2);

      if (Math.abs(zKeyLocal) > halfSizeKey.getKey(2))
         return false;

      double xLocal = (candidate.getKey(0) - centerKey.getKey(0)) * cosYaw - (candidate.getKey(1) - centerKey.getKey(1)) * sinYaw;
      double yLocal = (candidate.getKey(0) - centerKey.getKey(0)) * sinYaw + (candidate.getKey(1) - centerKey.getKey(1)) * cosYaw;

      if (Math.abs(xLocal) > halfSizeKey.getKey(0))
         return false;

      if (Math.abs(yLocal) > halfSizeKey.getKey(1))
         return false;

      return true;
   }

   @Override
   public OcTreeBoundingBoxWithCenterAndYaw getCopy()
   {
      return new OcTreeBoundingBoxWithCenterAndYaw(this);
   }
}
