package us.ihmc.octoMap.boundingBox;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.tools.OcTreeKeyConversionTools;

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

   public OcTreeBoundingBoxWithCenterAndYaw(OcTreeSimpleBoundingBox simpleBoundingBox)
   {
      set(simpleBoundingBox);
   }

   public OcTreeBoundingBoxWithCenterAndYaw(OcTreeBoundingBoxWithCenterAndYaw other)
   {
      set(other);
   }

   public OcTreeBoundingBoxWithCenterAndYaw(Point3d minCoordinate, Point3d maxCoordinate)
   {
      set(minCoordinate, maxCoordinate);
   }

   public OcTreeBoundingBoxWithCenterAndYaw(Point3d minCoordinate, Point3d maxCoordinate, double resolution, int treeDepth)
   {
      set(minCoordinate, maxCoordinate);
      update(resolution, treeDepth);
   }

   public void set(OcTreeSimpleBoundingBox simpleBoundingBox)
   {
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

   public void setHalfSize(double[] halfSize)
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

   public void update(double resolution, int treeDepth)
   {
      if (centerKeyDirtyBit)
      {
         boolean success = OcTreeKeyConversionTools.coordinateToKey(centerMetric, resolution, treeDepth, centerKey);
         if (!success)
            System.err.println(getClass().getSimpleName() + " (in update): ERROR while generating center key.");
      }
      else if (centerMetricDirtyBit)
      {
         halfSizeMetric.set(halfSizeKey.getKey(0), halfSizeKey.getKey(1), halfSizeKey.getKey(2));
         halfSizeMetric.scale(resolution);
      }

      if (halfSizeKeyDirtyBit)
      {
         int k0 = (int) Math.floor(halfSizeMetric.getX() / resolution);
         int k1 = (int) Math.floor(halfSizeMetric.getY() / resolution);
         int k2 = (int) Math.floor(halfSizeMetric.getZ() / resolution);
         halfSizeKey.set(k0, k1, k2);
      }
      else if (centerMetricDirtyBit)
      {
         OcTreeKeyConversionTools.keyToCoordinate(halfSizeKey, halfSizeMetric, resolution, treeDepth);
      }

      centerKeyDirtyBit = false;
      halfSizeKeyDirtyBit = false;
      centerMetricDirtyBit = false;
      halfSizeMetricDirtyBit = false;
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

   public void getBoundingBoxHalfSize(Tuple3d halfSizeToPack)
   {
      halfSizeToPack.set(halfSizeMetric);
   }

   public void getBoundingBoxHalfSize(double[] halfSizeToPack)
   {
      halfSizeMetric.get(halfSizeToPack);
   }
}
