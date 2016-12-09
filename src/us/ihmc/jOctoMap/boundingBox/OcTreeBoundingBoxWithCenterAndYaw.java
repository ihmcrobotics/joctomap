package us.ihmc.jOctoMap.boundingBox;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.apache.commons.math3.util.FastMath;

import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.tools.OcTreeKeyConversionTools;
import us.ihmc.jOctoMap.tools.OcTreeKeyTools;
import us.ihmc.jOctoMap.tools.JOctoMapGeometryTools.RayBoxIntersectionResult;

public class OcTreeBoundingBoxWithCenterAndYaw implements OcTreeBoundingBoxInterface
{
   private final Vector3d offsetMetric = new Vector3d();
   private final OcTreeSimpleBoundingBox simpleBoundingBox = new OcTreeSimpleBoundingBox();
   private double yaw = 0.0, sinYaw = 0.0, cosYaw = 1.0;
   private OcTreeKey offsetKey = new OcTreeKey();
   private int centerOffsetKey;

   private boolean offsetMetricDirtyBit = true;
   private boolean offsetKeyDirtyBit = true;

   public OcTreeBoundingBoxWithCenterAndYaw()
   {
   }

   public OcTreeBoundingBoxWithCenterAndYaw(OcTreeBoundingBoxWithCenterAndYaw other)
   {
      set(other);
   }

   public OcTreeBoundingBoxWithCenterAndYaw(Point3d minCoordinate, Point3d maxCoordinate, double resolution, int treeDepth)
   {
      setLocalMinMaxCoordinates(minCoordinate, maxCoordinate);
      update(resolution, treeDepth);
   }

   public OcTreeBoundingBoxWithCenterAndYaw(OcTreeKeyReadOnly minKey, OcTreeKeyReadOnly maxKey, double resolution, int treeDepth)
   {
      setLocalMinMaxKeys(minKey, maxKey);
      update(resolution, treeDepth);
   }

   public OcTreeBoundingBoxWithCenterAndYaw(OcTreeSimpleBoundingBox simpleBoundingBox, double resolution, int treeDepth)
   {
      setLocalBoundingBox(simpleBoundingBox, resolution, treeDepth);
      update(resolution, treeDepth);
   }

   public void setLocalBoundingBox(OcTreeSimpleBoundingBox simpleBoundingBox, double resolution, int treeDepth)
   {
      setLocalBoundingBox(simpleBoundingBox);
      simpleBoundingBox.update(resolution, treeDepth);
   }

   public void setLocalBoundingBox(OcTreeSimpleBoundingBox simpleBoundingBox)
   {
      this.simpleBoundingBox.set(simpleBoundingBox);
   }

   public void set(OcTreeBoundingBoxWithCenterAndYaw other)
   {
      offsetMetric.set(other.offsetMetric);
      offsetKey.set(other.offsetKey);
      simpleBoundingBox.set(other.simpleBoundingBox);
      yaw = other.yaw;
      sinYaw = other.sinYaw;
      cosYaw = other.cosYaw;
      offsetMetricDirtyBit = other.offsetMetricDirtyBit;
      offsetKeyDirtyBit = other.offsetKeyDirtyBit;
   }

   public void setLocalMinX(double xMin)
   {
      simpleBoundingBox.setMinX(xMin);
   }

   public void setLocalMinY(double yMin)
   {
      simpleBoundingBox.setMinY(yMin);
   }

   public void setLocalMinZ(double zMin)
   {
      simpleBoundingBox.setMinZ(zMin);
   }

   public void setLocalMaxX(double xMax)
   {
      simpleBoundingBox.setMaxX(xMax);
   }
   
   public void setLocalMaxY(double yMax)
   {
      simpleBoundingBox.setMaxY(yMax);
   }
   
   public void setLocalMaxZ(double zMax)
   {
      simpleBoundingBox.setMaxZ(zMax);
   }

   public void setLocalMinMaxCoordinates(Point3d minCoordinate, Point3d maxCoordinate)
   {
      simpleBoundingBox.setMinMaxCoordinates(minCoordinate, maxCoordinate);
   }

   public void setLocalMinMaxCoordinates(double[] minCoordinate, double[] maxCoordinate)
   {
      simpleBoundingBox.setMinMaxCoordinates(minCoordinate, maxCoordinate);
   }

   public void setLocalMinMaxKeys(OcTreeKeyReadOnly minKey, OcTreeKeyReadOnly maxKey)
   {
      simpleBoundingBox.setMinMaxKeys(minKey, maxKey);
   }

   public void setOffset(Tuple3d offset, double resolution, int treeDepth)
   {
      setOffset(offset);
      update(resolution, treeDepth);
   }

   public void setOffset(Tuple3d offset)
   {
      offsetMetric.set(offset);
      offsetMetricDirtyBit = false;
      offsetKeyDirtyBit = true;
   }

   public void setOffset(OcTreeKey offset, double resolution, int treeDepth)
   {
      setOffset(offset);
      update(resolution, treeDepth);
   }

   public void setOffset(OcTreeKey offset)
   {
      offsetKey.set(offset);
      offsetKeyDirtyBit = false;
      offsetMetricDirtyBit = true;
   }

   public void setHalfSize(Vector3d halfSize)
   {
      simpleBoundingBox.setMinCoordinate(-halfSize.getX(), -halfSize.getY(), -halfSize.getZ());
      simpleBoundingBox.setMaxCoordinate(halfSize.getX(), halfSize.getY(), halfSize.getZ());
   }

   public void setHalfSize(double[] halfSize)
   {
      simpleBoundingBox.setMinCoordinate(-halfSize[0], -halfSize[1], -halfSize[2]);
      simpleBoundingBox.setMaxCoordinate(halfSize[0], halfSize[1], halfSize[2]);
   }

   public void setYaw(double yaw)
   {
      this.yaw = yaw;
      sinYaw = Math.sin(yaw);
      cosYaw = Math.cos(yaw);
   }

   public void setYawFromQuaternion(Quat4d quaternion)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getW();
      double yaw = FastMath.atan2(2.0 * (qx * qy + qz * qs), 1.0 - 2.0 * (qy * qy + qz * qz));
      setYaw(yaw);
   }

   public void update(double resolution, int treeDepth)
   {
      if (offsetKeyDirtyBit)
      {
         boolean success = OcTreeKeyConversionTools.coordinateToKey(offsetMetric, resolution, treeDepth, offsetKey);
         if (!success)
            System.err.println(getClass().getSimpleName() + " (in update): ERROR while generating offset key.");
      }
      else if (offsetMetricDirtyBit)
      {
         OcTreeKeyConversionTools.keyToCoordinate(offsetKey, offsetMetric, resolution, treeDepth);
      }

      offsetMetricDirtyBit = false;
      offsetKeyDirtyBit = false;

      centerOffsetKey = OcTreeKeyTools.computeCenterOffsetKey(treeDepth);

      simpleBoundingBox.update(resolution, treeDepth);
   }

   @Override
   public boolean isInBoundingBox(double x, double y, double z)
   {
      if (offsetMetricDirtyBit)
         throw new RuntimeException("The bounding box offset coordinate is not up to date.");

      double xLocal = (x - offsetMetric.getX()) * cosYaw + (y - offsetMetric.getY()) * sinYaw;
      double yLocal = -(x - offsetMetric.getX()) * sinYaw + (y - offsetMetric.getY()) * cosYaw;
      double zLocal = z - offsetMetric.getZ();

      return simpleBoundingBox.isInBoundingBox(xLocal, yLocal, zLocal);
   }

   @Override
   public boolean isInBoundingBox(int k0, int k1, int k2)
   {
      if (offsetKeyDirtyBit)
         throw new RuntimeException("The bounding box offset key is not up to date.");

      int k0Local = (int) ((k0 - offsetKey.getKey(0)) * cosYaw + (k1 - offsetKey.getKey(1)) * sinYaw + centerOffsetKey);
      int k1Local = (int) (-(k0 - offsetKey.getKey(0)) * sinYaw + (k1 - offsetKey.getKey(1)) * cosYaw + centerOffsetKey);
      int k2Local = k2 - offsetKey.getKey(2) + centerOffsetKey;

      return simpleBoundingBox.isInBoundingBox(k0Local, k1Local, k2Local);
   }

   @Override
   public RayBoxIntersectionResult rayIntersection(Point3d rayOrigin, Vector3d rayDirection, double maxRayLength)
   {
      if (offsetMetricDirtyBit)
         throw new RuntimeException("The bounding box offset coordinate is not up to date.");

      double xLocal = (rayOrigin.getX() - offsetMetric.getX()) * cosYaw + (rayOrigin.getY() - offsetMetric.getY()) * sinYaw;
      double yLocal = -(rayOrigin.getX() - offsetMetric.getX()) * sinYaw + (rayOrigin.getY() - offsetMetric.getY()) * cosYaw;
      double zLocal = rayOrigin.getZ() - offsetMetric.getZ();
      Point3d rayOriginLocal = new Point3d(xLocal, yLocal, zLocal);

      double vxLocal = rayDirection.getX() * cosYaw + rayDirection.getY() * sinYaw;
      double vyLocal = -rayDirection.getX() * sinYaw + rayDirection.getY() * cosYaw;
      double vzLocal = rayDirection.getZ();
      Vector3d rayDirectionLocal = new Vector3d(vxLocal, vyLocal, vzLocal);

      RayBoxIntersectionResult rayIntersection = simpleBoundingBox.rayIntersection(rayOriginLocal, rayDirectionLocal, maxRayLength);

      if (rayIntersection == null)
         return null;

      Point3d entry = rayIntersection.getEnteringIntersection();
      if (entry != null)
      {
         double xWorld = entry.getX() * cosYaw - entry.getY() * sinYaw + offsetMetric.getX();
         double yWorld = entry.getX() * sinYaw + entry.getY() * cosYaw + offsetMetric.getY();
         entry.setX(xWorld);
         entry.setY(yWorld);
         entry.setZ(entry.getZ() + offsetMetric.getZ());
      }

      Point3d exit = rayIntersection.getExitingIntersection();
      if (exit != null)
      {
         double xWorld = exit.getX() * cosYaw - exit.getY() * sinYaw + offsetMetric.getX();
         double yWorld = exit.getX() * sinYaw + exit.getY() * cosYaw + offsetMetric.getY();
         exit.setX(xWorld);
         exit.setY(yWorld);
         exit.setZ(exit.getZ() + offsetMetric.getZ());
      }

      return rayIntersection;
   }

   @Override
   public OcTreeBoundingBoxWithCenterAndYaw getCopy()
   {
      return new OcTreeBoundingBoxWithCenterAndYaw(this);
   }

   public OcTreeKeyReadOnly getLocalMinKey()
   {
      return simpleBoundingBox.getMinKey();
   }

   public OcTreeKeyReadOnly getLocalMaxKey()
   {
      return simpleBoundingBox.getMaxKey();
   }

   public void getLocalMinCoordinate(Point3d minCoordinateToPack)
   {
      simpleBoundingBox.getMinCoordinate(minCoordinateToPack);
   }

   public void getLocalMaxCoordinate(Point3d maxCoordinateToPack)
   {
      simpleBoundingBox.getMaxCoordinate(maxCoordinateToPack);
   }

   public void getLocalMinCoordinate(double[] minCoordinateToPack)
   {
      simpleBoundingBox.getMinCoordinate(minCoordinateToPack);
   }

   public void getLocalMaxCoordinate(double[] maxCoordinateToPack)
   {
      simpleBoundingBox.getMaxCoordinate(maxCoordinateToPack);
   }

   public void getLocalSize(Vector3d size)
   {
      simpleBoundingBox.getSize(size);
   }

   public void getCenterCoordinate(Point3d centerToPack)
   {
      simpleBoundingBox.getCenterCoordinate(centerToPack);
      double xWorld = centerToPack.getX() * cosYaw - centerToPack.getY() * sinYaw;
      double yWorld = centerToPack.getX() * sinYaw + centerToPack.getY() * cosYaw;
      double zWorld = centerToPack.getZ();
      centerToPack.set(xWorld, yWorld, zWorld);
      centerToPack.add(offsetMetric);
   }

   public void getLocalCenterCoordinate(Point3d centerToPack)
   {
      simpleBoundingBox.getCenterCoordinate(centerToPack);
      double xLocal = offsetMetric.getX() * cosYaw + offsetMetric.getY() * sinYaw;
      double yLocal = -offsetMetric.getX() * sinYaw + offsetMetric.getY() * cosYaw;
      double zLocal = offsetMetric.getZ();
      centerToPack.setX(centerToPack.getX() + xLocal);
      centerToPack.setY(centerToPack.getY() + yLocal);
      centerToPack.setZ(centerToPack.getZ() + zLocal);
   }

   public double getYaw()
   {
      return yaw;
   }
}
