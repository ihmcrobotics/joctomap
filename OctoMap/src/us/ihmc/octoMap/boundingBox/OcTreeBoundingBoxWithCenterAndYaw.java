package us.ihmc.octoMap.boundingBox;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.apache.commons.math3.util.FastMath;

import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.tools.OcTreeKeyConversionTools;

public class OcTreeBoundingBoxWithCenterAndYaw implements OcTreeBoundingBoxInterface
{
   private final Vector3d offsetMetric = new Vector3d();
   private final OcTreeSimpleBoundingBox simpleBoundingBox = new OcTreeSimpleBoundingBox();
   private double yaw = 0.0, sinYaw = 0.0, cosYaw = 1.0;

   private double resolution;
   private int treeDepth;

   public OcTreeBoundingBoxWithCenterAndYaw(double resolution, int treeDepth)
   {
      update(resolution, treeDepth);
   }

   public OcTreeBoundingBoxWithCenterAndYaw(OcTreeSimpleBoundingBox simpleBoundingBox, double resolution, int treeDepth)
   {
      setLocalBoundingBox(simpleBoundingBox);
      update(resolution, treeDepth);
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

   public void setLocalBoundingBox(OcTreeSimpleBoundingBox simpleBoundingBox)
   {
      simpleBoundingBox.update(resolution, treeDepth);
      this.simpleBoundingBox.set(simpleBoundingBox);
   }

   public void set(OcTreeBoundingBoxWithCenterAndYaw other)
   {
      offsetMetric.set(other.offsetMetric);
      simpleBoundingBox.set(other.simpleBoundingBox);
      yaw = other.yaw;
      sinYaw = other.sinYaw;
      cosYaw = other.cosYaw;
      resolution = other.resolution;
      treeDepth = other.treeDepth;
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

   public void setOffsetCoordinate(Point3d offset)
   {
      offsetMetric.set(offset);
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
      this.resolution = resolution;
      this.treeDepth = treeDepth;
      simpleBoundingBox.update(resolution, treeDepth);
   }

   @Override
   public boolean isInBoundingBox(double x, double y, double z)
   {
      double xLocal = (x - offsetMetric.getX()) * cosYaw + (y - offsetMetric.getY()) * sinYaw;
      double yLocal = -(x - offsetMetric.getX()) * sinYaw + (y - offsetMetric.getY()) * cosYaw;
      double zLocal = z - offsetMetric.getZ();

      return simpleBoundingBox.isInBoundingBox(xLocal, yLocal, zLocal);
   }

   private final Point3d transformedKey = new Point3d();

   @Override
   public boolean isInBoundingBox(OcTreeKeyReadOnly candidate)
   {
      OcTreeKeyConversionTools.keyToCoordinate(candidate, transformedKey, resolution, treeDepth);
      return isInBoundingBox(transformedKey);
   }
   
   //@Override
   public boolean trialIsInBoundingBox(OcTreeKeyReadOnly candidate)
   {
	   OcTreeKeyReadOnly minKey = simpleBoundingBox.getMinKey();
	   OcTreeKeyReadOnly maxKey = simpleBoundingBox.getMaxKey();
	   int offsetX = 40 * (int) offsetMetric.getX();
	   int offsetY = 40 * (int) offsetMetric.getY();
	   int offsetZ = 40 * (int) offsetMetric.getZ();
	   int referenceCorrection = 1 << (treeDepth - 1);
	   int[] array = new int[3];
	   
	   
	   array[0] = (int) ((candidate.getKey(0) - referenceCorrection - offsetX) * cosYaw + 
			   (candidate.getKey(1) - referenceCorrection - offsetY) * sinYaw + referenceCorrection);
	   array[1] = (int) (-(candidate.getKey(0) - referenceCorrection - offsetX) * sinYaw + 
			   (candidate.getKey(1) - referenceCorrection - offsetY) * cosYaw + referenceCorrection);
	   array[2] = (int) (candidate.getKey(2) - offsetZ);
	   
	   for (int i = 0; i < 3; i++)
	      {
	         if (array[i] < minKey.getKey(i) || array[i] > maxKey.getKey(i))
	            return false;
	      }
	      return true;
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
