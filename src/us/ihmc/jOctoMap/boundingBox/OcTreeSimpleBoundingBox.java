package us.ihmc.jOctoMap.boundingBox;

import java.util.Scanner;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;

import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.tools.JOctoMapGeometryTools;
import us.ihmc.jOctoMap.tools.JOctoMapGeometryTools.RayBoxIntersectionResult;
import us.ihmc.jOctoMap.tools.OcTreeKeyConversionTools;

public class OcTreeSimpleBoundingBox implements OcTreeBoundingBoxInterface
{
   private final Point3d minCoordinate = new Point3d();
   private final Point3d maxCoordinate = new Point3d();
   private final OcTreeKey minKey = new OcTreeKey();
   private final OcTreeKey maxKey = new OcTreeKey();

   private boolean minCoordinateDirtyBit = false;
   private boolean maxCoordinateDirtyBit = false;
   private boolean minKeyDirtyBit = false;
   private boolean maxKeyDirtyBit = false;

   public OcTreeSimpleBoundingBox()
   {
   }

   public OcTreeSimpleBoundingBox(OcTreeSimpleBoundingBox other)
   {
      set(other);
   }

   public OcTreeSimpleBoundingBox(Point3d minCoordinate, Point3d maxCoordinate, double resolution, int treeDepth)
   {
      setMinMaxCoordinates(minCoordinate, maxCoordinate);
      update(resolution, treeDepth);
   }

   public OcTreeSimpleBoundingBox(Point3d minCoordinate, Point3d maxCoordinate)
   {
      setMinMaxCoordinates(minCoordinate, maxCoordinate);
   }

   public OcTreeSimpleBoundingBox(double[] minCoordinate, double[] maxCoordinate)
   {
      setMinMaxCoordinates(minCoordinate, maxCoordinate);
   }

   public OcTreeSimpleBoundingBox(OcTreeKeyReadOnly minKey, OcTreeKeyReadOnly maxKey)
   {
      setMinMaxKeys(minKey, maxKey);
   }

   public void set(OcTreeSimpleBoundingBox other)
   {
      minCoordinate.set(other.minCoordinate);
      maxCoordinate.set(other.maxCoordinate);
      minKey.set(other.minKey);
      maxKey.set(other.maxKey);

      minCoordinateDirtyBit = other.minCoordinateDirtyBit;
      maxCoordinateDirtyBit = other.maxCoordinateDirtyBit;
      minKeyDirtyBit = other.minKeyDirtyBit;
      maxKeyDirtyBit = other.maxKeyDirtyBit;
   }

   public void setMinX(double xMin)
   {
      minCoordinate.setX(xMin);
      minCoordinateDirtyBit = false;
      minKeyDirtyBit = true;
   }

   public void setMinY(double yMin)
   {
      minCoordinate.setY(yMin);
      minCoordinateDirtyBit = false;
      minKeyDirtyBit = true;
   }

   public void setMinZ(double zMin)
   {
      minCoordinate.setZ(zMin);
      minCoordinateDirtyBit = false;
      minKeyDirtyBit = true;
   }

   public void setMaxX(double xMax)
   {
      maxCoordinate.setX(xMax);
      maxCoordinateDirtyBit = false;
      maxKeyDirtyBit = true;
   }
   
   public void setMaxY(double yMax)
   {
      maxCoordinate.setY(yMax);
      maxCoordinateDirtyBit = false;
      maxKeyDirtyBit = true;
   }
   
   public void setMaxZ(double zMax)
   {
      maxCoordinate.setZ(zMax);
      maxCoordinateDirtyBit = false;
      maxKeyDirtyBit = true;
   }

   public void setMinCoordinate(double xMin, double yMin, double zMin)
   {
      minCoordinate.set(xMin, yMin, zMin);
      minCoordinateDirtyBit = false;
      minKeyDirtyBit = true;
   }

   public void setMaxCoordinate(double xMax, double yMax, double zMax)
   {
      maxCoordinate.set(xMax, yMax, zMax);
      maxCoordinateDirtyBit = false;
      maxKeyDirtyBit = true;
   }

   public void setMinMaxCoordinates(double xMin, double yMin, double zMin, double xMax, double yMax, double zMax)
   {
      setMinCoordinate(xMin, yMin, zMin);
      setMaxCoordinate(xMax, yMax, zMax);
   }

   public void setMinCoordinate(Point3d minCoordinate)
   {
      setMinCoordinate(minCoordinate.getX(), minCoordinate.getY(), minCoordinate.getZ());
   }

   public void setMaxCoordinate(Point3d maxCoordinate)
   {
      setMaxCoordinate(maxCoordinate.getX(), maxCoordinate.getY(), maxCoordinate.getZ());
   }

   public void setMinMaxCoordinates(Point3d minCoordinate, Point3d maxCoordinate)
   {
      setMinCoordinate(minCoordinate);
      setMaxCoordinate(maxCoordinate);
   }

   public void setMinCoordinate(double[] minCoordinate)
   {
      setMinCoordinate(minCoordinate[0], minCoordinate[1], minCoordinate[2]);
   }

   public void setMaxCoordinate(double[] maxCoordinate)
   {
      setMaxCoordinate(maxCoordinate[0], maxCoordinate[1], maxCoordinate[2]);
   }

   public void setMinMaxCoordinates(double[] minCoordinate, double[] maxCoordinate)
   {
      setMinCoordinate(minCoordinate);
      setMaxCoordinate(maxCoordinate);
   }

   public void setMinCoordinate(Point3f minCoordinate)
   {
      setMinCoordinate(minCoordinate.getX(), minCoordinate.getY(), minCoordinate.getZ());
   }

   public void setMaxCoordinate(Point3f maxCoordinate)
   {
      setMaxCoordinate(maxCoordinate.getX(), maxCoordinate.getY(), maxCoordinate.getZ());
   }

   public void setMinMaxCoordinates(Point3f minCoordinate, Point3f maxCoordinate)
   {
      setMinCoordinate(minCoordinate);
      setMaxCoordinate(maxCoordinate);
   }

   public void setMinKey(OcTreeKeyReadOnly minKey)
   {
      this.minKey.set(minKey);
      minKeyDirtyBit = false;
      minCoordinateDirtyBit = true;
   }

   public void setMaxKey(OcTreeKeyReadOnly maxKey)
   {
      this.maxKey.set(maxKey);
      maxKeyDirtyBit = false;
      maxCoordinateDirtyBit = true;
   }

   public void setMinMaxKeys(OcTreeKeyReadOnly minKey, OcTreeKeyReadOnly maxKey)
   {
      setMinKey(minKey);
      setMaxKey(maxKey);
   }

   public void update(double resolution, int treeDepth)
   {
      if (minKeyDirtyBit)
      {
         boolean success = OcTreeKeyConversionTools.coordinateToKey(minCoordinate, resolution, treeDepth, minKey);
         if (!success)
            System.err.println(getClass().getSimpleName() + " (in update): ERROR while generating min key.");
      }
      else if (minCoordinateDirtyBit)
      {
         OcTreeKeyConversionTools.keyToCoordinate(minKey, minCoordinate, resolution, treeDepth);
      }

      if (maxKeyDirtyBit)
      {
         boolean success = OcTreeKeyConversionTools.coordinateToKey(maxCoordinate, resolution, treeDepth, maxKey);
         if (!success)
            System.err.println(getClass().getSimpleName() + " (in update): ERROR while generating max key.");
      }
      else if (maxCoordinateDirtyBit)
      {
         OcTreeKeyConversionTools.keyToCoordinate(maxKey, maxCoordinate, resolution, treeDepth);
      }

      minCoordinateDirtyBit = false;
      minKeyDirtyBit = false;
      maxCoordinateDirtyBit = false;
      maxKeyDirtyBit = false;
   }

   @Override
   public boolean isInBoundingBox(double x, double y, double z)
   {
      if (minCoordinateDirtyBit || maxCoordinateDirtyBit)
         throw new RuntimeException("The bounding box coordinates are not up to date.");

      if (x < minCoordinate.getX() || x > maxCoordinate.getX())
         return false;
      if (y < minCoordinate.getY() || y > maxCoordinate.getY())
         return false;
      if (z < minCoordinate.getZ() || z > maxCoordinate.getZ())
         return false;
      return true;
   }

   @Override
   public boolean isInBoundingBox(int k0, int k1, int k2)
   {
      if (minKeyDirtyBit || maxKeyDirtyBit)
         throw new RuntimeException("The bounding box keys are not up to date.");

      if (k0 < minKey.getKey(0) || k0 > maxKey.getKey(0))
         return false;
      if (k1 < minKey.getKey(1) || k1 > maxKey.getKey(1))
         return false;
      if (k2 < minKey.getKey(2) || k2 > maxKey.getKey(2))
         return false;
      return true;
   }

   @Override
   public RayBoxIntersectionResult rayIntersection(Point3d rayOrigin, Vector3d rayDirection, double maxRayLength)
   {
      if (minCoordinateDirtyBit || maxCoordinateDirtyBit)
         throw new RuntimeException("The bounding box coordinates are not up to date.");
      return JOctoMapGeometryTools.rayBoxIntersection(minCoordinate, maxCoordinate, rayOrigin, rayDirection, maxRayLength);
   }

   public OcTreeKeyReadOnly getMinKey()
   {
      if (minKeyDirtyBit)
         throw new RuntimeException("The bounding box min key is not up to date.");
      return minKey;
   }

   public OcTreeKeyReadOnly getMaxKey()
   {
      if (maxKeyDirtyBit)
         throw new RuntimeException("The bounding box max key is not up to date.");
      return maxKey;
   }

   public void getMinCoordinate(Point3d minCoordinateToPack)
   {
      if (minCoordinateDirtyBit)
         throw new RuntimeException("The bounding box min coordinate is not up to date.");
      minCoordinateToPack.set(minCoordinate);
   }

   public void getMaxCoordinate(Point3d maxCoordinateToPack)
   {
      if (maxCoordinateDirtyBit)
         throw new RuntimeException("The bounding box max coordinate is not up to date.");
      maxCoordinateToPack.set(maxCoordinate);
   }

   public void getMinCoordinate(double[] minCoordinateToPack)
   {
      if (minCoordinateDirtyBit)
         throw new RuntimeException("The bounding box min coordinate is not up to date.");
      minCoordinate.get(minCoordinateToPack);
   }

   public void getMaxCoordinate(double[] maxCoordinateToPack)
   {
      if (maxCoordinateDirtyBit)
         throw new RuntimeException("The bounding box max coordinate is not up to date.");
      maxCoordinate.get(maxCoordinateToPack);
   }

   public double getMinX()
   {
      if (minCoordinateDirtyBit)
         throw new RuntimeException("The bounding box min coordinate is not up to date.");
      return minCoordinate.getX();
   }

   public double getMinY()
   {
      if (minCoordinateDirtyBit)
         throw new RuntimeException("The bounding box min coordinate is not up to date.");
      return minCoordinate.getY();
   }

   public double getMinZ()
   {
      if (minCoordinateDirtyBit)
         throw new RuntimeException("The bounding box min coordinate is not up to date.");
      return minCoordinate.getZ();
   }

   public double getMaxX()
   {
      if (maxCoordinateDirtyBit)
         throw new RuntimeException("The bounding box max coordinate is not up to date.");
      return maxCoordinate.getX();
   }

   public double getMaxY()
   {
      if (maxCoordinateDirtyBit)
         throw new RuntimeException("The bounding box max coordinate is not up to date.");
      return maxCoordinate.getY();
   }

   public double getMaxZ()
   {
      if (maxCoordinateDirtyBit)
         throw new RuntimeException("The bounding box max coordinate is not up to date.");
      return maxCoordinate.getZ();
   }

   public void getSize(Vector3d sizeToPack)
   {
      if (minCoordinateDirtyBit || maxCoordinateDirtyBit)
         throw new RuntimeException("The bounding box min/max coordinate is not up to date.");

      sizeToPack.sub(maxCoordinate, minCoordinate);
   }

   public void getCenterCoordinate(Point3d centerToPack)
   {
      if (minCoordinateDirtyBit || maxCoordinateDirtyBit)
         throw new RuntimeException("The bounding box min/max coordinate is not up to date.");

      centerToPack.interpolate(maxCoordinate, minCoordinate, 0.5);
   }

   @Override
   public OcTreeSimpleBoundingBox getCopy()
   {
      return new OcTreeSimpleBoundingBox(this);
   }

   @Override
   public String toString()
   {
      return "min: " + minCoordinate + ", max: " + maxCoordinate;
   }

   public static OcTreeSimpleBoundingBox parse(String boundingBoxAsString)
   {
      boundingBoxAsString = boundingBoxAsString.replace("(", "").replace(")", " ").replace(",", "");
      Scanner scanner = new Scanner(boundingBoxAsString);
      scanner.next();
      double minX = scanner.nextDouble();
      double minY = scanner.nextDouble();
      double minZ = scanner.nextDouble();
      scanner.next();
      double maxX = scanner.nextDouble();
      double maxY = scanner.nextDouble();
      double maxZ = scanner.nextDouble();
      scanner.close();
      OcTreeSimpleBoundingBox boundingBox = new OcTreeSimpleBoundingBox();
      boundingBox.setMinCoordinate(minX, minY, minZ);
      boundingBox.setMaxCoordinate(maxX, maxY, maxZ);
      return boundingBox;
   }
}
