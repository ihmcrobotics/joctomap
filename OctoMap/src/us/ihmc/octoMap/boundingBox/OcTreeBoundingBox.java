package us.ihmc.octoMap.boundingBox;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.tools.OcTreeKeyConversionTools;

public class OcTreeBoundingBox implements OcTreeBoundingBoxInterface
{
   private final Point3d minCoordinate = new Point3d();
   private final Point3d maxCoordinate = new Point3d();
   private final OcTreeKey minKey = new OcTreeKey();
   private final OcTreeKey maxKey = new OcTreeKey();

   private boolean minCoordinateDirtyBit = false;
   private boolean maxCoordinateDirtyBit = false;
   private boolean minKeyDirtyBit = false;
   private boolean maxKeyDirtyBit = false;

   public OcTreeBoundingBox()
   {
   }

   public OcTreeBoundingBox(OcTreeBoundingBox other)
   {
      set(other);
   }

   public OcTreeBoundingBox(Point3d minCoordinate, Point3d maxCoordinate, double resolution, int treeDepth)
   {
      setMinMaxCoordinates(minCoordinate, maxCoordinate);
      update(resolution, treeDepth);
   }

   public OcTreeBoundingBox(Point3d minCoordinate, Point3d maxCoordinate)
   {
      setMinMaxCoordinates(minCoordinate, maxCoordinate);
   }

   public OcTreeBoundingBox(double[] minCoordinate, double[] maxCoordinate)
   {
      setMinMaxCoordinates(minCoordinate, maxCoordinate);
   }

   public void set(OcTreeBoundingBox other)
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

   public void setMinCoordinate(double xMin, double yMin, double zMin)
   {
      this.minCoordinate.set(xMin, yMin, zMin);
      minKeyDirtyBit = true;
   }

   public void setMaxCoordinate(double xMax, double yMax, double zMax)
   {
      this.maxCoordinate.set(xMax, yMax, zMax);
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
      minCoordinateDirtyBit = true;
   }

   public void setMaxKey(OcTreeKeyReadOnly maxKey)
   {
      this.maxKey.set(maxKey);
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
            System.err.println(getClass().getSimpleName() + " (in setMinCoordinate): ERROR while generating min key.");
      }
      else if (minCoordinateDirtyBit)
      {
         OcTreeKeyConversionTools.keyToCoordinate(minKey, minCoordinate, resolution, treeDepth);
      }

      if (maxKeyDirtyBit)
      {
         boolean success = OcTreeKeyConversionTools.coordinateToKey(maxCoordinate, resolution, treeDepth, maxKey);
         if (!success)
            System.err.println(getClass().getSimpleName() + " (in setMaxCoordinate): ERROR while generating max key.");
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
   public boolean isInBoundingBox(Point3d candidate)
   {
      return isInBoundingBox(candidate.getX(), candidate.getY(), candidate.getZ());
   }

   @Override
   public boolean isInBoundingBox(Point3f candidate)
   {
      return isInBoundingBox(candidate.getX(), candidate.getY(), candidate.getZ());
   }

   @Override
   public boolean isInBoundingBox(OcTreeKeyReadOnly candidate)
   {
      if (minKeyDirtyBit || maxKeyDirtyBit)
         throw new RuntimeException("The bounding box keys are not up to date.");

      for (int i = 0; i < 3; i++)
      {
         if (candidate.getKey(i) < minKey.getKey(i) || candidate.getKey(i) > maxKey.getKey(i))
            return false;
      }
      return true;
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

   @Override
   public OcTreeBoundingBox getCopy()
   {
      return new OcTreeBoundingBox(this);
   }
}

