package us.ihmc.octoMap.boundingBox;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;

import us.ihmc.octoMap.key.OcTreeKeyReadOnly;

public interface OcTreeBoundingBoxInterface
{

   boolean isInBoundingBox(double x, double y, double z);

   default boolean isInBoundingBox(Point3d candidate)
   {
      return isInBoundingBox(candidate.getX(), candidate.getY(), candidate.getZ());
   }

   default boolean isInBoundingBox(Point3f candidate)
   {
      return isInBoundingBox(candidate.getX(), candidate.getY(), candidate.getZ());
   }

   default boolean isInBoundingBox(OcTreeKeyReadOnly candidate)
   {
      return isInBoundingBox(candidate.getKey(0), candidate.getKey(1), candidate.getKey(2));
   }

   boolean isInBoundingBox(int k0, int k1, int k2);

   OcTreeBoundingBoxInterface getCopy();
}