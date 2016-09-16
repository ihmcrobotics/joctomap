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

   boolean isInBoundingBox(OcTreeKeyReadOnly candidate);

   OcTreeBoundingBoxInterface getCopy();
}