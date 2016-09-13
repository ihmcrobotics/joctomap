package us.ihmc.octoMap.ocTree.baseImplementation;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;

import us.ihmc.octoMap.key.OcTreeKeyReadOnly;

public interface OcTreeBoundingBoxInterface
{

   boolean isInBoundingBox(double x, double y, double z);

   boolean isInBoundingBox(Point3d candidate);

   boolean isInBoundingBox(Point3f candidate);

   boolean isInBoundingBox(OcTreeKeyReadOnly candidate);

   OcTreeBoundingBoxInterface getCopy();
}