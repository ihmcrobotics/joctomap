package us.ihmc.jOctoMap.boundingBox;

import us.ihmc.geometry.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.tools.JOctoMapGeometryTools.RayBoxIntersectionResult;

public interface OcTreeBoundingBoxInterface
{
   boolean isInBoundingBox(double x, double y, double z);

   default boolean isInBoundingBox(Point3DReadOnly candidate)
   {
      return isInBoundingBox(candidate.getX(), candidate.getY(), candidate.getZ());
   }

   default boolean isInBoundingBox(OcTreeKeyReadOnly candidate)
   {
      return isInBoundingBox(candidate.getKey(0), candidate.getKey(1), candidate.getKey(2));
   }

   boolean isInBoundingBox(int k0, int k1, int k2);

   default RayBoxIntersectionResult rayIntersection(Point3DReadOnly rayOrigin, Vector3DReadOnly rayDirection)
   {
      return rayIntersection(rayOrigin, rayDirection, Double.POSITIVE_INFINITY);
   }

   RayBoxIntersectionResult rayIntersection(Point3DReadOnly rayOrigin, Vector3DReadOnly rayDirection, double maxRayLength);

   OcTreeBoundingBoxInterface getCopy();
}