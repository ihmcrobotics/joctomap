package us.ihmc.octoMap.rules.interfaces;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.key.OcTreeKeyReadOnly;

public interface RayActionRule
{
   public void doAction(Point3d rayOrigin, Point3d rayEnd, Vector3d rayDirection, OcTreeKeyReadOnly key);
}
