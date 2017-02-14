package us.ihmc.jOctoMap.rules.interfaces;

import us.ihmc.geometry.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;

public interface RayActionRule
{
   public void doAction(Point3DReadOnly rayOrigin, Point3DReadOnly rayEnd, Vector3DReadOnly rayDirection, OcTreeKeyReadOnly key);
}
