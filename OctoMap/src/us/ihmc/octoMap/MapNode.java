package us.ihmc.octoMap;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class MapNode<TreeType>
{
   /** occupancy grid map */
   private TreeType nodeMap;
   private final RigidBodyTransform origin = new RigidBodyTransform();
   private String id;

   public MapNode()
   {
   }

   public MapNode(TreeType nodeMap, RigidBodyTransform origin)
   {
      this.nodeMap = nodeMap;
      this.origin.set(origin);
   }

   public MapNode(final Pointcloud cloud, RigidBodyTransform origin)
   {
   }

   public void clear()
   {
      
   }

   public Pointcloud generatePointcloud()
   {
      return null;
   }

   public void setId(String id)
   {
      this.id = id;
   }

   public TreeType getMap()
   {
      return nodeMap;
   }

   public RigidBodyTransform getOrigin()
   {
      return origin;
   }

   public String getId()
   {
      return id;
   }
}
