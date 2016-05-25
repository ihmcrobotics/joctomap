package us.ihmc.octoMap.pointCloud;

import us.ihmc.robotics.geometry.RigidBodyTransform;

/**
 * A connection between two \ref ScanNode "ScanNodes"
 */
public class ScanEdge
{
   private final ScanNode first;
   private final ScanNode second;
   private final RigidBodyTransform constraint = new RigidBodyTransform();
   private double weight;

   public ScanEdge(ScanNode first, ScanNode second, RigidBodyTransform constraint)
   {
      this.first = new ScanNode(first);
      this.second = new ScanNode(second);
      this.constraint.set(constraint);
      weight = 1.0;
   }

   public ScanEdge()
   {
      first = new ScanNode();
      second = new ScanNode();
   }

   public double getWeight()
   {
      return weight;
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   public ScanNode getFirst()
   {
      return first;
   }

   public ScanNode getSecond()
   {
      return second;
   }

   public RigidBodyTransform getConstraint()
   {
      return constraint;
   }

   public boolean equals(ScanEdge other)
   {
      return first.equals(other.first) && second.equals(other.second);
   }
}