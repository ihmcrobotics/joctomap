package us.ihmc.octoMap.iterators;

import javax.vecmath.Point3d;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.AbstractOcTreeNode;

public class OcTreeSuperNode<NODE extends AbstractOcTreeNode<NODE>>
{
   private NODE node;
   private int depth = -1;
   private int maxDepth = -1;

   public OcTreeSuperNode()
   {
      clear();
   }

   public void clear()
   {
      node = null;
      depth = -1;
      maxDepth = -1;
   }

   public void setAsRootSuperNode(NODE root, int maxDepth)
   {
      this.maxDepth = maxDepth;

      node = root;
      depth = 0;
   }

   public void setAsChildSuperNode(OcTreeSuperNode<NODE> parentNode, int childIndex)
   {
      this.maxDepth = parentNode.maxDepth;
      this.depth = parentNode.depth + 1;
      node = parentNode.node.getChild(childIndex);
   }

   /** @return the center coordinate of this node */
   public Point3d getCoordinate()
   {
      return new Point3d(getX(), getY(), getZ());
   }

   /** @return single coordinate of this node */
   public double getX()
   {
      return node.getX();
   }

   /** @return single coordinate of this node */
   public double getY()
   {
      return node.getY();
   }

   /** @return single coordinate of this node */
   public double getZ()
   {
      return node.getZ();
   }

   /** @return the side of the volume occupied by this node */
   public double getSize()
   {
      return node.getSize();
   }

   /** return depth of this node */
   public int getDepth()
   {
      return depth;
   }

   /** @return the OcTreeKey of this node */
   public OcTreeKeyReadOnly getKey()
   {
      return new OcTreeKey(getKey0(), getKey1(), getKey2());
   }

   public int getKey0()
   {
      return node.getKey0();
   }

   public int getKey1()
   {
      return node.getKey1();
   }

   public int getKey2()
   {
      return node.getKey2();
   }

   /** @return the NODE contained in this. */
   public NODE getNode()
   {
      return node;
   }

   /** @return whether the current node is a leaf, i.e. has no children or is at max level */
   public boolean isLeaf()
   {
      return !node.hasAtLeastOneChild() || depth == maxDepth;
   }

   public boolean epsilonEquals(OcTreeSuperNode<NODE> other, double epsilon)
   {
      if (!node.epsilonEquals(other.node, epsilon))
         return false;
      return depth == other.depth;
   }
}