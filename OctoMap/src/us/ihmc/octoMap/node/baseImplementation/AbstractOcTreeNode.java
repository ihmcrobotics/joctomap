package us.ihmc.octoMap.node.baseImplementation;

import static us.ihmc.octoMap.tools.OcTreeNodeTools.*;

import java.lang.reflect.Array;
import java.util.Arrays;

import javax.vecmath.Point3d;

import org.apache.commons.lang3.mutable.MutableInt;
import org.apache.commons.math3.util.Precision;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.NodeBuilder;
import us.ihmc.octoMap.tools.OcTreeKeyConversionTools;
import us.ihmc.octoMap.tools.OcTreeNodeTools;

public abstract class AbstractOcTreeNode<N extends AbstractOcTreeNode<N>>
{
   private static final boolean DEBUG_PROPERTIES = true;

   protected N[] children;
   private int k0 = -1, k1 = -1, k2 = -1;
   private float x = Float.NaN, y = Float.NaN, z = Float.NaN;
   private float size = Float.NaN;
   private int depth;

   public AbstractOcTreeNode()
   {
   }

   public abstract void copyData(N other);

   protected abstract void clear();

   private final void clearProperties()
   {
      k0 = -1;
      k1 = -1;
      k2 = -1;
      x = Float.NaN;
      y = Float.NaN;
      z = Float.NaN;
      size = Float.NaN;
      depth = -1;
   }

   public final void setProperties(OcTreeKeyReadOnly key, int depth, double resolution, int treeDepth)
   {
      setProperties(key.getKey(0), key.getKey(1), key.getKey(2), depth, resolution, treeDepth);
   }

   public final void setProperties(int k0, int k1, int k2, int depth, double resolution, int treeDepth)
   {
      this.k0 = k0;
      this.k1 = k1;
      this.k2 = k2;
      this.x = (float) OcTreeKeyConversionTools.keyToCoordinate(k0, depth, resolution, treeDepth);
      this.y = (float) OcTreeKeyConversionTools.keyToCoordinate(k1, depth, resolution, treeDepth);
      this.z = (float) OcTreeKeyConversionTools.keyToCoordinate(k2, depth, resolution, treeDepth);
      this.size = (float) OcTreeKeyConversionTools.computeNodeSize(depth, resolution, treeDepth);
      this.depth = depth;
   }

   public final void getKey(OcTreeKey keyToPack)
   {
      keyToPack.set(k0, k1, k2);
   }

   public final void getCoordinate(Point3d coordinateToPack)
   {
      coordinateToPack.set(x, y, z);
   }

   @SuppressWarnings("unchecked")
   public void allocateChildren()
   {
      children = (N[]) Array.newInstance(getClass(), 8);
   }

   public final void assignChildren(N[] newChildren)
   {
      children = newChildren;
   }

   @SuppressWarnings("unchecked")
   public final N cloneRecursive(NodeBuilder<N> nodeBuilder, MutableInt treeSize)
   {
      N ret = nodeBuilder.createNode();
      treeSize.increment();
      ret.copyData((N) this);

      AbstractOcTreeNode<?> retCasted = ret;
      retCasted.k0 = k0;
      retCasted.k1 = k1;
      retCasted.k2 = k2;
      retCasted.x = x;
      retCasted.y = y;
      retCasted.z = z;
      retCasted.size = size;

      if (hasArrayForChildren())
      {
         if (!ret.hasArrayForChildren())
            ret.allocateChildren();

         for (int i = 0; i < 8; i++)
         {
            if (children[i] != null)
               ret.children[i] = children[i].cloneRecursive(nodeBuilder, treeSize);
         }
      }

      return ret;
   }

   public final boolean hasArrayForChildren()
   {
      return children != null;
   }

   public final boolean hasAtLeastOneChild()
   {
      if (children == null)
         return false;

      for (int i = 0; i < 8; i++)
      {
         if (children[i] != null)
            return true;
      }
      return false;
   }

   public final void setChild(int childIndex, N newChild)
   {
      checkChildIndex(childIndex);
      if (!getClass().isInstance(newChild))
         throw new RuntimeException("Cannot add a child of a different type");
      children[childIndex] = newChild;
   }

   public final int getNumberOfNonNullChildren()
   {
      if (!hasAtLeastOneChild())
         return 0;
      int number = 0;
      for (N child : children)
      {
         if (child != null)
            number++;
      }
      return number;
   }

   public final N getChild(int childIndex)
   {
      checkChildIndex(childIndex);
      return children == null ? null : children[childIndex];
   }

   public final N removeChild(int childIndex)
   {
      OcTreeNodeTools.checkChildIndex(childIndex);

      N removedChild = children[childIndex];
      if (removedChild != null)
      {
         removedChild.clear();
         ((AbstractOcTreeNode<?>) removedChild).clearProperties();
      }
      children[childIndex] = null;
      return removedChild;
   }

   public final N[] removeChildren()
   {
      N[] removedChildren = children;
      children = null;
      return removedChildren;
   }

   public final int getKey0()
   {
      if (DEBUG_PROPERTIES)
         if (k0 == -1)
            throw new RuntimeException("Key has not been set");
      return k0;
   }

   public final int getKey1()
   {
      if (DEBUG_PROPERTIES)
         if (k1 == -1)
            throw new RuntimeException("Key has not been set");
      return k1;
   }

   public final int getKey2()
   {
      if (DEBUG_PROPERTIES)
         if (k2 == -1)
            throw new RuntimeException("Key has not been set");
      return k2;
   }

   public final double getX()
   {
      if (DEBUG_PROPERTIES)
         if (Float.isNaN(x))
            throw new RuntimeException("Coordinate has not been set");
      return x;
   }

   public final double getY()
   {
      if (DEBUG_PROPERTIES)
         if (Float.isNaN(y))
            throw new RuntimeException("Coordinate has not been set");
      return y;
   }

   public final double getZ()
   {
      if (DEBUG_PROPERTIES)
         if (Float.isNaN(z))
            throw new RuntimeException("Coordinate has not been set");
      return z;
   }

   public final double getSize()
   {
      if (DEBUG_PROPERTIES)
         if (Float.isNaN(size))
            throw new RuntimeException("Size has not been set");
      return size;
   }

   public final int getDepth()
   {
      return depth;
   }

   public final boolean epsilonEquals(N other, double epsilon)
   {
      AbstractOcTreeNode<?> otherCasted = other;
      if (k0 != otherCasted.k0)
         return false;
      if (k1 != otherCasted.k1)
         return false;
      if (k2 != otherCasted.k2)
         return false;
      if (!Precision.equals(x, otherCasted.x, 1.0e-7))
         return false;
      if (!Precision.equals(y, otherCasted.y, 1.0e-7))
         return false;
      if (!Precision.equals(z, otherCasted.z, 1.0e-7))
         return false;
      if (!Precision.equals(size, otherCasted.size, 1.0e-7))
         return false;
      return epsilonEqualsInternal(other, epsilon);
   }

   protected abstract boolean epsilonEqualsInternal(N other, double epsilon);

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": children = " + Arrays.toString(getChildrenSimpleNames());
   }

   protected String[] getChildrenSimpleNames()
   {
      String[] childrenNames = new String[8];
      if (children != null)
      {
         for (int i = 0; i < 8; i++)
         {
            N child = children[i];
            childrenNames[i] = child == null ? null : child.getClass().getSimpleName();
         }
      }
      return childrenNames;
   }
}
