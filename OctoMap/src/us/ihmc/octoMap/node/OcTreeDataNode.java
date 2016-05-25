package us.ihmc.octoMap.node;

import java.util.Arrays;

public abstract class OcTreeDataNode<V>
{
   protected V value;
   protected OcTreeDataNode<V>[] children;

   public OcTreeDataNode()
   {
   }

   public OcTreeDataNode(V initialValue)
   {
      value = initialValue;
   }

   public OcTreeDataNode(OcTreeDataNode<V> other)
   {
      copyData(other);

      if (other.hasAtLeastOneChild())
         allocateChildren();

      for (int i = 0; i < 8; i++)
         children[i] = other.children[i].cloneRecursive();
   }

   public void setValue(V value)
   {
      this.value = value;
   }

   public V getValue()
   {
      return value;
   }

   public void copyData(OcTreeDataNode<V> other)
   {
      value = other.value;
   }

   public abstract void updateOccupancyChildren();

   public abstract void allocateChildren();

   public abstract OcTreeDataNode<V> cloneRecursive();

   public abstract OcTreeDataNode<V> create();

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

   public final void setChild(int childIndex, OcTreeDataNode<V> newChild)
   {
      OcTreeDataNode.checkChildIndex(childIndex);
      if (!getClass().isInstance(newChild))
         throw new RuntimeException("Cannot add a child of a different type");
      children[childIndex] = newChild;
   }

   public final OcTreeDataNode<V> getChild(int childIndex)
   {
      checkChildIndex(childIndex);
      checkNodeHasChildren(this);
      checkNodeChildNotNull(this, childIndex);
      return children == null ? null : getChildUnsafe(childIndex);
   }

   public final OcTreeDataNode<V> getChildUnsafe(int childIndex)
   {
      return children[childIndex];
   }

   public final OcTreeDataNode<V> removeChild(int childIndex)
   {
      OcTreeDataNode.checkChildIndex(childIndex);
      return removeChildUnsafe(childIndex);
   }

   public final OcTreeDataNode<V> removeChildUnsafe(int childIndex)
   {
      OcTreeDataNode<V> removedChild = children[childIndex];
      children[childIndex] = null;
      return removedChild;
   }

   public final void removeChildren()
   {
      children = null;
   }

   public abstract boolean epsilonEquals(OcTreeDataNode<?> other);

   public abstract boolean epsilonEquals(OcTreeDataNode<V> other, V epsilon);

   @Override
   public String toString()
   {
      String[] childrenNames = new String[8];
      if (children != null)
      {
         for (int i = 0; i < 8; i++)
         {
            OcTreeDataNode<V> child = getChildUnsafe(i);
            childrenNames[i] = child == null ? null : child.getClass().getSimpleName();
         }
      }
      return getClass().getSimpleName() + ": value: " + value + ", children: " + Arrays.toString(childrenNames);
   }

   /** 
    * Safe test if node has a child at index childIdx.
    * First tests if there are any children. Replaces node->childExists(...)
    * \return true if the child at childIdx exists
    */
   public final static boolean nodeChildExists(OcTreeDataNode<?> node, int childIndex)
   {
      return node.children[childIndex] != null;
   }

   public final static void checkChildIndex(int childIndex)
   {
      if (childIndex > 7)
         throw new RuntimeException("Bad child index :" + childIndex + ", expected index to be in [0, 7].");
   }

   public final static void checkNodeHasChildren(OcTreeDataNode<?> node)
   {
      if (node.children == null)
         throw new RuntimeException("The given node has no children.");
   }

   public final static void checkNodeChildNotNull(OcTreeDataNode<?> node, int childIndex)
   {
      if (node.children[childIndex] == null)
         throw new RuntimeException("Child is already null.");
   }
}
