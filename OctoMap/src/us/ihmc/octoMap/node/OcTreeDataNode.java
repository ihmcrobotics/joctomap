package us.ihmc.octoMap.node;

import java.util.Arrays;
import java.util.HashMap;

import us.ihmc.robotics.lists.GenericTypeBuilder;

public abstract class OcTreeDataNode<V>
{
   private HashMap<Class<? extends OcTreeDataNode<?>>, GenericTypeBuilder<? extends OcTreeDataNode<?>>> builderCache = OcTreeNodeTools.BUILDER_CACHE_THREAD_LOCAL.get();

   protected V value;
   protected OcTreeDataNode<V>[] children;

   public OcTreeDataNode()
   {
   }

   public OcTreeDataNode(V initialValue)
   {
      value = initialValue;
   }

   public void copyData(OcTreeDataNode<V> other)
   {
      value = other.value;
   }

   public abstract void updateOccupancyChildren();

   @SuppressWarnings("unchecked")
   public final void allocateChildren()
   {
      children = new OcTreeDataNode[8];
   }

   public final OcTreeDataNode<V> cloneRecursive()
   {
      OcTreeDataNode<V> ret = create();
      ret.copyData(this);

      if (hasAtLeastOneChild())
         allocateChildren();

      for (int i = 0; i < 8; i++)
         ret.children[i] = children[i].cloneRecursive();

      return ret;
   }

   @SuppressWarnings("unchecked")
   public final OcTreeDataNode<V> create()
   {
      GenericTypeBuilder<? extends OcTreeDataNode<?>> builder = builderCache.get(getClass());
      if (builder == null)
      {
         builder = (GenericTypeBuilder<? extends OcTreeDataNode<?>>) GenericTypeBuilder.createBuilderWithEmptyConstructor(getClass());
         builderCache.put((Class<? extends OcTreeDataNode<?>>) getClass(), builder);
      }
      return (OcTreeDataNode<V>) builder.newInstance();
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

   public final void setChild(int childIndex, OcTreeDataNode<V> newChild)
   {
      OcTreeNodeTools.checkChildIndex(childIndex);
      if (!getClass().isInstance(newChild))
         throw new RuntimeException("Cannot add a child of a different type");
      children[childIndex] = newChild;
   }

   public final OcTreeDataNode<V> getChild(int childIndex)
   {
      return OcTreeNodeTools.getNodeChild(this, childIndex);
   }

   public final OcTreeDataNode<V> getChildUnsafe(int childIndex)
   {
      return children[childIndex];
   }

   public final OcTreeDataNode<V> removeChild(int childIndex)
   {
      OcTreeNodeTools.checkChildIndex(childIndex);
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
      return getClass().getSimpleName() + ": children = " + Arrays.toString(getChildrenSimpleNames());
   }

   protected String[] getChildrenSimpleNames()
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
      return childrenNames;
   }
}
