package us.ihmc.octoMap.node;

import java.util.Arrays;
import java.util.HashMap;

import us.ihmc.robotics.lists.GenericTypeBuilder;

public abstract class OcTreeDataNode<N extends OcTreeDataNode<N>>
{
   private HashMap<Class<? extends OcTreeDataNode<?>>, GenericTypeBuilder<? extends OcTreeDataNode<?>>> builderCache = OcTreeNodeTools.BUILDER_CACHE_THREAD_LOCAL
         .get();

   protected N[] children;

   public OcTreeDataNode()
   {
   }

   public abstract void copyData(N other);

   public abstract void updateOccupancyChildren();

   @SuppressWarnings("unchecked")
   public final void allocateChildren()
   {
      children = (N[]) new OcTreeDataNode[8];
   }

   @SuppressWarnings("unchecked")
   public final N cloneRecursive()
   {
      N ret = create();
      ret.copyData((N) this);

      if (hasAtLeastOneChild())
         allocateChildren();

      for (int i = 0; i < 8; i++)
         ret.children[i] = children[i].cloneRecursive();

      return ret;
   }

   @SuppressWarnings("unchecked")
   public final N create()
   {
      GenericTypeBuilder<N> builder = (GenericTypeBuilder<N>) builderCache.get(getClass());
      if (builder == null)
      {
         builder = (GenericTypeBuilder<N>) GenericTypeBuilder.createBuilderWithEmptyConstructor(getClass());
         builderCache.put((Class<? extends OcTreeDataNode<?>>) getClass(), builder);
      }
      return builder.newInstance();
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
      OcTreeNodeTools.checkChildIndex(childIndex);
      if (!getClass().isInstance(newChild))
         throw new RuntimeException("Cannot add a child of a different type");
      children[childIndex] = newChild;
   }

   @SuppressWarnings("unchecked")
   public final N getChild(int childIndex)
   {
      return OcTreeNodeTools.getNodeChild((N) this, childIndex);
   }

   public final N getChildUnsafe(int childIndex)
   {
      return children[childIndex];
   }

   public final N removeChild(int childIndex)
   {
      OcTreeNodeTools.checkChildIndex(childIndex);
      return removeChildUnsafe(childIndex);
   }

   public final N removeChildUnsafe(int childIndex)
   {
      N removedChild = children[childIndex];
      children[childIndex] = null;
      return removedChild;
   }

   public final void removeChildren()
   {
      children = null;
   }

   public abstract boolean epsilonEquals(N other);

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
            N child = getChildUnsafe(i);
            childrenNames[i] = child == null ? null : child.getClass().getSimpleName();
         }
      }
      return childrenNames;
   }
}
