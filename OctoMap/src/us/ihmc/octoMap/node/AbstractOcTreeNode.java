package us.ihmc.octoMap.node;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.HashMap;

public abstract class AbstractOcTreeNode<N extends AbstractOcTreeNode<N>>
{
   protected N[] children;
   private final HashMap<Class<? extends AbstractOcTreeNode<?>>, NodeBuilder<? extends AbstractOcTreeNode<?>>> builderCache = NodeManager.BUILDER_CACHE_THREAD_LOCAL.get();

   AbstractOcTreeNode()
   {
   }

   public abstract void copyData(N other);

   public abstract void updateOccupancyChildren();

   public abstract void clear();

   @SuppressWarnings("unchecked")
   public void allocateChildren()
   {
      children = (N[]) Array.newInstance(getClass(), 8);
   }

   public void assignChildren(N[] newChildren)
   {
      children = newChildren;
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

   @SuppressWarnings({"unchecked", "rawtypes"})
   public N create()
   {
      NodeBuilder<N> builder = (NodeBuilder<N>) builderCache.get(getClass());
      if (builder == null)
      {
         builder = (NodeBuilder<N>) new NodeBuilder(getClass());
         builderCache.put((Class<? extends AbstractOcTreeNode<?>>) getClass(), builder);
      }
      N ret = builder.createNode();
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
      OcTreeNodeTools.checkChildIndex(childIndex);
      if (!getClass().isInstance(newChild))
         throw new RuntimeException("Cannot add a child of a different type");
      setChildUnsafe(childIndex, newChild);
   }

   public void setChildUnsafe(int childIndex, N newChild)
   {
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
      if (removedChild != null)
         removedChild.clear();
      children[childIndex] = null;
      return removedChild;
   }

   public final N[] removeChildren()
   {
      N[] removedChildren = children;
      children = null;
      return removedChildren;
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
