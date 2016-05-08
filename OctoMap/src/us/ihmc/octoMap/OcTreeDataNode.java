package us.ihmc.octoMap;

public abstract class OcTreeDataNode<V, N extends OcTreeDataNode<V, N>>
{
   protected V value;
   protected N[] children;

   public OcTreeDataNode()
   {
   }

   public OcTreeDataNode(V initialValue)
   {
      value = initialValue;
   }

   public OcTreeDataNode(N other)
   {
      value = other.value;

      if (other.hasChildren())
         allocateChildren();

      for (int i = 0; i < 8; i++)
         children[i] = other.children[i].clone();
   }

   public void setValue(V value)
   {
      this.value = value;
   }

   public void copyData(OcTreeDataNode<V, N> other)
   {
      value = other.value;
   }

   abstract void allocateChildren();

   public abstract N clone();

   public abstract N create();

   public boolean hasChildren()
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

   public boolean equals(N other)
   {
      return value == other.value;
   }
}
