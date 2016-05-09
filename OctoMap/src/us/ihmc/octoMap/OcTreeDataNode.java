package us.ihmc.octoMap;

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

   public V getValue()
   {
      return value;
   }

   public void copyData(OcTreeDataNode<V> other)
   {
      value = other.value;
   }

   public abstract void updateOccupancyChildren();

   abstract void allocateChildren();

   public abstract OcTreeDataNode<V> clone();

   public abstract OcTreeDataNode<V> create();

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

   public boolean equals(OcTreeDataNode<V> other)
   {
      return value == other.value;
   }
}
