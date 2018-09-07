package us.ihmc.jOctoMap.key;

import java.util.HashSet;

public class OcTreeKeySet extends HashSet<OcTreeKeyReadOnly>
{
   private static final long serialVersionUID = 5475332642252522728L;

   public OcTreeKeySet()
   {
      this(100);
   }

   public OcTreeKeySet(int initialCapacity)
   {
      super(initialCapacity);
   }
}