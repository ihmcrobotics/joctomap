package us.ihmc.octoMap.key;

import java.util.ArrayList;
import java.util.ListIterator;

public class KeyRay extends ArrayList<OcTreeKey>
{
   private static final long serialVersionUID = -703213401402332667L;
   private static final int maxSize = 100000;

   public KeyRay()
   {
      super();
   }

   public KeyRay(int initialCapacity)
   {
      super(initialCapacity);
   }

   public KeyRay(KeyRay other)
   {
      super(other);
   }

   public boolean add(OcTreeKey key)
   {
      return addKey(key);
   }

   public boolean addKey(OcTreeKey key)
   {
      return super.add(new OcTreeKey(key));
   }

   public OcTreeKey getFirst()
   {
      return get(0);
   }

   public OcTreeKey getLast()
   {
      return get(size() - 1);
   }

   public int sizeMax()
   {
      return maxSize;
   }

   public ListIterator<OcTreeKey> reverseIterator()
   {
      return listIterator(size() - 1);
   }
}