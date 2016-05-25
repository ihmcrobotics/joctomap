package us.ihmc.octoMap.key;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

public class KeyRay implements Iterable<OcTreeKey>
{
   private static final int maxSize = 100000;
   private final List<OcTreeKey> ray = new ArrayList<>();

   public KeyRay()
   {
   }

   public KeyRay(KeyRay other)
   {
      ray.clear();
      ray.addAll(other.ray);
   }

   public void addKey(OcTreeKey key)
   {
      ray.add(key);
   }

   public OcTreeKey getFirst()
   {
      return ray.get(0);
   }

   public OcTreeKey getLast()
   {
      return ray.get(ray.size() - 1);
   }

   public void clear()
   {
      ray.clear();
   }

   public int size()
   {
      return ray.size();
   }

   public int sizeMax()
   {
      return maxSize;
   }

   @Override
   public Iterator<OcTreeKey> iterator()
   {
      return ray.iterator();
   }

   public ListIterator<OcTreeKey> reverseIterator()
   {
      return ray.listIterator(ray.size() - 1);
   }
}