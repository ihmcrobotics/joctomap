package us.ihmc.octoMap.key;

import gnu.trove.set.hash.TIntHashSet;

public class OcTreeKeySet
{
   private final TIntHashSet hashCodeSet;
   private final OcTreeKeyList keyData;

   public OcTreeKeySet()
   {
      this(100);
   }

   public OcTreeKeySet(int initialCapacity)
   {
      hashCodeSet = new TIntHashSet(initialCapacity);
      keyData = new OcTreeKeyList(initialCapacity);
   }

   public void clear()
   {
      hashCodeSet.clear();
      keyData.clear();
   }

   public int size()
   {
      return keyData.size();
   }

   public boolean add(OcTreeKeyReadOnly key)
   {
      if (hashCodeSet.add(key.hashCode()))
      {
         keyData.add(key);
         return true;
      }
      return false;
   }

   public boolean addAll(OcTreeKeyListReadOnly keyList)
   {
      boolean changed = false;
      for (int i = 0; i < keyList.size(); i++)
         changed |= add(keyList.get(i));
      return changed;
   }

   public boolean addAll(KeyRay keyRay)
   {
      boolean changed = false;
      for (int i = 0; i < keyRay.size(); i++)
         changed |= add(keyRay.get(i));
      return changed;
   }

   public boolean contains(OcTreeKeyReadOnly key)
   {
      return hashCodeSet.contains(key.hashCode());
   }

   public OcTreeKeyReadOnly get(int index)
   {
      return keyData.get(index);
   }
}