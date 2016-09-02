package us.ihmc.octoMap.key;

import java.util.HashSet;

public class OcTreeKeySet extends HashSet<OcTreeKey>
{
   private static final long serialVersionUID = 2780317356917541560L;

   public OcTreeKeySet()
   {
      super();
   }

   public OcTreeKeySet(int initialCapacity)
   {
      super(initialCapacity);
   }

   @Override
   public boolean add(OcTreeKey key)
   {
      return add((OcTreeKeyReadOnly) key);
   }

   public boolean add(OcTreeKeyReadOnly key)
   {
      return super.add(new OcTreeKey(key));
   }

   public boolean addAll(OcTreeKeyListReadOnly keyList)
   {
      boolean changed = false;
      for (int i = 0; i < keyList.size(); i++)
      {
         if (add(keyList.get(i)))
            changed = true;
      }
      return changed;
   }

   public boolean removeAll(OcTreeKeyListReadOnly keyList)
   {
      boolean changed = false;
      for (int i = 0; i < keyList.size(); i++)
      {
         if (remove(keyList.get(i)))
            changed = true;
      }
      return changed;
   }

   public boolean addAll(KeyRay keyRay)
   {
      boolean changed = false;
      for (int i = 0; i < keyRay.size(); i++)
      {
         if (add(keyRay.get(i)))
            changed = true;
      }
      return changed;
   }

   public boolean removeAll(KeyRay keyRay)
   {
      boolean changed = false;
      for (int i = 0; i < keyRay.size(); i++)
      {
         if (remove(keyRay.get(i)))
            changed = true;
      }
      return changed;
   }
}