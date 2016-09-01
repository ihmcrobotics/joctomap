package us.ihmc.octoMap.key;

import java.util.HashSet;
import java.util.List;

public class KeySet extends HashSet<OcTreeKeyReadOnly>
{
   private static final long serialVersionUID = 2780317356917541560L;

   public KeySet()
   {
      super();
   }

   public KeySet(int initialCapacity)
   {
      super(initialCapacity);
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
      return addAll((List<OcTreeKey>) keyRay);
   }

   public boolean addAll(List<? extends OcTreeKeyReadOnly> keyList)
   {
      boolean changed = false;
      for (int i = 0; i < keyList.size(); i++)
      {
         if (add(keyList.get(i)))
            changed = true;
      }
      return changed;
   }

   public boolean removeAll(List<? extends OcTreeKeyReadOnly> keyList)
   {
      boolean changed = false;
      for (int i = 0; i < keyList.size(); i++)
      {
         if (remove(keyList.get(i)))
            changed = true;
      }
      return changed;
   }
}