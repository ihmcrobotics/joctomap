package us.ihmc.jOctoMap.key;

public interface OcTreeKeyReadOnly
{

   int getKey(int index);

   @Override
   boolean equals(Object obj);

   boolean equals(OcTreeKey other);

   @Override
   int hashCode();

   @Override
   String toString();

}