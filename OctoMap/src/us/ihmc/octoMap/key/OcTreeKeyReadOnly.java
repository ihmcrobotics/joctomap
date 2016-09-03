package us.ihmc.octoMap.key;

public interface OcTreeKeyReadOnly
{

   int getKey(int index);

   boolean equals(Object obj);

   boolean equals(OcTreeKey other);

   int hashCode();

   String toString();

}