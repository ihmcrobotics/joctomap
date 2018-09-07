package us.ihmc.jOctoMap.key;

public interface OcTreeKeyListReadOnly
{

   /**
    * Returns the number of elements in this list.
    *
    * @return the number of elements in this list
    */
   int size();

   /**
    * Returns <tt>true</tt> if this list contains no elements.
    *
    * @return <tt>true</tt> if this list contains no elements
    */
   boolean isEmpty();

   /**
    * Returns the element at the specified position in this list.
    *
    * @param  index index of the element to return
    * @return the element at the specified position in this list
    * @throws IndexOutOfBoundsException if the index is out of range
     *         (<tt>index &lt; 0 || index &gt;= size()</tt>)
    */
   OcTreeKeyReadOnly get(int i);

   /**
    * Returns the last element of this list.
    * If the list is empty, it returns {@code null}.
    * @return the last element of this list
    */
   OcTreeKeyReadOnly getFirst();

   /**
    * Returns the last element of this list.
    * If the list is empty, it returns {@code null}.
    * @return the last element of this list
    */
   OcTreeKeyReadOnly getLast();

   /**
    * Returns <tt>true</tt> if this list contains the specified element.
    *
    * @param object element whose presence in this list is to be tested
    * @return <tt>true</tt> if this list contains the specified element
    */
   boolean contains(Object object);

   /**
    * Returns the index of the first occurrence of the specified element
    * in this list, or -1 if this list does not contain the element.
    */
   int indexOf(Object object);

   /**
    * Returns the index of the last occurrence of the specified element
    * in this list, or -1 if this list does not contain the element.
    */
   int lastIndexOf(Object object);

}