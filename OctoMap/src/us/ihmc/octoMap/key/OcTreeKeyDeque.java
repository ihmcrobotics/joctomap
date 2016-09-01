package us.ihmc.octoMap.key;

import java.util.ArrayDeque;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;

public class OcTreeKeyDeque extends ArrayDeque<OcTreeKey>
{
   private static final long serialVersionUID = 2764740419465316737L;

   private final ArrayDeque<OcTreeKey> unusedKeys;
   
   public OcTreeKeyDeque()
   {
      this(16);
   }

   public OcTreeKeyDeque(int numElements)
   {
      super(numElements);
      unusedKeys = new ArrayDeque<>(numElements);
      for (int i = 0; i < numElements; i++)
         unusedKeys.add(new OcTreeKey());
   }

   /** {@inheritDoc} */
   @Override
   public int size()
   {
      return super.size();
   }

   /** {@inheritDoc} */
   @Override
   public boolean isEmpty()
   {
      return super.isEmpty();
   }

   /**
    * Add an empty key at the front of this deque and return it.
    * @return the new empty key.
    */
   public OcTreeKey addFirst()
   {
      OcTreeKey newKey = getOrCreateUnusedKey();
      super.addFirst(newKey);
      return newKey;
   }

   /**
    * Add an empty key at the end of this deque and return it.
    * @return the new empty key.
    */
   public OcTreeKey addLast()
   {
      OcTreeKey newKey = getOrCreateUnusedKey();
      super.addLast(newKey);
      return newKey;
   }

   /** {@inheritDoc} */
   @Override
   public boolean add(OcTreeKey newKey)
   {
      return super.add(copyAndReturnLocalKey(newKey));
   }

   public boolean addAll(List<OcTreeKey> keyList)
   {
      boolean changed = false;

      for (int i = 0; i < keyList.size(); i++)
      {
         if (add(keyList.get(i)))
            changed = true;
      }
      return changed;
   }

   /**
    * The deque will be empty after this call returns.
    * The removed elements are saved in a local buffer for recycling purpose to prevent garbage generation.
    */
   @Override
   public void clear()
   {
      while (!super.isEmpty())
         unusedKeys.add(super.poll());
   }

   /** {@inheritDoc} */
   @Override
   public void addFirst(OcTreeKey newKey)
   {
      super.addFirst(copyAndReturnLocalKey(newKey));
   }

   /** {@inheritDoc} */
   @Override
   public void addLast(OcTreeKey newKey)
   {
      super.addLast(copyAndReturnLocalKey(newKey));
   }

   /** {@inheritDoc} */
   @Override
   public void push(OcTreeKey newKey)
   {
      super.push(copyAndReturnLocalKey(newKey));
   }

   /** {@inheritDoc} */
   @Override
   public boolean offerFirst(OcTreeKey newKey)
   {
      return super.offerFirst(copyAndReturnLocalKey(newKey));
   }

   /** {@inheritDoc} */
   @Override
   public boolean offerLast(OcTreeKey newKey)
   {
      return super.offerLast(copyAndReturnLocalKey(newKey));
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public OcTreeKey removeFirst()
   {
      OcTreeKey keyToReturn = super.removeFirst();
      unusedKeys.add(keyToReturn);
      return keyToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public OcTreeKey removeLast()
   {
      OcTreeKey keyToReturn = super.removeLast();
      unusedKeys.add(keyToReturn);
      return keyToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public OcTreeKey pollFirst()
   {
      OcTreeKey keyToReturn = super.pollFirst();
      unusedKeys.add(keyToReturn);
      return keyToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public OcTreeKey pollLast()
   {
      OcTreeKey keyToReturn = super.pollLast();
      unusedKeys.add(keyToReturn);
      return keyToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public OcTreeKey getFirst()
   {
      OcTreeKey keyToReturn = super.getFirst();
      unusedKeys.add(keyToReturn);
      return keyToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public OcTreeKey getLast()
   {
      OcTreeKey keyToReturn = super.getLast();
      unusedKeys.add(keyToReturn);
      return keyToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public OcTreeKey peekFirst()
   {
      OcTreeKey keyToReturn = super.peekFirst();
      unusedKeys.add(keyToReturn);
      return keyToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public OcTreeKey peekLast()
   {
      OcTreeKey keyToReturn = super.peekLast();
      unusedKeys.add(keyToReturn);
      return keyToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public OcTreeKey remove()
   {
      OcTreeKey keyToReturn = super.remove();
      unusedKeys.add(keyToReturn);
      return keyToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public OcTreeKey poll()
   {
      OcTreeKey keyToReturn = super.poll();
      unusedKeys.add(keyToReturn);
      return keyToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public OcTreeKey element()
   {
      OcTreeKey keyToReturn = super.element();
      unusedKeys.add(keyToReturn);
      return keyToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public OcTreeKey peek()
   {
      OcTreeKey keyToReturn = super.peek();
      unusedKeys.add(keyToReturn);
      return keyToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public OcTreeKey pop()
   {
      OcTreeKey keyToReturn = super.pop();
      unusedKeys.add(keyToReturn);
      return keyToReturn;
   }

   private OcTreeKey copyAndReturnLocalKey(OcTreeKeyReadOnly keyToCopy)
   {
      OcTreeKey localKey = getOrCreateUnusedKey();
      localKey.set(keyToCopy);
      return localKey;
   }

   private OcTreeKey getOrCreateUnusedKey()
   {
      if (unusedKeys.isEmpty())
         return new OcTreeKey();
      else
         return unusedKeys.poll();
   }

   @Override
   public String toString()
   {
      Iterator<OcTreeKey> iterator = super.iterator();
      if (!iterator.hasNext())
         return "[]";

      StringBuilder sb = new StringBuilder();
      sb.append('[');
      for (;;)
      {
         OcTreeKey nextKey = iterator.next();
         sb.append(nextKey);
         if (!iterator.hasNext())
            return sb.append(']').toString();
         sb.append(',').append(' ');
      }
   }

   /** Unsupported operation. */
   @Override
   public OcTreeKeyDeque clone()
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean remove(Object o)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean contains(Object o)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public Object[] toArray()
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public <T> T[] toArray(T[] a)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean containsAll(Collection<?> c)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean retainAll(Collection<?> c)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean removeFirstOccurrence(Object o)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean removeLastOccurrence(Object o)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean offer(OcTreeKey e)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public Iterator<OcTreeKey> descendingIterator()
   {
      throw new UnsupportedOperationException();
   }
}
