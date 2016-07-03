#ifndef STRUCTURE_H
#define STRUCTURE_H

#include <list>
#include <unordered_map>

#include "target.h"

/* template specialization to allow use of target iterators as keys in associative containers */
namespace std {
   template<>
   struct hash<STarget::TConstListIterator> {
      size_t operator () (const STarget::TConstListIterator& x) const {
         return std::hash<unsigned int>()(x->Id);
      }
   };
}

struct SStructure {
   using TList = std::list<SStructure>;
   using TListIterator = TList::iterator;
   using TConstListIterator = TList::const_iterator;

   /* Default constructor */
   SStructure() {}
   /* Constructor for a structure with single target */
   SStructure(STarget::TConstListIterator& it_member) :
      Members(1u, it_member) {}

   /* A list of iterators onto the target list */
   std::list<STarget::TConstListIterator> Members;
   /* A mapping between the iterators, representing the connectivity between targets */
   std::unordered_multimap<STarget::TConstListIterator, STarget::TConstListIterator> Connectivity;
   /* Identifier (0 => unassigned) */
   unsigned int Id = 0;
};

#endif
