/***************************************************************************       
*   Copyright 2012 - 2013 Advanced Micro Devices, Inc.                                     
*                                                                                    
*   Licensed under the Apache License, Version 2.0 (the "License");   
*   you may not use this file except in compliance with the License.                 
*   You may obtain a copy of the License at                                          
*                                                                                    
*       http://www.apache.org/licenses/LICENSE-2.0                      
*                                                                                    
*   Unless required by applicable law or agreed to in writing, software              
*   distributed under the License is distributed on an "AS IS" BASIS,              
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.         
*   See the License for the specific language governing permissions and              
*   limitations under the License.                                                   

***************************************************************************/
#pragma once
#if !defined( BOLT_AMP_TRANSFORM_ITERATOR_H )
#define BOLT_AMP_TRANSFORM_ITERATOR_H
#include "bolt/amp/bolt.h"
#include "bolt/amp/iterator/iterator_traits.h"
#include "bolt/amp/iterator/counting_iterator.h"
#include "bolt/amp/device_vector.h"

/*! \file bolt/cl/iterator/transform_iterator.h
    \brief
*/


namespace bolt {
namespace amp {

  struct transform_iterator_tag
      : public fancy_iterator_tag
      {   // identifying tag for random-access iterators
      };

      template< class UnaryFunc, class Iterator >
      class transform_iterator: public std::iterator< transform_iterator_tag,
                                                      std::result_of<UnaryFunc()>,
                                                      int >
      {
        public:
         typedef typename std::iterator< transform_iterator_tag, typename std::result_of<UnaryFunc()>, int>::difference_type
         difference_type;

         typedef transform_iterator<UnaryFunc,Iterator> transf_iterator;
         typedef typename std::iterator_traits<Iterator>::value_type value_type;
         typedef typename std::iterator_traits<Iterator>::pointer         pointer;

        // Default constructor
        transform_iterator( ):m_Index( 0 ) {}

        //  Basic constructor requires a reference to the container and a positional element
        transform_iterator( Iterator iiter, UnaryFunc ifunc, const control& ctl = control::getDefault( ) ): iter(iiter), func(ifunc), m_Index(iiter.m_Index) {}

        //  This copy constructor allows an iterator to convert into a transf_iterator, but not vica versa
        template< class OtherUnaryFunc, class OtherIter>
        transform_iterator( const transform_iterator< OtherUnaryFunc, OtherIter >& rhs ):m_Index( rhs.m_Index ){}

        //  This copy constructor allows an iterator to convert into a transf_iterator, but not vica versa
        transform_iterator< UnaryFunc, Iterator >& operator= ( const transform_iterator< UnaryFunc, Iterator >& rhs )
        {
            if( this == &rhs )
                return *this;

            func = rhs.func;
            iter = rhs.iter;

            m_Index = rhs.m_Index;
            return *this;
        }
            
        transform_iterator< UnaryFunc, Iterator >& operator+= ( const  difference_type & n )
        {
            advance( n );
            return *this;
        }
            
        const transform_iterator< UnaryFunc, Iterator > operator+ ( const difference_type & n ) const
        {
            transform_iterator< UnaryFunc, Iterator > result( *this );
            result.advance( n );
            return result;
        }

        const concurrency::array_view<int> & getBuffer( transf_iterator itr ) const
        {
            return *value;
        }
        

        const transform_iterator< UnaryFunc, Iterator > & getContainer( ) const
        {
            return *this;
        }

        difference_type operator- ( const transform_iterator< UnaryFunc, Iterator >& rhs ) const
        {
            return m_Index - rhs.m_Index;
        }

        //  Public member variables
        difference_type m_Index;

        //  Used for templatized copy constructor and the templatized equal operator
        template < typename, typename > friend class transform_iterator;

        //  For a transform_iterator, do nothing on an advance
        void advance( difference_type n )
        {
            m_Index += n;
        }

        // Pre-increment
        transform_iterator< UnaryFunc, Iterator > operator++ ( )
        {
            advance( 1 );
            transform_iterator< UnaryFunc, Iterator > result( *this );
            return result;
        }

        // Post-increment
        transform_iterator< UnaryFunc, Iterator > operator++ ( int )
        {
            transform_iterator< UnaryFunc, Iterator > result( *this );
            advance( 1 );
            return result;
        }

        // Pre-decrement
        transform_iterator< UnaryFunc, Iterator > operator--( ) const
        {
            transform_iterator< UnaryFunc, Iterator > result( *this );
            result.advance( -1 );
            return result;
        }

        // Post-decrement
        transform_iterator< UnaryFunc, Iterator > operator--( int ) const
        {
            transform_iterator< UnaryFunc, Iterator > result( *this );
            result.advance( -1 );
            return result;
        }

        difference_type getIndex() const
        {
            return m_Index;
        }

        template< class OtherUnaryFunc, class OtherIterator >
        bool operator== ( const transform_iterator< OtherUnaryFunc, OtherIterator >& rhs ) const
        {
            bool sameIndex = ( rhs.m_Index == m_Index );
            return sameIndex;
        }

        template< class OtherUnaryFunc, class OtherIterator >
        bool operator!= ( const transform_iterator< OtherUnaryFunc, OtherIterator >& rhs ) const
        {
            bool sameIndex = ( rhs.m_Index != m_Index );
            return sameIndex;
        }

        template< class OtherUnaryFunc, class OtherIterator >
        bool operator< ( const transform_iterator< OtherUnaryFunc, OtherIterator >& rhs ) const
        {
            bool sameIndex = (m_Index < rhs.m_Index);
            return sameIndex;
        }

        // Dereference operators
        value_type operator*() const
        {
          return func( iter[ m_Index ] );
        }

        value_type operator[](int x) const restrict(cpu,amp)
        {
          return func( iter[ x ] );
        }

        value_type operator[](int x) restrict(cpu,amp)
        {
          return func( iter[ x ] );
        }

        UnaryFunc func;
        Iterator iter;
        //value_type val_at;

      };


  template< class UnaryFunc, class Iterator >
  transform_iterator< UnaryFunc, Iterator > make_transform_iterator( Iterator iter, UnaryFunc func )
  {
      transform_iterator< UnaryFunc, Iterator > tmp( iter, func );
      return tmp;
  }

}
}


#endif
