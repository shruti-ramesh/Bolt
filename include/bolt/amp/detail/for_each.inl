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

///////////////////////////////////////////////////////////////////////////////
// AMP ForEach
//////////////////////////////////////////////////////////////////////////////

#pragma once
#if !defined( BOLT_AMP_FOR_EACH_INL )
#define BOLT_AMP_FOR_EACH_INL

#include <algorithm>
#include <type_traits>
#include "bolt/amp/bolt.h"
#include "bolt/amp/device_vector.h"
#include "bolt/amp/iterator/iterator_traits.h"




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


namespace bolt
{
    namespace amp
    {

              template<typename InputIterator , typename UnaryFunction >  
              InputIterator for_each (InputIterator first, InputIterator last, UnaryFunction f)
              {
              	  std::for_each(first, last, f);
				  /*size_t n = std::distance(first,last);
				  for(int i=0;i<(int)n ;i++)
				  {
					  first[i] = f(first[i]);
				  }*/
				  return first;
              }
              
              template<typename InputIterator , typename UnaryFunction >  
              InputIterator  for_each (bolt::amp::control &ctl, InputIterator first, InputIterator last, UnaryFunction f)
              {
              	   std::for_each(first, last, f);
				   return first;
              }
              
              template<typename InputIterator , typename Size , typename UnaryFunction > 
              InputIterator for_each_n  ( InputIterator  first,  Size  n,  UnaryFunction  f)
              {
              	   std::for_each(first, first + n, f);
				   return first;
              }
              
              template<typename InputIterator , typename Size , typename UnaryFunction > 
              InputIterator for_each_n  ( bolt::amp::control &ctl, InputIterator  first,  Size  n,  UnaryFunction  f)
              {
              	   std::for_each(first, first + n, f);
				   return first;
              }

	}
}

#endif // AMP_FOR_EACH_INL