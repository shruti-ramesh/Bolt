/***************************************************************************
*   © 2012,2014 Advanced Micro Devices, Inc. All rights reserved.
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
#if !defined( BOLT_AMP_SCATTER_INL )
#define BOLT_AMP_SCATTER_INL
#define SCATTER_WAVEFRNT_SIZE 264

#include <algorithm>
#include <type_traits>
#include "bolt/amp/bolt.h"
#include "bolt/amp/iterator/iterator_traits.h"
#include "bolt/amp/device_vector.h"
#include <amp.h>

#ifdef ENABLE_TBB
    #include "bolt/btbb/scatter.h"
#endif

namespace bolt {
namespace amp {

namespace detail {


/* Begin-- Serial Implementation of the scatter and scatter_if routines */
namespace serial{

template<typename InputIterator1,
         typename InputIterator2,
         typename OutputIterator>

void scatter( bolt::amp::control &ctl,
              InputIterator1 first1,
              InputIterator1 last1,
              InputIterator2 map,
              OutputIterator result)
{
       int numElements = static_cast< int >( std::distance( first1, last1 ) );

	   for (int iter = 0; iter<(int)numElements; iter++)
                *(result+*(map + iter)) = *(first1 + iter);
}


template< typename InputIterator1,
           typename InputIterator2,
           typename InputIterator3,
           typename OutputIterator,
           typename Predicate>
void scatter_if (bolt::amp::control &ctl,
                 InputIterator1 first1,
                 InputIterator1 last1,
                 InputIterator2 map,
                 InputIterator3 stencil,
                 OutputIterator result,
                 Predicate pred)
{
       int numElements = static_cast< int >( std::distance( first1, last1 ) );
	   for (int iter = 0; iter< numElements; iter++)
       {
             if(pred(stencil[iter]) != 0)
                  result[*(map+(iter))] = first1[iter];
       }
}

}//end of namespace serial


namespace btbb{

template< typename InputIterator1,
           typename InputIterator2,
           typename InputIterator3,
           typename OutputIterator,
           typename Predicate>
void scatter_if (bolt::amp::control &ctl,
                 InputIterator1 first1,
                 InputIterator1 last1,
                 InputIterator2 map,
                 InputIterator3 stencil,
                 OutputIterator result,
                 Predicate pred)
{
       bolt::btbb::scatter_if(first1, last1, map, stencil, result, pred);
}


template<typename InputIterator1,
         typename InputIterator2,
         typename OutputIterator>

void scatter( bolt::amp::control &ctl,
              InputIterator1 first1,
              InputIterator1 last1,
              InputIterator2 map,
              OutputIterator result)
{
       bolt::btbb::scatter(first1, last1, map, result);
}


}//end of namespace btbb

namespace amp{

    template< typename DVInputIterator1,
              typename DVInputIterator2,
              typename DVInputIterator3,
              typename DVOutputIterator,
              typename Predicate >
	typename std::enable_if< std::is_same< typename std::iterator_traits< DVOutputIterator >::iterator_category ,
                                       bolt::amp::device_vector_tag
                                     >::value
                       >::type
    scatter_if( bolt::amp::control &ctl,
                             const DVInputIterator1& first1,
                             const DVInputIterator1& last1,
                             const DVInputIterator2& map,
                             const DVInputIterator3& stencil,
                             const DVOutputIterator& result,
                             const Predicate& pred )
    {
		concurrency::accelerator_view av = ctl.getAccelerator().default_view;
		typedef std::iterator_traits< DVInputIterator1 >::value_type iType1;
		typedef std::iterator_traits< DVInputIterator2 >::value_type iType2;		
		typedef std::iterator_traits< DVInputIterator3 >::value_type iType3;
		typedef std::iterator_traits< DVOutputIterator >::value_type oType;

        const int szElements = static_cast< int >(  std::distance(first1,last1) );
		const int leng =  szElements + SCATTER_WAVEFRNT_SIZE - (szElements % SCATTER_WAVEFRNT_SIZE);
		concurrency::extent< 1 > inputExtent(leng);
                try
                {
                    concurrency::parallel_for_each(av,  inputExtent, [=](concurrency::index<1> idx) restrict(amp)
                    {
                        int globalId = idx[ 0 ];

                        if( globalId >= szElements)
                        return;
						iType2 m = map[ globalId ];
						iType3 s = stencil[ globalId ];

						if (pred( s ))
						{
							result [ m ] = first1 [ globalId ] ;
						}
						});	
                }
			    catch(std::exception &e)
                {
                      std::cout << "Exception while calling bolt::amp::scatter parallel_for_each"<<e.what()<<std::endl;
                      return;
                }
		result.getContainer().getBuffer(result, szElements).synchronize();
    };


    template< typename DVInputIterator1,
              typename DVInputIterator2,
              typename DVOutputIterator >
    typename std::enable_if< std::is_same< typename std::iterator_traits< DVOutputIterator >::iterator_category ,
                                       bolt::amp::device_vector_tag
                                     >::value
                       >::type
    scatter( bolt::amp::control &ctl,
                          const DVInputIterator1& first1,
                          const DVInputIterator1& last1,
                          const DVInputIterator2& map,
                          const DVOutputIterator& result )
    {
		concurrency::accelerator_view av = ctl.getAccelerator().default_view;
		typedef std::iterator_traits< DVInputIterator1 >::value_type iType1;
		typedef std::iterator_traits< DVInputIterator2 >::value_type iType2;
		typedef std::iterator_traits< DVOutputIterator >::value_type oType;

        const int szElements = static_cast< int >(  std::distance(first1,last1) );
		const unsigned int leng =  szElements + SCATTER_WAVEFRNT_SIZE - (szElements % SCATTER_WAVEFRNT_SIZE);
		concurrency::extent< 1 > inputExtent(leng);
                try
                {
                    concurrency::parallel_for_each(av,  inputExtent, [=](concurrency::index<1> idx) restrict(amp)
                    {
                        int globalId = idx[ 0 ];

                        if( globalId >= szElements)
                        return;
						iType2 m = map[ globalId ];
						result [ m ] = first1 [ globalId ] ;
						});	
                }
			    catch(std::exception &e)
                {
                      std::cout << "Exception while calling bolt::amp::scatter parallel_for_each"<<e.what()<<std::endl;
                      return;
                }
		result.getContainer().getBuffer(result, szElements).synchronize();
	};



	template< typename InputIterator1,
              typename MapIterator,
              typename InputIterator3,
              typename OutputIterator,
              typename Predicate >
	typename std::enable_if< std::is_same< typename std::iterator_traits< OutputIterator >::iterator_category ,
                                       std::random_access_iterator_tag
                                     >::value
                       >::type
    scatter_if( bolt::amp::control &ctl,
                const InputIterator1& first1,
                const InputIterator1& last1,
                const MapIterator& map,
                const InputIterator3& stencil,
                const OutputIterator& result,
                const Predicate& pred)
    {
  
		typedef typename std::iterator_traits<InputIterator1>::value_type iType1;
        typedef typename std::iterator_traits<MapIterator>::value_type iType2;
		typedef typename std::iterator_traits<InputIterator3>::value_type iType3;
        typedef typename std::iterator_traits<OutputIterator>::value_type oType;

        int sz = static_cast<int>( std::distance( first1, last1 ) );

        // Use host pointers memory since these arrays are only read once - no benefit to copying.
        // Map the input iterator to a device_vector
		device_vector< iType1, concurrency::array_view> dvInput( first1, last1, false, ctl );
		device_vector< iType2, concurrency::array_view> dvMap( map, sz, false, ctl );
		device_vector< iType3, concurrency::array_view> dvStencil( stencil, sz, false, ctl );
        // Map the result iterator to a device_vector
		device_vector< oType, concurrency::array_view> dvResult( result, sz, false, ctl );

		amp::scatter_if( ctl,
                        dvInput.begin(),
                        dvInput.end(),
                        dvMap.begin(),
					    dvStencil.begin(),
                        dvResult.begin( ),
					    pred);
		
        // This should immediately map/unmap the buffer
        dvResult.data( );
       
    }


	template< typename InputIterator,
              typename MapIterator,
              typename OutputIterator>
	typename std::enable_if< std::is_same< typename std::iterator_traits< OutputIterator >::iterator_category ,
                                       std::random_access_iterator_tag
                                     >::value
                       >::type
    scatter( bolt::amp::control &ctl,
             const InputIterator& first1,
             const InputIterator& last1,
             const MapIterator& map,
             const OutputIterator& result)
    {
  
		typedef typename std::iterator_traits<InputIterator>::value_type iType1;
        typedef typename std::iterator_traits<MapIterator>::value_type iType2;
        typedef typename std::iterator_traits<OutputIterator>::value_type oType;

        int sz = static_cast<int>( std::distance( first1, last1 ) );

        // Use host pointers memory since these arrays are only read once - no benefit to copying.
        // Map the input iterator to a device_vector
		device_vector< iType1, concurrency::array_view> dvInput( first1, last1, false, ctl );
		device_vector< iType2, concurrency::array_view> dvMap( map, sz, false, ctl );
        // Map the result iterator to a device_vector
		device_vector< oType, concurrency::array_view> dvResult( result, sz, false, ctl );

        amp::scatter( ctl,
                      dvInput.begin(),
                      dvInput.end(),
                      dvMap.begin(),
                      dvResult.begin( ));

        // This should immediately map/unmap the buffer
        dvResult.data( );

       
    }

}//end of namespace amp

	template< typename InputIterator1,
              typename InputIterator2,
              typename InputIterator3,
              typename OutputIterator,
	          typename Predicate>
    typename std::enable_if< 
               !(std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             std::input_iterator_tag 
                           >::value ||
               std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             bolt::amp::fancy_iterator_tag >::value ),
               void
                           >::type
    scatter_if( bolt::amp::control& ctl,
                const InputIterator1& first1,
                const InputIterator1& last1,
                const InputIterator2& map,
                const InputIterator3& stencil,
                const OutputIterator& result,
                const Predicate& pred)
    {   
		int sz = static_cast<int>( std::distance( first1, last1 ) );
        if (sz == 0)
            return;

        // Use host pointers memory since these arrays are only read once - no benefit to copying.
        bolt::amp::control::e_RunMode runMode = ctl.getForceRunMode();  // could be dynamic choice some day.
        if(runMode == bolt::amp::control::Automatic)
        {
          runMode = ctl.getDefaultPathToRun();
        }
        if( runMode == bolt::amp::control::SerialCpu )
        {
		    serial::scatter_if(ctl, first1, last1, map, stencil, result, pred);
        }
        else if( runMode == bolt::amp::control::MultiCoreCpu )
        {
#if defined( ENABLE_TBB )
            btbb::scatter_if(ctl, first1, last1, map, stencil, result, pred);
#else
            throw std::runtime_error( "The MultiCoreCpu version of scatter is not enabled to be built! \n" );

#endif
        }
        else
        {
            amp::scatter_if(ctl, first1, last1, map, stencil, result, pred);
        }

       
    };


	template< typename InputIterator1,
              typename InputIterator2,
              typename InputIterator3,
              typename OutputIterator,
	          typename Predicate>
    // Wrapper that uses default ::bolt::amp::control class, iterator interface
    typename std::enable_if< 
               (std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             std::input_iterator_tag 
                           >::value ||
               std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             bolt::amp::fancy_iterator_tag >::value ),
               void
                           >::type
    scatter_if( bolt::amp::control& ctl,
                     const InputIterator1& first1,
                     const InputIterator1& last1,
                     const InputIterator2& map,
                     const InputIterator3& stencil,
                     const OutputIterator& result,
                     const Predicate& pred,
                     const std::string& user_code )
    {
		//TODO: map cannot be a constant iterator! Throw compilation error for such a case.
        // TODO:  It should be possible to support non-random_access_iterator_tag iterators, if we copied the data
        // to a temporary buffer.  Should we?

        static_assert( std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                                     std::input_iterator_tag >::value , 
                       "Output vector should be a mutable vector. It cannot be of the type input_iterator_tag" );
        static_assert( std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                                     bolt::amp::fancy_iterator_tag >::value , 
                       "Output vector should be a mutable vector. It cannot be of type fancy_iterator_tag" );
    };




    template< typename InputIterator1,
              typename InputIterator2,
              typename OutputIterator>
	typename std::enable_if< 
               !(std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             std::input_iterator_tag 
                           >::value ||
               std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             bolt::amp::fancy_iterator_tag >::value ),
               void
                           >::type
    scatter( bolt::amp::control& ctl,
             const InputIterator1& first1,
             const InputIterator1& last1,
             const InputIterator2& map,
             const OutputIterator& result )
    {
       	
        int sz = static_cast<int>( std::distance( first1, last1 ) );
        if (sz == 0)
            return;

        // Use host pointers memory since these arrays are only read once - no benefit to copying.
        bolt::amp::control::e_RunMode runMode = ctl.getForceRunMode();  // could be dynamic choice some day.
        if(runMode == bolt::amp::control::Automatic)
        {
             runMode = ctl.getDefaultPathToRun();
        }
	  
        if( runMode == bolt::amp::control::SerialCpu )
		{
		    serial::scatter(ctl, first1, last1, map, result);
        }
        else if( runMode == bolt::amp::control::MultiCoreCpu )
        {
#if defined( ENABLE_TBB )           	   
            btbb::scatter(ctl, first1, last1, map, result);  
#else
            throw std::runtime_error( "The MultiCoreCpu version of scatter is not enabled to be built! \n" );

#endif
        }
        else 			
            amp::scatter(ctl, first1, last1, map, result);
    }



    template< typename InputIterator1,
              typename InputIterator2,
              typename OutputIterator>
	typename std::enable_if< 
               (std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             std::input_iterator_tag 
                           >::value ||
               std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             bolt::amp::fancy_iterator_tag >::value ),
               void
                           >::type
    scatter( bolt::amp::control& ctl,
             const InputIterator1& first1,
             const InputIterator1& last1,
             const InputIterator2& map,
             const OutputIterator& result )
    {
		//TODO: map cannot be a constant iterator! Throw compilation error for such a case.

        //static_assert( std::is_same< InputIterator1, std::input_iterator_tag >::value , "Bolt only supports random access iterator types" );
		static_assert( std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                                     std::input_iterator_tag >::value , 
                       "Output vector should be a mutable vector. It cannot be of the type input_iterator_tag" );
        static_assert( std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                                     bolt::amp::fancy_iterator_tag >::value , 
                       "Output vector should be a mutable vector. It cannot be of type fancy_iterator_tag" );
    }

} //End of detail namespace

////////////////////////////////////////////////////////////////////
// Scatter APIs
////////////////////////////////////////////////////////////////////
template< typename InputIterator1,
          typename InputIterator2,
          typename OutputIterator >
void scatter( bolt::amp::control& ctl,
              InputIterator1 first1,
              InputIterator1 last1,
              InputIterator2 map,
              OutputIterator result )
{
   detail::scatter( ctl,
                    first1,
                    last1,
                    map,
                    result);
}

template< typename InputIterator1,
          typename InputIterator2,
          typename OutputIterator >
void scatter( InputIterator1 first1,
              InputIterator1 last1,
              InputIterator2 map,
              OutputIterator result )
{
    scatter( control::getDefault( ),
             first1,
             last1,
             map,
             result);
}


////////////////////////////////////////////////////////////////////
// ScatterIf APIs
////////////////////////////////////////////////////////////////////
template< typename InputIterator1,
          typename InputIterator2,
          typename InputIterator3,
          typename OutputIterator >
void scatter_if( bolt::amp::control& ctl,
                 InputIterator1 first1,
                 InputIterator1 last1,
                 InputIterator2 map,
                 InputIterator3 stencil,
                 OutputIterator result )
{
    typedef typename  std::iterator_traits<InputIterator3>::value_type stencilType;
    detail::scatter_if( ctl,
                        first1,
                        last1,
                        map,
                        stencil,
                        result,
                        bolt::amp::identity <stencilType> ( ));
}

template< typename InputIterator1,
          typename InputIterator2,
          typename InputIterator3,
          typename OutputIterator >
void scatter_if( InputIterator1 first1,
                 InputIterator1 last1,
                 InputIterator2 map,
                 InputIterator3 stencil,
                 OutputIterator result)
{
    scatter_if( control::getDefault( ),
		        first1,
                last1,
                map,
                stencil,
                result );
}

template< typename InputIterator1,
          typename InputIterator2,
          typename InputIterator3,
          typename OutputIterator,
          typename Predicate >
void scatter_if( bolt::amp::control& ctl,
                 InputIterator1 first1,
                 InputIterator1 last1,
                 InputIterator2 map,
                 InputIterator3 stencil,
                 OutputIterator result,
                 Predicate pred )
{
    detail::scatter_if( ctl,
                        first1,
                        last1,
                        map,
                        stencil,
                        result,
                        pred);
}

template< typename InputIterator1,
          typename InputIterator2,
          typename InputIterator3,
          typename OutputIterator,
          typename Predicate >
void scatter_if( InputIterator1 first1,
                 InputIterator1 last1,
                 InputIterator2 map,
                 InputIterator3 stencil,
                 OutputIterator result,
                 Predicate pred )
{
    scatter_if( control::getDefault( ),
                first1,
                last1,
                map,
                stencil,
                result,
                pred);
}



} //End of amp namespace
} //End of bolt namespace

#endif
