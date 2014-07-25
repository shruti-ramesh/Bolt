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
#if !defined( BOLT_AMP_GATHER_INL )
#define BOLT_AMP_GATHER_INL
#define GATHER_WAVEFRNT_SIZE 64

#include <algorithm>
#include <type_traits>
#include "bolt/amp/bolt.h"
#include "bolt/amp/iterator/iterator_traits.h"
#include "bolt/amp/device_vector.h"
#include <amp.h>

#ifdef ENABLE_TBB
    #include "bolt/btbb/gather.h"
#endif

namespace bolt {
namespace amp {

namespace detail {

/* Begin-- Serial Implementation of the gather and gather_if routines */

namespace serial{

template< typename InputIterator1,
          typename InputIterator2,
          typename OutputIterator >

void gather(bolt::amp::control &ctl,
                   InputIterator1 mapfirst,
                   InputIterator1 maplast,
                   InputIterator2 input,
                   OutputIterator result)
{
   int numElements = static_cast< int >( std::distance( mapfirst, maplast ) );
   typedef typename  std::iterator_traits<InputIterator1>::value_type iType1;
   iType1 temp;
   for(int iter = 0; iter < numElements; iter++)
   {
                   temp = *(mapfirst + (int)iter);
                  *(result + (int)iter) = *(input + (int)temp);
   }
}


template< typename InputIterator1,
          typename InputIterator2,
          typename InputIterator3,
          typename OutputIterator,
          typename Predicate >

void gather_if(bolt::amp::control &ctl,
                      InputIterator1 mapfirst,
                      InputIterator1 maplast,
                      InputIterator2 stencil,
                      InputIterator3 input,
                      OutputIterator result,
                      Predicate pred)
{
   //std::cout<<"Serial code path ... \n";
   int numElements = static_cast< int >( std::distance( mapfirst, maplast ) );
   for(int iter = 0; iter < numElements; iter++)
   {
        if(pred(*(stencil + (int)iter)))
             result[(int)iter] = input[mapfirst[(int)iter]];
   }
}

}//end of namespace serial

namespace btbb{

template< typename InputIterator1,
          typename InputIterator2,
          typename OutputIterator >

void gather(bolt::amp::control &ctl,
                   InputIterator1 mapfirst,
                   InputIterator1 maplast,
                   InputIterator2 input,
                   OutputIterator result)
{
    bolt::btbb::gather(mapfirst, maplast, input, result);
}


template< typename InputIterator1,
          typename InputIterator2,
          typename InputIterator3,
          typename OutputIterator,
          typename Predicate >

void gather_if(bolt::amp::control &ctl,
                      InputIterator1 mapfirst,
                      InputIterator1 maplast,
                      InputIterator2 stencil,
                      InputIterator3 input,
                      OutputIterator result,
                      Predicate pred)
{
    bolt::btbb::gather_if(mapfirst, maplast, stencil, input, result, pred);
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
    gather_if( bolt::amp::control &ctl,
                            const DVInputIterator1& map_first,
                            const DVInputIterator1& map_last,
                            const DVInputIterator2& stencil,
                            const DVInputIterator3& input,
                            const DVOutputIterator& result,
                            const Predicate& pred )
    {
		concurrency::accelerator_view av = ctl.getAccelerator().default_view;
        typedef typename std::iterator_traits<DVInputIterator1>::value_type iType1;
        typedef typename std::iterator_traits<DVInputIterator2>::value_type iType2;
        typedef typename std::iterator_traits<DVInputIterator3>::value_type iType3;
        typedef typename std::iterator_traits<DVOutputIterator>::value_type oType;

       int szElements = static_cast< int >(std::distance( map_first, map_last));
		const int leng =  szElements + GATHER_WAVEFRNT_SIZE - (szElements % GATHER_WAVEFRNT_SIZE);
		concurrency::extent< 1 > inputExtent(leng);
		try
                {
                    concurrency::parallel_for_each(av,  inputExtent, [=](concurrency::index<1> idx) restrict(amp)
                    {
                        int globalId = idx[ 0 ];
                        if( globalId >= szElements)
                        return;

						iType1 m = map_first[ globalId ];
						iType2 s = stencil[ globalId ];
						if ( pred( s ) )
						{
							result [ globalId ] = input [ m ] ;
						}
						});	
                }
			    catch(std::exception &e)
                {
                      std::cout << "Exception while calling bolt::amp::gather parallel_for_each"<<e.what()<<std::endl;
					  return;
                }	
	};

	template< typename InputIterator1,
              typename InputIterator2,
              typename InputIterator3,
              typename OutputIterator,
              typename Predicate >
	typename std::enable_if< std::is_same< typename std::iterator_traits< OutputIterator >::iterator_category ,
                                       std::random_access_iterator_tag
                                     >::value
                       >::type
    gather_if( bolt::amp::control &ctl,
                            const InputIterator1& map_first,
                            const InputIterator1& map_last,
                            const InputIterator2& stencil,
                            const InputIterator3& input,
                            const OutputIterator& result,
                            const Predicate& pred)
    {
		typedef typename std::iterator_traits<InputIterator1>::value_type iType1;
        typedef typename std::iterator_traits<InputIterator2>::value_type iType2;
		typedef typename std::iterator_traits<InputIterator3>::value_type iType3;
        typedef typename std::iterator_traits<OutputIterator>::value_type oType;

        int sz = static_cast<int>( std::distance( map_first, map_last ) );

        // Map the input iterator to a device_vector
		device_vector< iType1, concurrency::array_view> dvMap( map_first, map_last, false, ctl );
		device_vector< iType2, concurrency::array_view> dvStencil( stencil, sz, false, ctl );
		device_vector< iType3, concurrency::array_view> dvInput( input, sz, false, ctl );

        // Map the output iterator to a device_vector
		device_vector< oType, concurrency::array_view> dvResult( result, sz, false, ctl );
        amp::gather_if( ctl,
                            dvMap.begin( ),
                            dvMap.end( ),
                            dvStencil.begin( ),
                            dvInput.begin( ),
                            dvResult.begin( ),
                            pred);
		
        // This should immediately map/unmap the buffer
        dvResult.data( );

	}

    template< typename DVInputIterator1,
              typename DVInputIterator2,
              typename DVOutputIterator >
    typename std::enable_if< std::is_same< typename std::iterator_traits< DVOutputIterator >::iterator_category ,
                                       bolt::amp::device_vector_tag
                                     >::value
                       >::type
    gather( bolt::amp::control &ctl,
                         const DVInputIterator1& map_first,
                         const DVInputIterator1& map_last,
                         const DVInputIterator2& input,
                         const DVOutputIterator& result )
    {
		concurrency::accelerator_view av = ctl.getAccelerator().default_view;
        typedef typename std::iterator_traits<DVInputIterator1>::value_type iType1;
        typedef typename std::iterator_traits<DVInputIterator2>::value_type iType2;
        typedef typename std::iterator_traits<DVOutputIterator>::value_type oType;
        int szElements = static_cast< int >(std::distance( map_first, map_last));
		const unsigned int leng =  szElements + GATHER_WAVEFRNT_SIZE - (szElements % GATHER_WAVEFRNT_SIZE);
		concurrency::extent< 1 > inputExtent(leng);
		try
                {
                    concurrency::parallel_for_each(av,  inputExtent, [=](concurrency::index<1> idx) restrict(amp)
                    {
                        int globalId = idx[ 0 ];

                        if( globalId >= szElements)
                        return;
						iType1 m = map_first[ globalId ];
						result [ globalId ] = input [ m ] ;
						});	
                }
			    catch(std::exception &e)
                {
                      std::cout << "Exception while calling bolt::amp::gather parallel_for_each"<<e.what()<<std::endl;
					  return;
                }
    };


	template< typename InputIterator1,
              typename InputIterator2,
              typename OutputIterator >
	typename std::enable_if< std::is_same< typename std::iterator_traits< OutputIterator >::iterator_category ,
                                       std::random_access_iterator_tag
                                     >::value
                       >::type
    gather( bolt::amp::control &ctl,
                         const InputIterator1& map_first,
                         const InputIterator1& map_last,
                         const InputIterator2& input,
                         const OutputIterator& result)
    {

		typedef typename std::iterator_traits<InputIterator1>::value_type iType1;
        typedef typename std::iterator_traits<InputIterator2>::value_type iType2;
        typedef typename std::iterator_traits<OutputIterator>::value_type oType;

        int sz = static_cast<int>( std::distance( map_first, map_last ) );

        // Map the input iterator to a device_vector		  
		device_vector< iType1, concurrency::array_view> dvMap( map_first, map_last, false, ctl );
		device_vector< iType2, concurrency::array_view> dvInput( input, sz, false, ctl );
        // Map the output iterator to a device_vector
		device_vector< oType, concurrency::array_view> dvResult( result, sz, false, ctl );
        amp::gather( ctl,
                        dvMap.begin( ),
                        dvMap.end( ),
                        dvInput.begin( ),
                        dvResult.begin( ) );
	    
        // This should immediately map/unmap the buffer
        dvResult.data( );

	}

} // end of amp namespace

    template< typename InputIterator1,
              typename InputIterator2,
              typename InputIterator3,
              typename OutputIterator,
              typename Predicate >
	typename std::enable_if< 
               !(std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             std::input_iterator_tag 
                           >::value ||
               std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             bolt::amp::fancy_iterator_tag >::value ),
               void
                           >::type
    gather_if( bolt::amp::control& ctl,
               const InputIterator1& map_first,
               const InputIterator1& map_last,
               const InputIterator2& stencil,
               const InputIterator3& input,
               const OutputIterator& result,
               const Predicate& pred )
    {
        
		int sz = static_cast<int>( std::distance( map_first, map_last ) );
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
		    serial::gather_if(ctl, map_first, map_last, stencil, input, result, pred);
        }
        else if( runMode == bolt::amp::control::MultiCoreCpu )
        {
#if defined( ENABLE_TBB )
           btbb::gather_if(ctl, map_first, map_last, stencil, input, result, pred);
#else
           throw std::runtime_error( "The MultiCoreCpu version of gather is not enabled to be built! \n" );
#endif
        }
        else
        {
            amp::gather_if(ctl, map_first, map_last, stencil, input, result, pred);
        }
    }


    template< typename InputIterator1,
              typename InputIterator2,
              typename InputIterator3,
              typename OutputIterator,
              typename Predicate >
	typename std::enable_if< 
               (std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             std::input_iterator_tag 
                           >::value ||
               std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             bolt::amp::fancy_iterator_tag >::value ),
               void
                           >::type
    gather_if( bolt::amp::control& ctl,
                                         const InputIterator1& map_first,
                                         const InputIterator1& map_last,
                                         const InputIterator2& stencil,
                                         const InputIterator3& input,
                                         const OutputIterator& result,
                                         const Predicate& pred )
    {
        //static_assert( std::is_same< InputIterator1, std::input_iterator_tag >::value , "Bolt only supports random access iterator types" );
		static_assert( std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                                     std::input_iterator_tag >::value , 
                       "Output vector should be a mutable vector. It cannot be of the type input_iterator_tag" );
        static_assert( std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                                     bolt::amp::fancy_iterator_tag >::value , 
                       "Output vector should be a mutable vector. It cannot be of type fancy_iterator_tag" );
    };


    template< typename InputIterator1,
              typename InputIterator2,
              typename OutputIterator >
	typename std::enable_if< 
               !(std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             std::input_iterator_tag 
                           >::value ||
               std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             bolt::amp::fancy_iterator_tag >::value ),
               void
                           >::type
    gather( bolt::amp::control& ctl,
            const InputIterator1& map_first,
            const InputIterator1& map_last,
            const InputIterator2& input,
            const OutputIterator& result)
    {
        
        int sz = static_cast<int>( std::distance( map_first, map_last ) );
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
		   serial::gather(ctl, map_first, map_last, input, result);
        }
        else if( runMode == bolt::amp::control::MultiCoreCpu )
        {
#if defined( ENABLE_TBB )
           btbb::gather(ctl, map_first, map_last , input, result);
#else
           throw std::runtime_error( "The MultiCoreCpu version of gather is not enabled to be built! \n" );
#endif
        }
        else
        {		
            amp::gather(ctl, map_first, map_last, input, result);
        }


    }

    template< typename InputIterator1,
              typename InputIterator2,
              typename OutputIterator >
	typename std::enable_if< 
               (std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             std::input_iterator_tag 
                           >::value ||
               std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             bolt::amp::fancy_iterator_tag >::value ),
               void
                           >::type
    gather( bolt::amp::control& ctl,
            const InputIterator1& map_first,
            const InputIterator1& map_last,
            const InputIterator2& input,
            const OutputIterator& result)
    {
        //static_assert( std::is_same< InputIterator1, std::input_iterator_tag >::value , "Bolt only supports random access iterator types" );
		static_assert( std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                                     std::input_iterator_tag >::value , 
                       "Output vector should be a mutable vector. It cannot be of the type input_iterator_tag" );
        static_assert( std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                                     bolt::amp::fancy_iterator_tag >::value , 
                       "Output vector should be a mutable vector. It cannot be of type fancy_iterator_tag" );
    };


} //End of detail namespace

////////////////////////////////////////////////////////////////////
// Gather APIs
////////////////////////////////////////////////////////////////////
template< typename InputIterator1,
          typename InputIterator2,
          typename OutputIterator >
void gather( bolt::amp::control& ctl,
             InputIterator1 map_first,
             InputIterator1 map_last,
             InputIterator2 input,
             OutputIterator result )
{
    detail::gather( ctl,
                    map_first,
                    map_last,
                    input,
                    result);
}

template< typename InputIterator1,
          typename InputIterator2,
          typename OutputIterator >
void gather( InputIterator1 map_first,
             InputIterator1 map_last,
             InputIterator2 input,
             OutputIterator result)
{
    gather( control::getDefault( ),
    map_first,
    map_last,
    input,
    result);
}


////////////////////////////////////////////////////////////////////
// GatherIf APIs
////////////////////////////////////////////////////////////////////
template< typename InputIterator1,
          typename InputIterator2,
          typename InputIterator3,
          typename OutputIterator >
void gather_if( bolt::amp::control& ctl,
                InputIterator1 map_first,
                InputIterator1 map_last,
                InputIterator2 stencil,
                InputIterator3 input,
                OutputIterator result)
{
    typedef typename std::iterator_traits<InputIterator2>::value_type stencilType;
    detail::gather_if( ctl,
               map_first,
               map_last,
               stencil,
               input,
               result,
               bolt::amp::identity <stencilType> ( ) );
}

template< typename InputIterator1,
          typename InputIterator2,
          typename InputIterator3,
          typename OutputIterator >
void gather_if( InputIterator1 map_first,
                InputIterator1 map_last,
                InputIterator2 stencil,
                InputIterator3 input,
                OutputIterator result )
{
   gather_if( control::getDefault( ),
		       map_first,
               map_last,
               stencil,
               input,
               result );
}

template< typename InputIterator1,
          typename InputIterator2,
          typename InputIterator3,
          typename OutputIterator,
          typename Predicate >
void gather_if( bolt::amp::control& ctl,
                InputIterator1 map_first,
                InputIterator1 map_last,
                InputIterator2 stencil,
                InputIterator3 input,
                OutputIterator result,
                Predicate pred )
{
     detail::gather_if( ctl,
                       map_first,
                       map_last,
                       stencil,
                       input,
                       result,
                       pred);
}

template< typename InputIterator1,
          typename InputIterator2,
          typename InputIterator3,
          typename OutputIterator,
          typename Predicate >
void gather_if(  InputIterator1 map_first,
                 InputIterator1 map_last,
                 InputIterator2 stencil,
                 InputIterator3 input,
                 OutputIterator result,
                 Predicate pred)
{
    gather_if( control::getDefault( ),
               map_first,
               map_last,
               stencil,
               input,
               result,
               pred);
}


} //End of cl namespace
} //End of bolt namespace

#endif
