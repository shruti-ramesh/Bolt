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

///////////////////////////////////////////////////////////////////////////////
// AMP Transform
//////////////////////////////////////////////////////////////////////////////

#pragma once
#if !defined( BOLT_AMP_TRANSFORM_INL )
#define BOLT_AMP_TRANSFORM_INL
#define TRANSFORM_WAVEFRNT_SIZE 256

#ifdef BOLT_ENABLE_PROFILING
#include "bolt/AsyncProfiler.h"
//AsyncProfiler aProfiler("transform");
#endif

#include <algorithm>
#include <type_traits>
#include "bolt/amp/bolt.h"
#include "bolt/amp/device_vector.h"
#include "bolt/amp/iterator/iterator_traits.h"

#ifdef ENABLE_TBB
    #include "bolt/btbb/transform.h"
#endif



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace bolt
{
namespace amp
{
namespace detail
{
            
namespace serial{

    template<typename InputIterator1, typename InputIterator2, typename OutputIterator, typename BinaryFunction>
    void binary_transform( bolt::amp::control &ctl, const InputIterator1& first1, const InputIterator1& last1,
                        const InputIterator2& first2, const OutputIterator& result, const BinaryFunction& f)
    {
        size_t sz = (last1 - first1);
        if (sz == 0)
            return;
        for(int index=0; index < (int)(sz); index++)
        {
            *(result + index) = f( *(first1+index), *(first2+index) );
        }
    }

    template<typename Iterator, typename OutputIterator, typename UnaryFunction>
    void unary_transform( bolt::amp::control &ctl, Iterator& first, Iterator& last,
                    OutputIterator& result, UnaryFunction& f )
    {
        size_t sz = (last - first);
        if (sz == 0)
            return;
        for(int index=0; index < (int)(sz); index++)
        {
            *(result + index) = f( *(first+index) );
        }

        return;
    }
}//end of namespace serial


namespace btbb{
    template<typename InputIterator1, typename InputIterator2, typename OutputIterator, typename BinaryFunction>
    void binary_transform( bolt::amp::control &ctl, const InputIterator1& first1, const InputIterator1& last1,
                        const InputIterator2& first2, const OutputIterator& result, const BinaryFunction& f)
    {
        bolt::btbb::transform(first1, last1, first2, result, f);
    }

    template<typename Iterator, typename OutputIterator, typename UnaryFunction>
    void unary_transform( bolt::amp::control &ctl, Iterator& first, Iterator& last,
                    OutputIterator& result, UnaryFunction& f )
    {
        bolt::btbb::transform(first, last, result, f);
    }
}//end of namespace btbb


namespace amp{

    /*! \brief This template function overload is used strictly for device_vector and AMP implementations. 
        \detail 
    */
    template<typename DVInputIterator1, typename DVInputIterator2, typename DVOutputIterator, typename BinaryFunction>
    typename std::enable_if< 
               std::is_same< typename std::iterator_traits< DVOutputIterator >::iterator_category ,
                                       bolt::amp::device_vector_tag
                           >::value
                           >::type
    binary_transform( ::bolt::amp::control &ctl, const DVInputIterator1& first1, const DVInputIterator1& last1,
                      const DVInputIterator2& first2, const DVOutputIterator& result, const BinaryFunction& f)
           
    {
        concurrency::accelerator_view av = ctl.getAccelerator().default_view;

        typedef std::iterator_traits< DVInputIterator1 >::value_type iType1;
        typedef std::iterator_traits< DVInputIterator2 >::value_type iType2;
        typedef std::iterator_traits< DVOutputIterator >::value_type oType;

        const int szElements =  static_cast< int >( std::distance( first1, last1 ) );

        const unsigned int leng =  szElements + TRANSFORM_WAVEFRNT_SIZE - (szElements % TRANSFORM_WAVEFRNT_SIZE);

        concurrency::extent< 1 > inputExtent(leng);

        try
        {

            concurrency::parallel_for_each(av,  inputExtent, [=](concurrency::index<1> idx) restrict(amp)
            {
                int globalId = idx[ 0 ];

                if( globalId >= szElements)
                return;

                result[globalId] = f(first1[globalId],first2[globalId]);
            });
        }

		catch(std::exception &e)
        {

              std::cout << "Exception while calling bolt::amp::transform parallel_for_each"<<e.what()<<std::endl;

              return;
        }
    }



	/*! \brief This template function overload is used strictly std random access vectors and AMP implementations. 
        \detail 
    */
    template<typename InputIterator1, typename InputIterator2, typename OutputIterator, typename BinaryFunction>
    typename std::enable_if< std::is_same< typename std::iterator_traits< OutputIterator >::iterator_category ,
                                       std::random_access_iterator_tag
                                     >::value
                           >::type
    binary_transform( ::bolt::amp::control &ctl, const InputIterator1& first1, const InputIterator1& last1,
                      const InputIterator2& first2, const OutputIterator& result, const BinaryFunction& f)
    {
        int sz = static_cast<int>(last1 - first1);
        if (sz == 0)
            return;
        typedef typename std::iterator_traits<InputIterator1>::value_type  iType1;
        typedef typename std::iterator_traits<InputIterator2>::value_type  iType2;
        typedef typename std::iterator_traits<OutputIterator>::value_type  oType;

		// Use host pointers memory since these arrays are only read once - no benefit to copying.
        // Map the input iterator to a device_vector
		device_vector< iType1, concurrency::array_view > dvInput1( first1, last1, false, ctl );
		device_vector< iType2, concurrency::array_view > dvInput2( first2, sz, false, ctl );
        // Map the output iterator to a device_vector
        device_vector< oType, concurrency::array_view > dvOutput( result, sz, true, ctl );

        amp::binary_transform( ctl, dvInput1.begin( ), dvInput1.end( ), dvInput2.begin( ), dvOutput.begin( ), f );

        // This should immediately map/unmap the buffer
        dvOutput.data( );
		return;

    }


	template<typename DVInputIterator, typename DVOutputIterator, typename UnaryFunction>
    typename std::enable_if< std::is_same< typename std::iterator_traits< DVOutputIterator >::iterator_category ,
                                       bolt::amp::device_vector_tag
                                     >::value
                       >::type
    unary_transform( ::bolt::amp::control &ctl, const DVInputIterator& first, const DVInputIterator& last,
    const DVOutputIterator& result, const UnaryFunction& f)
    {
          
        typedef std::iterator_traits< DVInputIterator >::value_type iType;
        typedef std::iterator_traits< DVOutputIterator >::value_type oType;


        const int szElements =  static_cast< int >( std::distance( first, last ) );
        concurrency::accelerator_view av = ctl.getAccelerator().default_view;

        const unsigned int leng =  szElements + TRANSFORM_WAVEFRNT_SIZE - (szElements % TRANSFORM_WAVEFRNT_SIZE);

        concurrency::extent< 1 > inputExtent(leng);

        try
        {

            concurrency::parallel_for_each(av,  inputExtent, [=](concurrency::index<1> idx) restrict(amp)
            {
                int globalId = idx[ 0 ];

                if( globalId >= szElements)
                return;

                result[globalId] = f(first[globalId]);
            });
        }

		catch(std::exception &e)
        {

               std::cout << "Exception while calling bolt::amp::transform parallel_for_each"<<e.what()<<std::endl;

               return;
        }
    }


	/*! \brief This template function overload is used strictly std random access vectors and AMP implementations. 
        \detail 
    */
    template<typename InputIterator, typename OutputIterator, typename UnaryFunction>
    typename std::enable_if< std::is_same< typename std::iterator_traits< OutputIterator >::iterator_category ,
                                       std::random_access_iterator_tag
                                     >::value
                           >::type
    unary_transform( ::bolt::amp::control &ctl, const InputIterator& first, const InputIterator& last,
    const OutputIterator& result, const UnaryFunction& f)
    {
        int sz = static_cast<int>(last - first);
        if (sz == 0)
            return;
        typedef typename std::iterator_traits<InputIterator>::value_type  iType;
        typedef typename std::iterator_traits<OutputIterator>::value_type oType;
       
		// Use host pointers memory since these arrays are only read once - no benefit to copying.

        // Map the input iterator to a device_vector
        //device_vector< iType > dvInput( first, last, ctl );
        device_vector< iType, concurrency::array_view > dvInput( first, last, false, ctl );

        // Map the output iterator to a device_vector
        device_vector< oType, concurrency::array_view > dvOutput( result, sz, true, ctl );

        amp::unary_transform( ctl, dvInput.begin( ), dvInput.end( ), dvOutput.begin( ), f );

        // This should immediately map/unmap the buffer
        dvOutput.data( );
		return;

    }

}//namespace amp


	/*! \brief This template function overload is used strictly for device vectors and std random access vectors. 
        \detail Here we branch out into the SerialCpu, MultiCore TBB or The AMP code paths. 
    */
    template<typename InputIterator1, typename InputIterator2, typename OutputIterator, typename BinaryFunction>
    typename std::enable_if< 
             !(std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             std::input_iterator_tag 
                           >::value ||
               std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             bolt::amp::fancy_iterator_tag >::value) 
                           >::type
    binary_transform(::bolt::amp::control& ctl, const InputIterator1& first1, const InputIterator1& last1, 
                     const InputIterator2& first2, const OutputIterator& result, const BinaryFunction& f)
    {
        const int sz =  static_cast< int >( std::distance( first1, last1 )); 
        if (sz == 0)
            return;

        bolt::amp::control::e_RunMode runMode = ctl.getForceRunMode();  // could be dynamic choice some day.
        if(runMode == bolt::amp::control::Automatic)
        {
           runMode = ctl.getDefaultPathToRun();
        }
		
        if( runMode == bolt::amp::control::SerialCpu )
        {
            serial::binary_transform(ctl, first1, last1, first2, result, f );
            return;
        }
        else if( runMode == bolt::amp::control::MultiCoreCpu )
        {
#if defined( ENABLE_TBB )
            btbb::binary_transform(ctl, first1, last1, first2, result, f);
#else
            throw std::runtime_error( "The MultiCoreCpu version of transform is not enabled to be built! \n" );
#endif
            return;
        }
        else
        {
            amp::binary_transform( ctl, first1, last1, first2, result, f );
            return;
        }       
        return;
    }
    

     
    /*! \brief This template function overload is used to seperate input_iterator and fancy_iterator as 
        destination iterators from all other iterators
        \detail This template function overload is used to seperate input_iterator and fancy_iterator as 
        destination iterators from all other iterators. We enable this overload and should result 
        in a compilation failure.
    */
    // TODO - test the below code path
    template<typename InputIterator1, typename InputIterator2, typename OutputIterator, typename BinaryFunction>
    typename std::enable_if< 
               std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             std::input_iterator_tag 
                           >::value ||
               std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             bolt::amp::fancy_iterator_tag >::value 
                           >::type
    binary_transform(::bolt::amp::control& ctl, const InputIterator1& first1, const InputIterator1& last1, 
                     const InputIterator2& first2, const OutputIterator& result, const BinaryFunction& f)
    {
        //TODO - Shouldn't we support transform for input_iterator_tag also. 
        static_assert( std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                                     std::input_iterator_tag >::value , 
                       "Output vector should be a mutable vector. It cannot be of the type input_iterator_tag" );
        static_assert( std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                                     bolt::amp::fancy_iterator_tag >::value , 
                       "Output vector should be a mutable vector. It cannot be of type fancy_iterator_tag" );
    }



	 /*! \brief This template function overload is used strictly for device vectors and std random access vectors. 
        \detail Here we branch out into the SerialCpu, MultiCore TBB or The AMP code paths. 
    */
    template<typename InputIterator, typename OutputIterator, typename UnaryFunction>
    typename std::enable_if< 
             !(std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             std::input_iterator_tag 
                           >::value ||
               std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             bolt::amp::fancy_iterator_tag >::value) 
                           >::type
    unary_transform(::bolt::amp::control& ctl, InputIterator& first,
         InputIterator& last,  OutputIterator& result,  UnaryFunction& f)
    {
        const int sz =  static_cast< int >( std::distance( first, last ) );
        if (sz == 0)
            return;

        bolt::amp::control::e_RunMode runMode = ctl.getForceRunMode();  // could be dynamic choice some day.
        if(runMode == bolt::amp::control::Automatic)
        {
           runMode = ctl.getDefaultPathToRun();
        }
		
        if( runMode == bolt::amp::control::SerialCpu )
        {
            serial::unary_transform(ctl, first, last, result, f );
            return;
        }
        else if( runMode == bolt::amp::control::MultiCoreCpu )
        {
#if defined( ENABLE_TBB )
          
            btbb::unary_transform(ctl, first, last, result, f);
#else
            throw std::runtime_error( "The MultiCoreCpu version of transform is not enabled to be built! \n" );
#endif
            return;
        }
        else
        {
            amp::unary_transform( ctl, first, last, result, f );
            return;
        }       
        return;
    }
    

          
    /*! \brief This template function overload is used to seperate input_iterator and fancy_iterator as destination iterators from all other iterators
        \detail This template function overload is used to seperate input_iterator and fancy_iterator as destination iterators from all other iterators. 
                We enable this overload and should result in a compilation failure.
    */
    // TODO - test the below code path
    template<typename InputIterator, typename OutputIterator, typename UnaryFunction>
    typename std::enable_if< 
               std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             std::input_iterator_tag 
                           >::value ||
               std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                             bolt::amp::fancy_iterator_tag >::value 
                           >::type
    unary_transform(::bolt::amp::control& ctl, const InputIterator& first1,
        const InputIterator& last1, const OutputIterator& result, const UnaryFunction& f)
    {
        //TODO - Shouldn't we support transform for input_iterator_tag also. 
        static_assert( std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                                     std::input_iterator_tag >::value , 
                       "Output vector should be a mutable vector. It cannot be of the type input_iterator_tag" );
        static_assert( std::is_same< typename std::iterator_traits< OutputIterator>::iterator_category, 
                                     bolt::amp::fancy_iterator_tag >::value , 
                       "Output vector should be a mutable vector. It cannot be of type fancy_iterator_tag" );
    }


}//end of namespace detail


        //////////////////////////////////////////
        //  Transform overloads
        //////////////////////////////////////////
        // default control, two-input transform, std:: iterator
        template<typename InputIterator1, typename InputIterator2, typename OutputIterator, typename BinaryFunction>
        void transform( bolt::amp::control& ctl,
                       InputIterator1 first1,
                       InputIterator1 last1,
                       InputIterator2 first2,
                       OutputIterator result,
                       BinaryFunction f )
        {
			  using bolt::amp::detail::binary_transform;
              binary_transform( ctl, first1, last1, first2, result, f );

        }


        // default control, two-input transform, std:: iterator
        template<typename InputIterator1, typename InputIterator2, typename OutputIterator, typename BinaryFunction>
        void transform( InputIterator1 first1,
                        InputIterator1 last1,
                        InputIterator2 first2,
                        OutputIterator result,
                        BinaryFunction f )
        {
              using bolt::amp::transform;
              transform( control::getDefault(), first1, last1, first2, result, f);
        }

        // default control, two-input transform, std:: iterator
        template<typename InputIterator, typename OutputIterator, typename UnaryFunction>
        void transform( bolt::amp::control& ctl,
                        InputIterator first1,
                        InputIterator last1,
                        OutputIterator result,
                        UnaryFunction f )
        {
              using bolt::amp::detail::unary_transform;
              unary_transform( ctl, first1, last1, result, f);
        }

        // default control, two-input transform, std:: iterator
        template<typename InputIterator, typename OutputIterator, typename UnaryFunction>
        void transform( InputIterator first1,
                        InputIterator last1,
                        OutputIterator result,
                        UnaryFunction f )
        {
              using bolt::amp::transform;
              transform( control::getDefault(), first1, last1, result, f );
        }


    } //end of namespace amp
} //end of namespace bolt

#endif // AMP_TRANSFORM_INL