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
#pragma warning(disable:4244)

#include "common/stdafx.h"
#include <vector>
#include <array>
//#include "bolt/cl/iterator/counting_iterator.h"
#include "bolt/cl/iterator/transform_iterator.h"
#include "bolt/cl/copy.h"
#include "bolt/cl/transform_scan.h"
#include "bolt/cl/transform_reduce.h"
#include "bolt/cl/transform.h"
#include "bolt/cl/count.h"
#include "bolt/cl/reduce.h"
#include "bolt/cl/reduce_by_key.h"
#include "bolt/cl/generate.h"
#include "bolt/cl/inner_product.h"
#include "bolt/cl/scatter.h"
#include "bolt/cl/gather.h"
#include "bolt/cl/functional.h"
#include "bolt/cl/distance.h"
#include "bolt/miniDump.h"
#include "bolt/unicode.h"

#include <gtest/gtest.h>
#include "common/test_common.h"
#include <boost/program_options.hpp>
#define BCKND cl

namespace po = boost::program_options;

BOLT_FUNCTOR(square,
    struct square
    {
        int operator() (const int x)  const { return x + 2; }
        typedef int result_type;
    };
);

BOLT_FUNCTOR(add_4,
    struct add_4
    {
        int operator() (const int x)  const { return x + 4; }
        typedef int result_type;
    };
);

BOLT_FUNCTOR(add_3,
    struct add_3
    {
        int operator() (const int x)  const { return x + 3; }
        typedef int result_type;
    };
);


BOLT_FUNCTOR(add_0,
    struct add_0
    {
        int operator() (const int x)  const { return x; }
        typedef int result_type;
    };
);


int global_id = 0;

int get_global_id(int i)
{
    return global_id++;
}

BOLT_FUNCTOR(gen_input,
    struct gen_input
    {
        int operator() ()  const { return get_global_id(0); }
        typedef int result_type;
    };
);

BOLT_FUNCTOR(UDD, 
struct UDD
{
    int i;
    float f;
  
    bool operator == (const UDD& other) const {
        return ((i == other.i) && (f == other.f));
    }

	UDD operator + (const UDD &rhs) const
    {
      UDD _result;
      _result.i = i + rhs.i;
      _result.f = f + rhs.f;
      return _result;
    }

	UDD operator-() const
    {
        UDD r;
        r.i = -i;
        r.f = -f;
        return r;
    }
    
    UDD()
        : i(0), f(0) { }
    UDD(int _in)
        : i(_in), f((float)(_in+2) ){ }
};
);

BOLT_FUNCTOR(squareUDD_result_float,
    struct squareUDD_result_float
    {
        float operator() (const UDD& x)  const 
        { 
            return ((float)x.i + x.f);
        }
        typedef float result_type;
    };
);

BOLT_FUNCTOR(squareUDD_resultUDD,
    struct squareUDD_resultUDD
    {
        UDD operator() (const UDD& x)  const 
        { 
            UDD tmp;
            tmp.i = x.i * x.i;
            tmp.i = x.f * x.f;
            return tmp;
        }
        typedef UDD result_type;
    };
);

BOLT_FUNCTOR(cubeUDD,
    struct cubeUDD
    {
        float operator() (const UDD& x)  const 
        { 
            return ((float)x.i + x.f + 3.0f);
        }
        typedef float result_type;
    };
);


BOLT_FUNCTOR(cubeUDD_resultUDD,
    struct cubeUDD_resultUDD
    {
        UDD operator() (const UDD& x)  const 
        { 
            UDD tmp;
            tmp.i = x.i * x.i * x.i;
            tmp.i = x.f * x.f * x.f;
            return tmp;
        }
        typedef UDD result_type;
    };
);


BOLT_FUNCTOR( UDDplus,
struct UDDplus
{
   UDD operator() (const UDD &lhs, const UDD &rhs) const
   {
     UDD _result;
     _result.i = lhs.i + rhs.i;
     _result.f = lhs.f + rhs.f;
     return _result;
   }

};
);


/*Create Device Vector Iterators*/
BOLT_TEMPLATE_REGISTER_NEW_ITERATOR( bolt::cl::device_vector, int, UDD);

/*Create Transform iterators*/
BOLT_TEMPLATE_REGISTER_NEW_TRANSFORM_ITERATOR( bolt::cl::transform_iterator, square, UDD);
BOLT_TEMPLATE_REGISTER_NEW_TRANSFORM_ITERATOR( bolt::cl::transform_iterator, add_3, int);
BOLT_TEMPLATE_REGISTER_NEW_TRANSFORM_ITERATOR( bolt::cl::transform_iterator, add_4, int);
BOLT_TEMPLATE_REGISTER_NEW_TRANSFORM_ITERATOR( bolt::cl::transform_iterator, add_0, int);

BOLT_TEMPLATE_REGISTER_NEW_TRANSFORM_ITERATOR( bolt::cl::transform_iterator, squareUDD_result_float, UDD);
BOLT_TEMPLATE_REGISTER_NEW_TRANSFORM_ITERATOR( bolt::cl::transform_iterator, squareUDD_resultUDD, UDD);
BOLT_TEMPLATE_REGISTER_NEW_TRANSFORM_ITERATOR( bolt::cl::transform_iterator, cubeUDD_resultUDD, UDD);
BOLT_TEMPLATE_REGISTER_NEW_TRANSFORM_ITERATOR( bolt::cl::transform_iterator, cubeUDD, UDD);

BOLT_TEMPLATE_REGISTER_NEW_TYPE( bolt::cl::constant_iterator, int, UDD );

BOLT_TEMPLATE_REGISTER_NEW_TYPE(bolt::cl::plus, float, UDD );
BOLT_TEMPLATE_REGISTER_NEW_TYPE(bolt::cl::negate, float, UDD );

BOLT_FUNCTOR(gen_input_udd,
    struct gen_input_udd
    {
        UDD operator() ()  const 
       { 
            int i=get_global_id(0);
            UDD temp;
            temp.i = i;
            temp.f = (float)i;
            return temp; 
        }
        typedef int result_type;
    };
);


BOLT_FUNCTOR( is_even,				  
struct is_even{
    bool operator () (int x)
    {
        return ( (x % 2)==0);
    }
};
);

template<
    typename oType,
    typename BinaryFunction,
    typename T>
oType*
Serial_scan(
    oType *values,
    oType *result,
    unsigned int  num,
    const BinaryFunction binary_op,
    const bool Incl,
    const T &init)
{
    oType  sum, temp;
    if(Incl){
      *result = *values; // assign value
      sum = *values;
    }
    else {
        temp = *values;
       *result = (oType)init;
       sum = binary_op( *result, temp);
    }
    for ( unsigned int i= 1; i<num; i++)
    {
        oType currentValue = *(values + i); // convertible
        if (Incl)
        {
            oType r = binary_op( sum, currentValue);
            *(result + i) = r;
            sum = r;
        }
        else // new segment
        {
            *(result + i) = sum;
            sum = binary_op( sum, currentValue);

        }
    }
    return result;
}

template<
    typename kType,
    typename vType,
    typename koType,
    typename voType,
    typename BinaryFunction>
unsigned int
Serial_reduce_by_key(
	kType* keys,
	vType* values,
	koType* keys_output,
	voType* values_output,
	BinaryFunction binary_op,
	unsigned int  numElements
	)
{
 
    static_assert( std::is_convertible< vType, voType >::value,
                   "InputIterator2 and OutputIterator's value types are not convertible." );

    // do zeroeth element
    values_output[0] = values[0];
    keys_output[0] = keys[0];
    unsigned int count = 1;
    // rbk oneth element and beyond

    unsigned int vi=1, vo=0, ko=0;
    for ( unsigned int i= 1; i<numElements; i++)
    {
        // load keys
        kType currentKey  = keys[i];
        kType previousKey = keys[i-1];

        // load value
        voType currentValue = values[vi];
        voType previousValue = values_output[vo];

        previousValue = values_output[vo];
        // within segment
        if (currentKey == previousKey)
        {
            voType r = binary_op( previousValue, currentValue);
            values_output[vo] = r;
            keys_output[ko] = currentKey;

        }
        else // new segment
        {
            vo++;
            ko++;
            values_output[vo] = currentValue;
            keys_output[ko] = currentKey;
            count++; //To count the number of elements in the output array
        }
        vi++;
    }

    return count;

}

template<typename InputIterator1,
         typename InputIterator2,
         typename OutputIterator>

void Serial_scatter (InputIterator1 first1,
                     InputIterator1 last1,
                     InputIterator2 map,
                     OutputIterator result)
{
       size_t numElements = static_cast<  size_t >( std::distance( first1, last1 ) );

	   for (int iter = 0; iter<(int)numElements; iter++)
                *(result+*(map + iter)) = *(first1 + iter);
}


template<typename InputIterator1,
         typename InputIterator2,
		 typename InputIterator3,
         typename OutputIterator,
         typename Predicate>

void Serial_scatter_if (InputIterator1 first1,
                     InputIterator1 last1,
                     InputIterator2 map,
					 InputIterator3 stencil,
                     OutputIterator result,
					 Predicate pred)
{
       size_t numElements = static_cast< size_t >( std::distance( first1, last1 ) );
	   for (int iter = 0; iter< (int)numElements; iter++)
       {
             if(pred(stencil[iter]) != 0)
                  result[*(map+(iter))] = first1[iter];
       }
}

template<typename InputIterator1,
         typename InputIterator2,
         typename OutputIterator>

void Serial_gather (InputIterator1 map_first,
                     InputIterator1 map_last,
                     InputIterator2 input,
                     OutputIterator result)
{
       int numElements = static_cast< int >( std::distance( map_first, map_last ) );
       typedef typename  std::iterator_traits<InputIterator1>::value_type iType1;
       iType1 temp;
       for(int iter = 0; iter < numElements; iter++)
       {
              temp = *(map_first + (int)iter);
              *(result + (int)iter) = *(input + (int)temp);
       }
}

template<typename InputIterator1,
         typename InputIterator2,
		 typename InputIterator3,
         typename OutputIterator,
         typename Predicate>

void Serial_gather_if (InputIterator1 map_first,
                     InputIterator1 map_last,
                     InputIterator2 stencil,
					 InputIterator3 input,
                     OutputIterator result,
					 Predicate pred)
{
    unsigned int numElements = static_cast< unsigned int >( std::distance( map_first, map_last ) );
    for(unsigned int iter = 0; iter < numElements; iter++)
    {
        if(pred(*(stencil + (int)iter)))
             result[(int)iter] = input[map_first[(int)iter]];
    }
}

TEST( TransformIterator, FirstTest)
{
    {
        const int length = 1<<10;
        std::vector< int > svInVec( length );
        std::vector< int > svOutVec( length );
        bolt::BCKND::device_vector< int > dvInVec( length );
        bolt::BCKND::device_vector< int > dvOutVec( length );

        square sq;
        gen_input gen;
        typedef std::vector< int >::const_iterator                                                         sv_itr;
        typedef bolt::BCKND::device_vector< int >::iterator                                                dv_itr;
        typedef bolt::BCKND::transform_iterator< square, std::vector< int >::const_iterator>               sv_trf_itr;
        typedef bolt::BCKND::transform_iterator< square, bolt::BCKND::device_vector< int >::iterator>      dv_trf_itr;
    
        /*Create Iterators*/
        sv_trf_itr sv_trf_begin (svInVec.begin(), sq), sv_trf_end (svInVec.end(), sq);
        dv_trf_itr dv_trf_begin (dvInVec.begin(), sq), dv_trf_end (dvInVec.end(), sq);
    
        /*Generate inputs*/
        std::generate(svInVec.begin(), svInVec.end(), gen);    
        bolt::BCKND::generate(dvInVec.begin(), dvInVec.end(), gen);

        sv_trf_itr::difference_type dist1 = bolt::cl::distance(sv_trf_begin, sv_trf_end);
        sv_trf_itr::difference_type dist2 = bolt::cl::distance(dv_trf_begin, dv_trf_end );

        EXPECT_EQ( dist1, dist2 );
        //std::cout << "distance = " << dist1 << "\n" ;

        for(int i =0; i< length; i++)
        {
            int temp1, temp2;
            temp1 = *sv_trf_begin++;
            temp2 = *dv_trf_begin++;
            EXPECT_EQ( temp1, temp2 );
            //std::cout << temp1 << "   " << temp2 << "\n";
        }
        global_id = 0; // Reset the global id counter
    }
}


TEST( TransformIterator, UDDTest)
{
    {
        const int length = 1<<10;
        std::vector< UDD > svInVec( length );
        std::vector< UDD > svOutVec( length );
        bolt::BCKND::device_vector< UDD > dvInVec( length );
        bolt::BCKND::device_vector< UDD > dvOutVec( length );

        squareUDD_result_float sqUDD;
        gen_input_udd genUDD;
        /*Type defintions*/
        typedef std::vector< UDD >::const_iterator                                                          sv_itr;
        typedef bolt::BCKND::device_vector< UDD >::iterator                                                 dv_itr;
        typedef bolt::BCKND::transform_iterator< squareUDD_result_float, std::vector< UDD >::const_iterator>             sv_trf_itr;
        typedef bolt::BCKND::transform_iterator< squareUDD_result_float, bolt::BCKND::device_vector< UDD >::iterator>    dv_trf_itr;
    
        /*Create Iterators*/
        sv_trf_itr sv_trf_begin (svInVec.begin(), sqUDD), sv_trf_end (svInVec.begin(), sqUDD);

        dv_trf_itr dv_trf_begin (dvInVec.begin(), sqUDD), dv_trf_end (dvInVec.begin(), sqUDD);
 
        /*Generate inputs*/
        std::generate(svInVec.begin(), svInVec.end(), genUDD);
        bolt::BCKND::generate(dvInVec.begin(), dvInVec.end(), genUDD);

        int dist1 = static_cast< int >(std::distance(sv_trf_begin, sv_trf_end));
        int dist2 = static_cast< int >(std::distance(dv_trf_begin, dv_trf_end));

        EXPECT_EQ( dist1, dist2 );
        //std::cout << "distance = " << dist1 << "\n" ;

        for(int i =0; i< length; i++)
        {
            float temp1, temp2; //Return type of the unary function is a float
            temp1 = (float)*sv_trf_begin++;
            temp2 = (float)*dv_trf_begin++;
            EXPECT_FLOAT_EQ( temp1, temp2 );
            //std::cout << temp1 << "   " << temp2 << "\n";
        }
        global_id = 0; // Reset the global id counter
    }
    
}


TEST( TransformIterator, UnaryTransformRoutine)
{
    {
        const int length = 1<<10;
        std::vector< int > svIn1Vec( length );
        std::vector< int > svOutVec( length );
        std::vector< int > stlOut( length );
        bolt::BCKND::device_vector< int > dvIn1Vec( length );
        bolt::BCKND::device_vector< int > dvOutVec( length );

        add_3 add3;
        gen_input gen;
        typedef std::vector< int >::const_iterator                                                     sv_itr;
        typedef bolt::BCKND::device_vector< int >::iterator                                            dv_itr;
        typedef bolt::BCKND::counting_iterator< int >                                                  counting_itr;
        typedef bolt::BCKND::constant_iterator< int >                                                  constant_itr;
        typedef bolt::BCKND::transform_iterator< add_3, std::vector< int >::const_iterator>            sv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_3, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_4, std::vector< int >::const_iterator>            sv_trf_itr_add4;
        typedef bolt::BCKND::transform_iterator< add_4, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add4;    
        /*Create Iterators*/
        sv_trf_itr_add3 sv_trf_begin1 (svIn1Vec.begin(), add3), sv_trf_end1 (svIn1Vec.end(), add3);

        dv_trf_itr_add3 dv_trf_begin1 (dvIn1Vec.begin(), add3), dv_trf_end1 (dvIn1Vec.end(), add3);

        counting_itr count_itr_begin(0);
        counting_itr count_itr_end = count_itr_begin + length;
        constant_itr const_itr_begin(1);
        constant_itr const_itr_end = const_itr_begin + length;

        /*Generate inputs*/
        global_id = 0;
        std::generate(svIn1Vec.begin(), svIn1Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn1Vec.begin(), dvIn1Vec.end(), gen);
        global_id = 0;
        {/*Test case when inputs are trf Iterators*/
            bolt::cl::transform(sv_trf_begin1, sv_trf_end1, svOutVec.begin(), add3);
            bolt::cl::transform(dv_trf_begin1, dv_trf_end1, dvOutVec.begin(), add3);
            /*Compute expected results*/
            std::transform(sv_trf_begin1, sv_trf_end1, stlOut.begin(), add3);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the both are randomAccessIterator */
            bolt::cl::transform(svIn1Vec.begin(), svIn1Vec.end(), svOutVec.begin(), add3);
            bolt::cl::transform(dvIn1Vec.begin(), dvIn1Vec.end(), dvOutVec.begin(), add3);
            /*Compute expected results*/
            std::transform(svIn1Vec.begin(), svIn1Vec.end(), stlOut.begin(), add3);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the first input is constant iterator and the second is a counting iterator */
            bolt::cl::transform(const_itr_begin, const_itr_end, svOutVec.begin(), add3);
            bolt::cl::transform(const_itr_begin, const_itr_end, dvOutVec.begin(), add3);
            /*Compute expected results*/
            std::vector<int> const_vector(length,1);
            std::transform(const_vector.begin(), const_vector.end(), stlOut.begin(), add3);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the first input is constant iterator and the second is a counting iterator */
            bolt::cl::transform(count_itr_begin, count_itr_end, svOutVec.begin(), add3);
            bolt::cl::transform(count_itr_begin, count_itr_end, dvOutVec.begin(), add3);
            /*Compute expected results*/
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;
            std::transform(count_vector.begin(), count_vector.end(), stlOut.begin(), add3);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        global_id = 0; // Reset the global id counter
    }
}


TEST( TransformIterator, UnaryTransformUDDRoutine)
{
    {
        const int length = 1<<10;
        std::vector< UDD > svIn1Vec( length );
        std::vector< UDD > svOutVec( length );
        std::vector< UDD > stlOut( length );

        bolt::BCKND::device_vector< UDD > dvIn1Vec( length );
        bolt::BCKND::device_vector< UDD > dvOutVec( length );

		std::vector< float > stlOut_float( length );
		std::vector< float > svOutVec_float( length );
		bolt::BCKND::device_vector< float > dvOutVec_float( length );

        squareUDD_resultUDD sqUDD;
		squareUDD_result_float sqUDD_float;

        gen_input_udd genUDD;

        typedef std::vector< UDD>::const_iterator                                                     sv_itr;
        typedef bolt::BCKND::device_vector< UDD >::iterator                                            dv_itr;
        typedef bolt::BCKND::counting_iterator< int >                                                  counting_itr;
        typedef bolt::BCKND::constant_iterator< UDD >                                                  constant_itr;
        typedef bolt::BCKND::transform_iterator< squareUDD_resultUDD, std::vector< UDD >::const_iterator>            sv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< squareUDD_resultUDD, bolt::BCKND::device_vector< UDD >::iterator>   dv_trf_itr_add3;
     
        /*Create Iterators*/
        sv_trf_itr_add3 sv_trf_begin1 (svIn1Vec.begin(), sqUDD), sv_trf_end1 (svIn1Vec.end(), sqUDD);

        dv_trf_itr_add3 dv_trf_begin1 (dvIn1Vec.begin(), sqUDD), dv_trf_end1 (dvIn1Vec.end(), sqUDD);

		UDD temp;
		temp.i=1, temp.f=2.5f;

        counting_itr count_itr_begin(0);
        counting_itr count_itr_end = count_itr_begin + length;
        constant_itr const_itr_begin(temp);
        constant_itr const_itr_end = const_itr_begin + length;

        /*Generate inputs*/
        global_id = 0;
        std::generate(svIn1Vec.begin(), svIn1Vec.end(), genUDD);
        global_id = 0;
        bolt::BCKND::generate(dvIn1Vec.begin(), dvIn1Vec.end(), genUDD);
        global_id = 0;
        {/*Test case when input is trf Iterator*/
            bolt::cl::transform(sv_trf_begin1, sv_trf_end1, svOutVec.begin(), sqUDD);
            bolt::cl::transform(dv_trf_begin1, dv_trf_end1, dvOutVec.begin(), sqUDD);
            /*Compute expected results*/
            std::transform(sv_trf_begin1, sv_trf_end1, stlOut.begin(), sqUDD);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

		 {/*Test case when input is trf Iterator and Output is float vector*/
            bolt::cl::transform(sv_trf_begin1, sv_trf_end1, svOutVec_float.begin(), sqUDD_float);
            bolt::cl::transform(dv_trf_begin1, dv_trf_end1, dvOutVec_float.begin(), sqUDD_float);
            /*Compute expected results*/
            std::transform(sv_trf_begin1, sv_trf_end1, stlOut_float.begin(), sqUDD_float);
            /*Check the results*/
            cmpArrays(svOutVec_float, stlOut_float, length);
            cmpArrays(dvOutVec_float, stlOut_float, length);
        }


        {/*Test case when the input is randomAccessIterator */
            bolt::cl::transform(svIn1Vec.begin(), svIn1Vec.end(), svOutVec.begin(), sqUDD);
            bolt::cl::transform(dvIn1Vec.begin(), dvIn1Vec.end(), dvOutVec.begin(), sqUDD);
            /*Compute expected results*/
            std::transform(svIn1Vec.begin(), svIn1Vec.end(), stlOut.begin(), sqUDD);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the input is constant iterator  */
            bolt::cl::transform(const_itr_begin, const_itr_end, svOutVec.begin(), sqUDD);
            bolt::cl::transform(const_itr_begin, const_itr_end, dvOutVec.begin(), sqUDD);
            /*Compute expected results*/
            std::vector<UDD> const_vector(length,temp);
            std::transform(const_vector.begin(), const_vector.end(), stlOut.begin(), sqUDD);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the input is a counting iterator */
            bolt::cl::transform(count_itr_begin, count_itr_end, svOutVec.begin(), sqUDD);
            bolt::cl::transform(count_itr_begin, count_itr_end, dvOutVec.begin(), sqUDD);
            /*Compute expected results*/
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;
            std::transform(count_vector.begin(), count_vector.end(), stlOut.begin(), sqUDD);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        global_id = 0; // Reset the global id counter
    }
}


TEST( TransformIterator, BinaryTransformRoutine)
{
    {
        const int length = 1<<10;
        std::vector< int > svIn1Vec( length );
        std::vector< int > svIn2Vec( length );
        std::vector< int > svOutVec( length );
        std::vector< int > stlOut( length );
        bolt::BCKND::device_vector< int > dvIn1Vec( length );
        bolt::BCKND::device_vector< int > dvIn2Vec( length );
        bolt::BCKND::device_vector< int > dvOutVec( length );

        add_3 add3;
        add_4 add4;
        bolt::cl::plus<int> plus;
        gen_input gen;
        typedef std::vector< int >::const_iterator                                                     sv_itr;
        typedef bolt::BCKND::device_vector< int >::iterator                                            dv_itr;
        typedef bolt::BCKND::counting_iterator< int >                                                  counting_itr;
        typedef bolt::BCKND::constant_iterator< int >                                                  constant_itr;
        typedef bolt::BCKND::transform_iterator< add_3, std::vector< int >::const_iterator>            sv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_3, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_4, std::vector< int >::const_iterator>            sv_trf_itr_add4;
        typedef bolt::BCKND::transform_iterator< add_4, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add4;    
        /*Create Iterators*/
        sv_trf_itr_add3 sv_trf_begin1 (svIn1Vec.begin(), add3), sv_trf_end1 (svIn1Vec.end(), add3);
        sv_trf_itr_add4 sv_trf_begin2 (svIn2Vec.begin(), add4);
        dv_trf_itr_add3 dv_trf_begin1 (dvIn1Vec.begin(), add3), dv_trf_end1 (dvIn1Vec.end(), add3);
        dv_trf_itr_add4 dv_trf_begin2 (dvIn2Vec.begin(), add4);
        counting_itr count_itr_begin(0);
        counting_itr count_itr_end = count_itr_begin + length;
        constant_itr const_itr_begin(1);
        constant_itr const_itr_end = const_itr_begin + length;

        /*Generate inputs*/
        global_id = 0;
        std::generate(svIn1Vec.begin(), svIn1Vec.end(), gen);
        global_id = 0;
        std::generate(svIn2Vec.begin(), svIn2Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn1Vec.begin(), dvIn1Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn2Vec.begin(), dvIn2Vec.end(), gen);
        global_id = 0;
        {/*Test case when both inputs are trf Iterators*/
            bolt::cl::transform(sv_trf_begin1, sv_trf_end1, sv_trf_begin2, svOutVec.begin(), plus);
            bolt::cl::transform(dv_trf_begin1, dv_trf_end1, dv_trf_begin2, dvOutVec.begin(), plus);
            /*Compute expected results*/
            std::transform(sv_trf_begin1, sv_trf_end1, sv_trf_begin2, stlOut.begin(), plus);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the first input is trf_itr and the second is a randomAccessIterator */
            bolt::cl::transform(sv_trf_begin1, sv_trf_end1, svIn2Vec.begin(), svOutVec.begin(), plus);
            bolt::cl::transform(dv_trf_begin1, dv_trf_end1, dvIn2Vec.begin(), dvOutVec.begin(), plus);
            /*Compute expected results*/
            std::transform(sv_trf_begin1, sv_trf_end1, svIn2Vec.begin(), stlOut.begin(), plus);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the second input is trf_itr and the first is a randomAccessIterator */
            bolt::cl::transform(svIn1Vec.begin(), svIn1Vec.end(), sv_trf_begin2, svOutVec.begin(), plus);
            bolt::cl::transform(dvIn1Vec.begin(), dvIn1Vec.end(), dv_trf_begin2, dvOutVec.begin(), plus);
            /*Compute expected results*/
            std::transform(svIn1Vec.begin(), svIn1Vec.end(), sv_trf_begin2, stlOut.begin(), plus);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the both are randomAccessIterator */
            bolt::cl::transform(svIn1Vec.begin(), svIn1Vec.end(), svIn2Vec.begin(), svOutVec.begin(), plus);
            bolt::cl::transform(dvIn1Vec.begin(), dvIn1Vec.end(), dvIn2Vec.begin(), dvOutVec.begin(), plus);
            /*Compute expected results*/
            std::transform(svIn1Vec.begin(), svIn1Vec.end(), svIn2Vec.begin(), stlOut.begin(), plus);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the first input is trf_itr and the second is a constant iterator */
            bolt::cl::transform(sv_trf_begin1, sv_trf_end1, const_itr_begin, svOutVec.begin(), plus);
            bolt::cl::transform(dv_trf_begin1, dv_trf_end1, const_itr_begin, dvOutVec.begin(), plus);
            /*Compute expected results*/
            std::vector<int> const_vector(length,1);
            std::transform(sv_trf_begin1, sv_trf_end1, const_vector.begin(), stlOut.begin(), plus);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the first input is trf_itr and the second is a counting iterator */
            bolt::cl::transform(sv_trf_begin1, sv_trf_end1, count_itr_begin, svOutVec.begin(), plus);
            bolt::cl::transform(dv_trf_begin1, dv_trf_end1, count_itr_begin, dvOutVec.begin(), plus);
            /*Compute expected results*/
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;
            std::transform(sv_trf_begin1, sv_trf_end1, count_vector.begin(), stlOut.begin(), plus);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the first input is constant iterator and the second is a counting iterator */
            bolt::cl::transform(const_itr_begin, const_itr_end, count_itr_begin, svOutVec.begin(), plus);
            bolt::cl::transform(const_itr_begin, const_itr_end, count_itr_begin, dvOutVec.begin(), plus);
            /*Compute expected results*/
           std::vector<int> const_vector(length,1);
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;            
            std::transform(const_vector.begin(), const_vector.end(), count_vector.begin(), stlOut.begin(), plus);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        global_id = 0; // Reset the global id counter
    }
}


TEST( TransformIterator, BinaryTransformUDDRoutine)
{
    {
        const int length = 1<<10;
        std::vector< UDD > svIn1Vec( length );
        std::vector< UDD > svIn2Vec( length );
        std::vector< UDD > svOutVec( length );
        std::vector< UDD > stlOut( length );
        bolt::BCKND::device_vector< UDD > dvIn1Vec( length );
        bolt::BCKND::device_vector< UDD > dvIn2Vec( length );
        bolt::BCKND::device_vector< UDD > dvOutVec( length );


		std::vector< float > stlOut_float( length );
		std::vector< float > svOutVec_float( length );
		bolt::BCKND::device_vector< float > dvOutVec_float( length );


        bolt::cl::plus<UDD> plus;

		cubeUDD_resultUDD cbUDD;
        gen_input_udd genUDD;
		squareUDD_resultUDD sqUDD;

		typedef std::vector< UDD>::const_iterator                                                     sv_itr;
        typedef bolt::BCKND::device_vector< UDD >::iterator                                            dv_itr;
        typedef bolt::BCKND::counting_iterator< int >                                                  counting_itr;
        typedef bolt::BCKND::constant_iterator< UDD >                                                  constant_itr;
        typedef bolt::BCKND::transform_iterator< squareUDD_resultUDD, std::vector< UDD >::const_iterator>            sv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< squareUDD_resultUDD, bolt::BCKND::device_vector< UDD >::iterator>   dv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< cubeUDD_resultUDD, std::vector< UDD >::const_iterator>            sv_trf_itr_add4;
        typedef bolt::BCKND::transform_iterator< cubeUDD_resultUDD, bolt::BCKND::device_vector< UDD >::iterator>   dv_trf_itr_add4;    
        /*Create Iterators*/
        sv_trf_itr_add3 sv_trf_begin1 (svIn1Vec.begin(), sqUDD), sv_trf_end1 (svIn1Vec.end(), sqUDD);
		sv_trf_itr_add4 sv_trf_begin2 (svIn2Vec.begin(), cbUDD);
        dv_trf_itr_add3 dv_trf_begin1 (dvIn1Vec.begin(), sqUDD), dv_trf_end1 (dvIn1Vec.end(), sqUDD);
		dv_trf_itr_add4 dv_trf_begin2 (dvIn2Vec.begin(), cbUDD);

		UDD temp;
		temp.i=1, temp.f=2.5f;

        counting_itr count_itr_begin(0);
        counting_itr count_itr_end = count_itr_begin + length;
        constant_itr const_itr_begin(temp);
        constant_itr const_itr_end = const_itr_begin + length;

        /*Generate inputs*/
        global_id = 0;
        std::generate(svIn1Vec.begin(), svIn1Vec.end(), genUDD);
        global_id = 0;
        std::generate(svIn2Vec.begin(), svIn2Vec.end(), genUDD);
        global_id = 0;
        bolt::BCKND::generate(dvIn1Vec.begin(), dvIn1Vec.end(), genUDD);
        global_id = 0;
        bolt::BCKND::generate(dvIn2Vec.begin(), dvIn2Vec.end(), genUDD);
        global_id = 0;
        {/*Test case when both inputs are trf Iterators*/
            bolt::cl::transform(sv_trf_begin1, sv_trf_end1, sv_trf_begin2, svOutVec.begin(), plus);
            bolt::cl::transform(dv_trf_begin1, dv_trf_end1, dv_trf_begin2, dvOutVec.begin(), plus);
            /*Compute expected results*/
            std::transform(sv_trf_begin1, sv_trf_end1, sv_trf_begin2, stlOut.begin(), plus);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the first input is trf_itr and the second is a randomAccessIterator */
            bolt::cl::transform(sv_trf_begin1, sv_trf_end1, svIn2Vec.begin(), svOutVec.begin(), plus);
            bolt::cl::transform(dv_trf_begin1, dv_trf_end1, dvIn2Vec.begin(), dvOutVec.begin(), plus);
            /*Compute expected results*/
            std::transform(sv_trf_begin1, sv_trf_end1, svIn2Vec.begin(), stlOut.begin(), plus);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the second input is trf_itr and the first is a randomAccessIterator */
            bolt::cl::transform(svIn1Vec.begin(), svIn1Vec.end(), sv_trf_begin2, svOutVec.begin(), plus);
            bolt::cl::transform(dvIn1Vec.begin(), dvIn1Vec.end(), dv_trf_begin2, dvOutVec.begin(), plus);
            /*Compute expected results*/
            std::transform(svIn1Vec.begin(), svIn1Vec.end(), sv_trf_begin2, stlOut.begin(), plus);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the both are randomAccessIterator */
            bolt::cl::transform(svIn1Vec.begin(), svIn1Vec.end(), svIn2Vec.begin(), svOutVec.begin(), plus);
            bolt::cl::transform(dvIn1Vec.begin(), dvIn1Vec.end(), dvIn2Vec.begin(), dvOutVec.begin(), plus);
            /*Compute expected results*/
            std::transform(svIn1Vec.begin(), svIn1Vec.end(), svIn2Vec.begin(), stlOut.begin(), plus);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the first input is trf_itr and the second is a constant iterator */
            bolt::cl::transform(sv_trf_begin1, sv_trf_end1, const_itr_begin, svOutVec.begin(), plus);
            bolt::cl::transform(dv_trf_begin1, dv_trf_end1, const_itr_begin, dvOutVec.begin(), plus);
            /*Compute expected results*/
            std::vector<int> const_vector(length,1);
            std::transform(sv_trf_begin1, sv_trf_end1, const_vector.begin(), stlOut.begin(), plus);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the first input is trf_itr and the second is a counting iterator */
            bolt::cl::transform(sv_trf_begin1, sv_trf_end1, count_itr_begin, svOutVec.begin(), plus);
            bolt::cl::transform(dv_trf_begin1, dv_trf_end1, count_itr_begin, dvOutVec.begin(), plus);
            /*Compute expected results*/
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;
            std::transform(sv_trf_begin1, sv_trf_end1, count_vector.begin(), stlOut.begin(), plus);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the first input is constant iterator and the second is a counting iterator */
            bolt::cl::transform(const_itr_begin, const_itr_end, count_itr_begin, svOutVec.begin(), plus);
            bolt::cl::transform(const_itr_begin, const_itr_end, count_itr_begin, dvOutVec.begin(), plus);
            /*Compute expected results*/
           std::vector<int> const_vector(length,1);
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;            
            std::transform(const_vector.begin(), const_vector.end(), count_vector.begin(), stlOut.begin(), plus);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        global_id = 0; // Reset the global id counter
    }
}


TEST( TransformIterator, InclusiveTransformScanRoutine)
{
    {
        const int length = 1<<10;
        std::vector< int > svIn1Vec( length );
        std::vector< int > svOutVec( length );
        std::vector< int > stlOut( length );
        bolt::BCKND::device_vector< int > dvIn1Vec( length );
        bolt::BCKND::device_vector< int > dvOutVec( length );

        add_3 add3;

        bolt::cl::negate<int> nI2;
        bolt::cl::plus<int> addI2;


        gen_input gen;
        typedef std::vector< int >::const_iterator                                                   sv_itr;
        typedef bolt::BCKND::device_vector< int >::iterator                                          dv_itr;
        typedef bolt::BCKND::counting_iterator< int >                                                counting_itr;
        typedef bolt::BCKND::constant_iterator< int >                                                constant_itr;
        typedef bolt::BCKND::transform_iterator< add_3, std::vector< int >::const_iterator>          sv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_3, bolt::BCKND::device_vector< int >::iterator> dv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_4, std::vector< int >::const_iterator>          sv_trf_itr_add4;
        typedef bolt::BCKND::transform_iterator< add_4, bolt::BCKND::device_vector< int >::iterator> dv_trf_itr_add4;    
        /*Create Iterators*/
        sv_trf_itr_add3 sv_trf_begin1 (svIn1Vec.begin(), add3), sv_trf_end1 (svIn1Vec.end(), add3);

        dv_trf_itr_add3 dv_trf_begin1 (dvIn1Vec.begin(), add3), dv_trf_end1 (dvIn1Vec.end(), add3);

        counting_itr count_itr_begin(0);
        counting_itr count_itr_end = count_itr_begin + length;
        constant_itr const_itr_begin(1);
        constant_itr const_itr_end = const_itr_begin + length;

        /*Generate inputs*/
        global_id = 0;
        std::generate(svIn1Vec.begin(), svIn1Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn1Vec.begin(), dvIn1Vec.end(), gen);
        global_id = 0;
        {/*Test case when inputs are trf Iterators*/
            bolt::cl::transform_inclusive_scan(sv_trf_begin1, sv_trf_end1, svOutVec.begin(), nI2, addI2);
            bolt::cl::transform_inclusive_scan(dv_trf_begin1, dv_trf_end1, dvOutVec.begin(), nI2, addI2);
            /*Compute expected results*/
            std::transform(sv_trf_begin1, sv_trf_end1, stlOut.begin(), nI2);
            std::partial_sum(stlOut.begin(), stlOut.end(), stlOut.begin(), addI2);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the both are randomAccessIterator */
            bolt::cl::transform_inclusive_scan(svIn1Vec.begin(), svIn1Vec.end(), svOutVec.begin(), nI2, addI2);
            bolt::cl::transform_inclusive_scan(dvIn1Vec.begin(), dvIn1Vec.end(), dvOutVec.begin(), nI2, addI2);
            /*Compute expected results*/
            std::transform(svIn1Vec.begin(), svIn1Vec.end(), stlOut.begin(), nI2);
            std::partial_sum(stlOut.begin(), stlOut.end(), stlOut.begin(), addI2);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the first input is constant iterator and the second is a counting iterator */
            bolt::cl::transform_inclusive_scan(const_itr_begin, const_itr_end, svOutVec.begin(), nI2, addI2);
            bolt::cl::transform_inclusive_scan(const_itr_begin, const_itr_end, dvOutVec.begin(), nI2, addI2);
            /*Compute expected results*/
            std::vector<int> const_vector(length,1);
            std::transform(const_vector.begin(), const_vector.end(), stlOut.begin(), nI2);
            std::partial_sum(stlOut.begin(), stlOut.end(), stlOut.begin(), addI2);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the first input is constant iterator and the second is a counting iterator */
            bolt::cl::transform_inclusive_scan(count_itr_begin, count_itr_end, svOutVec.begin(), nI2, addI2);
            bolt::cl::transform_inclusive_scan(count_itr_begin, count_itr_end, dvOutVec.begin(), nI2, addI2);
            /*Compute expected results*/
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;
            std::transform(count_vector.begin(), count_vector.end(), stlOut.begin(), nI2);
            std::partial_sum(stlOut.begin(), stlOut.end(), stlOut.begin(), addI2);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        global_id = 0; // Reset the global id counter
    }
}

TEST( TransformIterator, InclusiveTransformScanUDDRoutine)
{
    {
        const int length = 1<<10;
        std::vector< UDD > svIn1Vec( length );
        std::vector< UDD > svOutVec( length );
        std::vector< UDD > stlOut( length );
        bolt::BCKND::device_vector< UDD > dvIn1Vec( length );
        bolt::BCKND::device_vector< UDD > dvOutVec( length );

       // bolt::cl::negate<UDD> nI2;
        bolt::cl::plus<UDD> addI2;

		squareUDD_resultUDD sqUDD;
        gen_input_udd genUDD;

        typedef std::vector< UDD >::const_iterator                                                   sv_itr;
        typedef bolt::BCKND::device_vector< UDD >::iterator                                          dv_itr;
        typedef bolt::BCKND::counting_iterator< int >                                                counting_itr;
        typedef bolt::BCKND::constant_iterator< UDD >                                                constant_itr;
        typedef bolt::BCKND::transform_iterator< squareUDD_resultUDD, std::vector< UDD >::const_iterator>          sv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< squareUDD_resultUDD, bolt::BCKND::device_vector< UDD >::iterator> dv_trf_itr_add3;
       
        /*Create Iterators*/
        sv_trf_itr_add3 sv_trf_begin1 (svIn1Vec.begin(), sqUDD), sv_trf_end1 (svIn1Vec.end(), sqUDD);

        dv_trf_itr_add3 dv_trf_begin1 (dvIn1Vec.begin(), sqUDD), dv_trf_end1 (dvIn1Vec.end(), sqUDD);

        UDD temp;
		temp.i=1, temp.f=2.5f;

        counting_itr count_itr_begin(0);
        counting_itr count_itr_end = count_itr_begin + length;
        constant_itr const_itr_begin(temp);
        constant_itr const_itr_end = const_itr_begin + length;

        /*Generate inputs*/
        global_id = 0;
        std::generate(svIn1Vec.begin(), svIn1Vec.end(), genUDD);
        global_id = 0;
        bolt::BCKND::generate(dvIn1Vec.begin(), dvIn1Vec.end(), genUDD);
        global_id = 0;
        {/*Test case when input is trf Iterator*/
            bolt::cl::transform_inclusive_scan(sv_trf_begin1, sv_trf_end1, svOutVec.begin(), sqUDD /*nI2*/, addI2);
            bolt::cl::transform_inclusive_scan(dv_trf_begin1, dv_trf_end1, dvOutVec.begin(), sqUDD /*nI2*/, addI2);
            /*Compute expected results*/
            std::transform(sv_trf_begin1, sv_trf_end1, stlOut.begin(), sqUDD /*nI2*/);
            std::partial_sum(stlOut.begin(), stlOut.end(), stlOut.begin(), addI2);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        //{/*Test case when the input is randomAccessIterator */
        //    bolt::cl::transform_inclusive_scan(svIn1Vec.begin(), svIn1Vec.end(), svOutVec.begin(), nI2, addI2);
        //    bolt::cl::transform_inclusive_scan(dvIn1Vec.begin(), dvIn1Vec.end(), dvOutVec.begin(), nI2, addI2);
        //    /*Compute expected results*/
        //    std::transform(svIn1Vec.begin(), svIn1Vec.end(), stlOut.begin(), nI2);
        //    std::partial_sum(stlOut.begin(), stlOut.end(), stlOut.begin(), addI2);
        //    /*Check the results*/
        //    cmpArrays(svOutVec, stlOut, length);
        //    cmpArrays(dvOutVec, stlOut, length);
        //}
        //{/*Test case when the input is constant iterator */
        //    bolt::cl::transform_inclusive_scan(const_itr_begin, const_itr_end, svOutVec.begin(), nI2, addI2);
        //    bolt::cl::transform_inclusive_scan(const_itr_begin, const_itr_end, dvOutVec.begin(), nI2, addI2);
        //    /*Compute expected results*/
        //    std::vector<int> const_vector(length,1);
        //    std::transform(const_vector.begin(), const_vector.end(), stlOut.begin(), nI2);
        //    std::partial_sum(stlOut.begin(), stlOut.end(), stlOut.begin(), addI2);
        //    /*Check the results*/
        //    cmpArrays(svOutVec, stlOut, length);
        //    cmpArrays(dvOutVec, stlOut, length);
        //}
        //{/*Test case when the input is a counting iterator */
        //    bolt::cl::transform_inclusive_scan(count_itr_begin, count_itr_end, svOutVec.begin(), sqUDD /*nI2*/, addI2);
        //    bolt::cl::transform_inclusive_scan(count_itr_begin, count_itr_end, dvOutVec.begin(), sqUDD /*nI2*/, addI2);
        //    /*Compute expected results*/
        //    std::vector<int> count_vector(length);
        //    for (int index=0;index<length;index++)
        //        count_vector[index] = index;
        //    std::transform(count_vector.begin(), count_vector.end(), stlOut.begin(), sqUDD /*nI2*/);
        //    std::partial_sum(stlOut.begin(), stlOut.end(), stlOut.begin(), addI2);
        //    /*Check the results*/
        //    cmpArrays(svOutVec, stlOut, length);
        //    cmpArrays(dvOutVec, stlOut, length);
        //}
        global_id = 0; // Reset the global id counter
    }
}


TEST( TransformIterator, ExclusiveTransformScanRoutine)
{
    {
        const int length = 1<<10;
        std::vector< int > svIn1Vec( length );
        std::vector< int > svOutVec( length );
        std::vector< int > stlOut( length );
        bolt::BCKND::device_vector< int > dvIn1Vec( length );
        bolt::BCKND::device_vector< int > dvOutVec( length );

        add_3 add3;

        bolt::cl::negate<int> nI2;
        bolt::cl::plus<int> addI2;
        int n = (int) 1 + rand()%10;

        gen_input gen;
        typedef std::vector< int >::const_iterator                                                     sv_itr;
        typedef bolt::BCKND::device_vector< int >::iterator                                            dv_itr;
        typedef bolt::BCKND::counting_iterator< int >                                                  counting_itr;
        typedef bolt::BCKND::constant_iterator< int >                                                  constant_itr;
        typedef bolt::BCKND::transform_iterator< add_3, std::vector< int >::const_iterator>            sv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_3, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_4, std::vector< int >::const_iterator>            sv_trf_itr_add4;
        typedef bolt::BCKND::transform_iterator< add_4, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add4;    
        /*Create Iterators*/
        sv_trf_itr_add3 sv_trf_begin1 (svIn1Vec.begin(), add3), sv_trf_end1 (svIn1Vec.end(), add3);

        dv_trf_itr_add3 dv_trf_begin1 (dvIn1Vec.begin(), add3), dv_trf_end1 (dvIn1Vec.end(), add3);

        counting_itr count_itr_begin(0);
        counting_itr count_itr_end = count_itr_begin + length;
        constant_itr const_itr_begin(1);
        constant_itr const_itr_end = const_itr_begin + length;

        /*Generate inputs*/
        global_id = 0;
        std::generate(svIn1Vec.begin(), svIn1Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn1Vec.begin(), dvIn1Vec.end(), gen);
        global_id = 0;
        {/*Test case when inputs are trf Iterators*/
            bolt::cl::transform_exclusive_scan(sv_trf_begin1, sv_trf_end1, svOutVec.begin(), nI2, n, addI2);
            bolt::cl::transform_exclusive_scan(dv_trf_begin1, dv_trf_end1, dvOutVec.begin(), nI2, n, addI2);
            /*Compute expected results*/
            std::transform(sv_trf_begin1, sv_trf_end1, stlOut.begin(), nI2);
            Serial_scan<int,  bolt::cl::plus< int >, int>(&stlOut[0], &stlOut[0], length, addI2, false, n);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the both are randomAccessIterator */
            bolt::cl::transform_exclusive_scan(svIn1Vec.begin(), svIn1Vec.end(), svOutVec.begin(), nI2, n, addI2);
            bolt::cl::transform_exclusive_scan(dvIn1Vec.begin(), dvIn1Vec.end(), dvOutVec.begin(), nI2, n, addI2);
            /*Compute expected results*/
            std::transform(svIn1Vec.begin(), svIn1Vec.end(), stlOut.begin(), nI2);
            Serial_scan<int,  bolt::cl::plus< int >, int>(&stlOut[0], &stlOut[0], length, addI2, false, n);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the first input is constant iterator and the second is a counting iterator */
            bolt::cl::transform_exclusive_scan(const_itr_begin, const_itr_end, svOutVec.begin(), nI2, n, addI2);
            bolt::cl::transform_exclusive_scan(const_itr_begin, const_itr_end, dvOutVec.begin(), nI2, n, addI2);
            /*Compute expected results*/
            std::vector<int> const_vector(length,1);
            std::transform(const_vector.begin(), const_vector.end(), stlOut.begin(), nI2);
            Serial_scan<int,  bolt::cl::plus< int >, int>(&stlOut[0], &stlOut[0], length, addI2, false, n);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the first input is constant iterator and the second is a counting iterator */
            bolt::cl::transform_exclusive_scan(count_itr_begin, count_itr_end, svOutVec.begin(), nI2, n, addI2);
            bolt::cl::transform_exclusive_scan(count_itr_begin, count_itr_end, dvOutVec.begin(), nI2, n, addI2);
            /*Compute expected results*/
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;
            std::transform(count_vector.begin(), count_vector.end(), stlOut.begin(), nI2);
            Serial_scan<int,  bolt::cl::plus< int >, int>(&stlOut[0], &stlOut[0], length, addI2, false, n);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        global_id = 0; // Reset the global id counter
    }
}

TEST( TransformIterator, ExclusiveTransformScanUDDRoutine)
{
    {
        const int length = 1<<10;
        std::vector< UDD > svIn1Vec( length );
        std::vector< UDD > svOutVec( length );
        std::vector< UDD > stlOut( length );
        bolt::BCKND::device_vector< UDD > dvIn1Vec( length );
        bolt::BCKND::device_vector< UDD > dvOutVec( length );

		bolt::cl::negate<UDD> nI2;
        bolt::cl::plus<UDD> addI2;

		squareUDD_resultUDD sqUDD;
        gen_input_udd genUDD;
		int n = (int) 1 + rand()%10;

        typedef std::vector< UDD >::const_iterator                                                   sv_itr;
        typedef bolt::BCKND::device_vector< UDD >::iterator                                          dv_itr;
        typedef bolt::BCKND::counting_iterator< int >                                                counting_itr;
        typedef bolt::BCKND::constant_iterator< UDD >                                                constant_itr;
        typedef bolt::BCKND::transform_iterator< squareUDD_resultUDD, std::vector< UDD >::const_iterator>          sv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< squareUDD_resultUDD, bolt::BCKND::device_vector< UDD >::iterator> dv_trf_itr_add3;
       
        /*Create Iterators*/
        sv_trf_itr_add3 sv_trf_begin1 (svIn1Vec.begin(), sqUDD), sv_trf_end1 (svIn1Vec.end(), sqUDD);

        dv_trf_itr_add3 dv_trf_begin1 (dvIn1Vec.begin(), sqUDD), dv_trf_end1 (dvIn1Vec.end(), sqUDD);

        UDD temp;
		temp.i=1, temp.f=2.5f;

        counting_itr count_itr_begin(0);
        counting_itr count_itr_end = count_itr_begin + length;
        constant_itr const_itr_begin(temp);
        constant_itr const_itr_end = const_itr_begin + length;

        /*Generate inputs*/
        global_id = 0;
        std::generate(svIn1Vec.begin(), svIn1Vec.end(), genUDD);
        global_id = 0;
        bolt::BCKND::generate(dvIn1Vec.begin(), dvIn1Vec.end(), genUDD);
        global_id = 0;
        {/*Test case when inputs is a trf Iterator*/
            bolt::cl::transform_exclusive_scan(sv_trf_begin1, sv_trf_end1, svOutVec.begin(), nI2, n, addI2);
            bolt::cl::transform_exclusive_scan(dv_trf_begin1, dv_trf_end1, dvOutVec.begin(), nI2, n, addI2);
            /*Compute expected results*/
            std::transform(sv_trf_begin1, sv_trf_end1, stlOut.begin(), nI2);
            Serial_scan<UDD,  bolt::cl::plus< UDD >, UDD>(&stlOut[0], &stlOut[0], length, addI2, false, n);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when input is a randomAccessIterator */
            bolt::cl::transform_exclusive_scan(svIn1Vec.begin(), svIn1Vec.end(), svOutVec.begin(), nI2, n, addI2);
            bolt::cl::transform_exclusive_scan(dvIn1Vec.begin(), dvIn1Vec.end(), dvOutVec.begin(), nI2, n, addI2);
            /*Compute expected results*/
            std::transform(svIn1Vec.begin(), svIn1Vec.end(), stlOut.begin(), nI2);
            Serial_scan<UDD,  bolt::cl::plus< UDD >, UDD>(&stlOut[0], &stlOut[0], length, addI2, false, n);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when first input is a counting iterator */
            bolt::cl::transform_exclusive_scan(const_itr_begin, const_itr_end, svOutVec.begin(), nI2, n, addI2);
            bolt::cl::transform_exclusive_scan(const_itr_begin, const_itr_end, dvOutVec.begin(), nI2, n, addI2);
            /*Compute expected results*/
            std::vector<UDD> const_vector(length,1);
            std::transform(const_vector.begin(), const_vector.end(), stlOut.begin(), nI2);
            Serial_scan<UDD,  bolt::cl::plus< UDD >, UDD>(&stlOut[0], &stlOut[0], length, addI2, false, n);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        //{/*Test case when the input is a constant iterator */
        //    bolt::cl::transform_exclusive_scan(count_itr_begin, count_itr_end, svOutVec.begin(), nI2, n, addI2);
        //    bolt::cl::transform_exclusive_scan(count_itr_begin, count_itr_end, dvOutVec.begin(), nI2, n, addI2);
        //    /*Compute expected results*/
        //    std::vector<UDD> count_vector(length);
        //    for (int index=0;index<length;index++)
        //        count_vector[index] = index;
        //    std::transform(count_vector.begin(), count_vector.end(), stlOut.begin(), nI2);
        //    Serial_scan<UDD,  bolt::cl::plus< UDD >, UDD>(&stlOut[0], &stlOut[0], length, addI2, false, n);
        //    /*Check the results*/
        //    cmpArrays(svOutVec, stlOut, length);
        //    cmpArrays(dvOutVec, stlOut, length);
        //}
        global_id = 0; // Reset the global id counter
    }
}

TEST( TransformIterator, ReduceRoutine)
{
    {
        const int length = 1<<10;
        std::vector< int > svIn1Vec( length );
        std::vector< int > svIn2Vec( length );
        std::vector< int > svOutVec( length );
        std::vector< int > stlOut( length );
        bolt::BCKND::device_vector< int > dvIn1Vec( length );
        bolt::BCKND::device_vector< int > dvIn2Vec( length );
        bolt::BCKND::device_vector< int > dvOutVec( length );

        add_3 add3;
        add_4 add4;
        bolt::cl::plus<int> plus;
        gen_input gen;
        typedef std::vector< int >::const_iterator                                                         sv_itr;
        typedef bolt::BCKND::device_vector< int >::iterator                                                dv_itr;
        typedef bolt::BCKND::counting_iterator< int >                                                      counting_itr;
        typedef bolt::BCKND::constant_iterator< int >                                                      constant_itr;
        typedef bolt::BCKND::transform_iterator< add_3, std::vector< int >::const_iterator>                sv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_3, bolt::BCKND::device_vector< int >::iterator>       dv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_4, std::vector< int >::const_iterator>                sv_trf_itr_add4;
        typedef bolt::BCKND::transform_iterator< add_4, bolt::BCKND::device_vector< int >::iterator>       dv_trf_itr_add4;    
        /*Create Iterators*/
        sv_trf_itr_add3 sv_trf_begin1 (svIn1Vec.begin(), add3), sv_trf_end1 (svIn1Vec.end(), add3);
        sv_trf_itr_add4 sv_trf_begin2 (svIn2Vec.begin(), add4);
        dv_trf_itr_add3 dv_trf_begin1 (dvIn1Vec.begin(), add3), dv_trf_end1 (dvIn1Vec.end(), add3);
        dv_trf_itr_add4 dv_trf_begin2 (dvIn2Vec.begin(), add4);
        counting_itr count_itr_begin(0);
        counting_itr count_itr_end = count_itr_begin + length;
        constant_itr const_itr_begin(1);
        constant_itr const_itr_end = const_itr_begin + length;

        /*Generate inputs*/
        global_id = 0;
        std::generate(svIn1Vec.begin(), svIn1Vec.end(), gen);
        global_id = 0;
        std::generate(svIn2Vec.begin(), svIn2Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn1Vec.begin(), dvIn1Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn2Vec.begin(), dvIn2Vec.end(), gen);
        global_id = 0;
        {/*Test case when inputs are trf Iterators*/
            int sv_result = bolt::cl::reduce(sv_trf_begin1, sv_trf_end1, 0, plus);
            int dv_result = bolt::cl::reduce(dv_trf_begin1, dv_trf_end1, 0, plus);
            /*Compute expected results*/
            int expected_result = std::accumulate(sv_trf_begin1, sv_trf_end1, 0, plus);
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }
        {/*Test case when the first input is trf_itr and the second is a randomAccessIterator */
            int sv_result = bolt::cl::reduce(svIn2Vec.begin(), svIn2Vec.end(), 0, plus);
            int dv_result = bolt::cl::reduce(dvIn2Vec.begin(), dvIn2Vec.end(), 0, plus);
            /*Compute expected results*/
            int expected_result = std::accumulate(svIn2Vec.begin(), svIn2Vec.end(), 0, plus);
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }
        {/*Test case when the first input is trf_itr and the second is a constant iterator */
            int sv_result = bolt::cl::reduce(const_itr_begin, const_itr_end, 0, plus);
            int dv_result = bolt::cl::reduce(const_itr_begin, const_itr_end, 0, plus);
            /*Compute expected results*/
            int expected_result = std::accumulate(const_itr_begin, const_itr_end, 0, plus);
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }
        {/*Test case when the first input is trf_itr and the second is a counting iterator */
            int sv_result = bolt::cl::reduce(count_itr_begin, count_itr_end, 0, plus);
            int dv_result = bolt::cl::reduce(count_itr_begin, count_itr_end, 0, plus);
            /*Compute expected results*/
            int expected_result = std::accumulate(count_itr_begin, count_itr_end, 0, plus);
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }
        global_id = 0; // Reset the global id counter
    }
}


TEST( TransformIterator, ReduceUDDRoutine)
{
    {
        const int length = 1<<10;
        std::vector< UDD > svIn1Vec( length );
        std::vector< UDD > svIn2Vec( length );
        std::vector< UDD > svOutVec( length );
        std::vector< UDD > stlOut( length );
        bolt::BCKND::device_vector< UDD > dvIn1Vec( length );
        bolt::BCKND::device_vector< UDD > dvIn2Vec( length );
        bolt::BCKND::device_vector< UDD > dvOutVec( length );


        UDDplus plus;
		squareUDD_resultUDD sqUDD;
		cubeUDD_resultUDD cbUDD;
        gen_input_udd genUDD;

        typedef std::vector< UDD >::const_iterator                                                   sv_itr;
        typedef bolt::BCKND::device_vector< UDD >::iterator                                          dv_itr;
        typedef bolt::BCKND::counting_iterator< int >                                                counting_itr;
        typedef bolt::BCKND::constant_iterator< UDD >                                                constant_itr;
        typedef bolt::BCKND::transform_iterator< squareUDD_resultUDD, std::vector< UDD >::const_iterator>          sv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< squareUDD_resultUDD, bolt::BCKND::device_vector< UDD >::iterator> dv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< cubeUDD_resultUDD, std::vector< UDD >::const_iterator>                sv_trf_itr_add4;
        typedef bolt::BCKND::transform_iterator< cubeUDD_resultUDD, bolt::BCKND::device_vector< UDD >::iterator>       dv_trf_itr_add4;   


        /*Create Iterators*/
        sv_trf_itr_add3 sv_trf_begin1 (svIn1Vec.begin(), sqUDD), sv_trf_end1 (svIn1Vec.end(), sqUDD);
		sv_trf_itr_add4 sv_trf_begin2 (svIn2Vec.begin(), cbUDD);
        dv_trf_itr_add3 dv_trf_begin1 (dvIn1Vec.begin(), sqUDD), dv_trf_end1 (dvIn1Vec.end(), sqUDD);
		dv_trf_itr_add4 dv_trf_begin2 (dvIn2Vec.begin(), cbUDD);

        UDD temp;
		temp.i=1, temp.f=2.5f;

        counting_itr count_itr_begin(0);
        counting_itr count_itr_end = count_itr_begin + length;
        constant_itr const_itr_begin(temp);
        constant_itr const_itr_end = const_itr_begin + length;

        /*Generate inputs*/
        global_id = 0;
        std::generate(svIn1Vec.begin(), svIn1Vec.end(), genUDD);
		global_id = 0;
        std::generate(svIn2Vec.begin(), svIn2Vec.end(), genUDD);
        global_id = 0;
        bolt::BCKND::generate(dvIn1Vec.begin(), dvIn1Vec.end(), genUDD);
        global_id = 0;
		bolt::BCKND::generate(dvIn2Vec.begin(), dvIn2Vec.end(), genUDD);
        global_id = 0;

		UDD UDDzero;
        UDDzero.i = 0;
        UDDzero.f = 0.0f;


        {/*Test case when input is trf Iterator*/
            UDD sv_result = bolt::cl::reduce(sv_trf_begin1, sv_trf_end1, UDDzero, plus);
            UDD dv_result = bolt::cl::reduce(dv_trf_begin1, dv_trf_end1, UDDzero, plus);
            /*Compute expected results*/
            UDD expected_result = std::accumulate(sv_trf_begin1, sv_trf_end1, UDDzero, plus);
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }
        {/*Test case when input is a randomAccessIterator */
            UDD sv_result = bolt::cl::reduce(svIn2Vec.begin(), svIn2Vec.end(), UDDzero, plus);
            UDD dv_result = bolt::cl::reduce(dvIn2Vec.begin(), dvIn2Vec.end(), UDDzero, plus);
            /*Compute expected results*/
            UDD expected_result = std::accumulate(svIn2Vec.begin(), svIn2Vec.end(), UDDzero, plus);
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }
        {/*Test case wheninput is a constant iterator */
            UDD sv_result = bolt::cl::reduce(const_itr_begin, const_itr_end, UDDzero, plus);
            UDD dv_result = bolt::cl::reduce(const_itr_begin, const_itr_end, UDDzero, plus);
            /*Compute expected results*/
            UDD expected_result = std::accumulate(const_itr_begin, const_itr_end, UDDzero, plus);
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }
        {/*Test case when input is a counting iterator */
            UDD sv_result = bolt::cl::reduce(count_itr_begin, count_itr_end, UDDzero, plus);
            UDD dv_result = bolt::cl::reduce(count_itr_begin, count_itr_end, UDDzero, plus);
            /*Compute expected results*/
            UDD expected_result = std::accumulate(count_itr_begin, count_itr_end, UDDzero, plus);
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }
        global_id = 0; // Reset the global id counter
    }
}


TEST( TransformIterator, TransformReduceRoutine)
{
    {
        const int length = 1<<10;
        std::vector< int > svIn1Vec( length );
        std::vector< int > stlOut( length );
        bolt::BCKND::device_vector< int > dvIn1Vec( length );

        add_3 add3;
        gen_input gen;
        typedef std::vector< int >::const_iterator                                                     sv_itr;
        typedef bolt::BCKND::device_vector< int >::iterator                                            dv_itr;
        typedef bolt::BCKND::counting_iterator< int >                                                  counting_itr;
        typedef bolt::BCKND::constant_iterator< int >                                                  constant_itr;
        typedef bolt::BCKND::transform_iterator< add_3, std::vector< int >::const_iterator>            sv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_3, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_4, std::vector< int >::const_iterator>            sv_trf_itr_add4;
        typedef bolt::BCKND::transform_iterator< add_4, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add4;    
        /*Create Iterators*/
        sv_trf_itr_add3 sv_trf_begin1 (svIn1Vec.begin(), add3), sv_trf_end1 (svIn1Vec.end(), add3);

        dv_trf_itr_add3 dv_trf_begin1 (dvIn1Vec.begin(), add3), dv_trf_end1 (dvIn1Vec.end(), add3);

        counting_itr count_itr_begin(0);
        counting_itr count_itr_end = count_itr_begin + length;
        constant_itr const_itr_begin(1);
        constant_itr const_itr_end = const_itr_begin + length;

        /*Generate inputs*/
        global_id = 0;
        std::generate(svIn1Vec.begin(), svIn1Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn1Vec.begin(), dvIn1Vec.end(), gen);
        global_id = 0;

        int init = (int) rand();
        bolt::cl::plus<int> plus;
        {/*Test case when inputs are trf Iterators*/
            int sv_result = bolt::cl::transform_reduce(sv_trf_begin1, sv_trf_end1, add3, init, plus);
            int dv_result = bolt::cl::transform_reduce(dv_trf_begin1, dv_trf_end1,  add3, init, plus);
            /*Compute expected results*/
            std::transform(sv_trf_begin1, sv_trf_end1, stlOut.begin(), add3);
            int expected_result = std::accumulate(stlOut.begin(), stlOut.end(), init, plus);
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }

        {/*Test case when the both are randomAccessIterator */
            int sv_result = bolt::cl::transform_reduce(svIn1Vec.begin(), svIn1Vec.end(), add3, init, plus);
            int dv_result = bolt::cl::transform_reduce(dvIn1Vec.begin(), dvIn1Vec.end(), add3, init, plus);
            /*Compute expected results*/
            std::transform(svIn1Vec.begin(), svIn1Vec.end(), stlOut.begin(), add3);
            int expected_result = std::accumulate(stlOut.begin(), stlOut.end(), init, plus);
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }
        {/*Test case when the first input is constant iterator and the second is a counting iterator */
            int sv_result = bolt::cl::transform_reduce(const_itr_begin, const_itr_end, add3, init, plus);
            int dv_result = bolt::cl::transform_reduce(const_itr_begin, const_itr_end, add3, init, plus);
            /*Compute expected results*/
            std::vector<int> const_vector(length,1);
            std::transform(const_vector.begin(), const_vector.end(), stlOut.begin(), add3);
            int expected_result = std::accumulate(stlOut.begin(), stlOut.end(), init, plus);
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }
        {/*Test case when the first input is constant iterator and the second is a counting iterator */
            int sv_result = bolt::cl::transform_reduce(count_itr_begin, count_itr_end, add3, init, plus);
            int dv_result = bolt::cl::transform_reduce(count_itr_begin, count_itr_end, add3, init, plus);
            /*Compute expected results*/
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;
            std::transform(count_vector.begin(), count_vector.end(), stlOut.begin(), add3);
            int expected_result = std::accumulate(stlOut.begin(), stlOut.end(), init, plus);
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }
        global_id = 0; // Reset the global id counter
    }
}


#if 0
TEST( TransformIterator, CopyRoutine)
{
    {
        const int length = 1<<10;
        std::vector< int > svIn1Vec( length );
        std::vector< int > svOutVec( length );
        std::vector< int > stlOut( length );
        bolt::BCKND::device_vector< int > dvIn1Vec( length );
        bolt::BCKND::device_vector< int > dvOutVec( length );

        add_3 add3;

        gen_input gen;
        typedef std::vector< int >::const_iterator                                                   sv_itr;
        typedef bolt::BCKND::device_vector< int >::iterator                                          dv_itr;
        typedef bolt::BCKND::counting_iterator< int >                                                counting_itr;
        typedef bolt::BCKND::constant_iterator< int >                                                constant_itr;
        typedef bolt::BCKND::transform_iterator< add_3, std::vector< int >::const_iterator>          sv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_3, bolt::BCKND::device_vector< int >::iterator> dv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_4, std::vector< int >::const_iterator>          sv_trf_itr_add4;
        typedef bolt::BCKND::transform_iterator< add_4, bolt::BCKND::device_vector< int >::iterator> dv_trf_itr_add4;    
        /*Create Iterators*/
        sv_trf_itr_add3 sv_trf_begin1 (svIn1Vec.begin(), add3), sv_trf_end1 (svIn1Vec.end(), add3);

        dv_trf_itr_add3 dv_trf_begin1 (dvIn1Vec.begin(), add3), dv_trf_end1 (dvIn1Vec.end(), add3);

        counting_itr count_itr_begin(0);
        counting_itr count_itr_end = count_itr_begin + length;
        constant_itr const_itr_begin(1);
        constant_itr const_itr_end = const_itr_begin + length;

        /*Generate inputs*/
        global_id = 0;
        std::generate(svIn1Vec.begin(), svIn1Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn1Vec.begin(), dvIn1Vec.end(), gen);
        global_id = 0;
        {/*Test case when inputs are trf Iterators*/
            bolt::cl::copy(sv_trf_begin1, sv_trf_end1, svOutVec.begin());
            bolt::cl::copy(dv_trf_begin1, dv_trf_end1, dvOutVec.begin());
            /*Compute expected results*/
            std::copy(sv_trf_begin1, sv_trf_end1, stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the both are randomAccessIterator */
            bolt::cl::copy(svIn1Vec.begin(), svIn1Vec.end(), svOutVec.begin());
            bolt::cl::copy(dvIn1Vec.begin(), dvIn1Vec.end(), dvOutVec.begin());
            /*Compute expected results*/
            std::copy(svIn1Vec.begin(), svIn1Vec.end(), stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the first input is constant iterator and the second is a counting iterator */
            bolt::cl::copy(const_itr_begin, const_itr_end, svOutVec.begin());
            bolt::cl::copy(const_itr_begin, const_itr_end, dvOutVec.begin());
            /*Compute expected results*/
            std::vector<int> const_vector(length,1);
            std::copy(const_vector.begin(), const_vector.end(), stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        {/*Test case when the first input is constant iterator and the second is a counting iterator */
            //bolt::cl::copy(count_itr_begin, count_itr_end, svOutVec.begin());
            //bolt::cl::copy(count_itr_begin, count_itr_end, dvOutVec.begin());
            bolt::cl::copy_n(count_itr_begin, length, svOutVec.begin());
            bolt::cl::copy_n(count_itr_begin, length, dvOutVec.begin());
            /*Compute expected results*/
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;
            std::copy(count_vector.begin(), count_vector.end(), stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        global_id = 0; // Reset the global id counter
    }
}
#endif




TEST( TransformIterator, CountRoutine)
{
    {
        const int length = 1<<10;
        std::vector< int > svIn1Vec( length );
        //std::vector< int > stlOut( length );
        bolt::BCKND::device_vector< int > dvIn1Vec( length );

        add_3 add3;
        gen_input gen;
        typedef std::vector< int >::const_iterator                                                     sv_itr;
        typedef bolt::BCKND::device_vector< int >::iterator                                            dv_itr;
        typedef bolt::BCKND::counting_iterator< int >                                                  counting_itr;
        typedef bolt::BCKND::constant_iterator< int >                                                  constant_itr;
        typedef bolt::BCKND::transform_iterator< add_3, std::vector< int >::const_iterator>            sv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_3, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_4, std::vector< int >::const_iterator>            sv_trf_itr_add4;
        typedef bolt::BCKND::transform_iterator< add_4, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add4;    
        /*Create Iterators*/
        sv_trf_itr_add3 sv_trf_begin1 (svIn1Vec.begin(), add3), sv_trf_end1 (svIn1Vec.end(), add3);

        dv_trf_itr_add3 dv_trf_begin1 (dvIn1Vec.begin(), add3), dv_trf_end1 (dvIn1Vec.end(), add3);

        counting_itr count_itr_begin(0);
        counting_itr count_itr_end = count_itr_begin + length;
        constant_itr const_itr_begin(1);
        constant_itr const_itr_end = const_itr_begin + length;

        /*Generate inputs*/
        global_id = 0;
        std::generate(svIn1Vec.begin(), svIn1Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn1Vec.begin(), dvIn1Vec.end(), gen);
        global_id = 0;

        int val = (int) rand();

        {/*Test case when inputs are trf Iterators*/
            bolt::cl::iterator_traits<bolt::cl::device_vector<int>::iterator>::difference_type sv_result = (int) bolt::cl::count(sv_trf_begin1, sv_trf_end1, val);
            bolt::cl::iterator_traits<bolt::cl::device_vector<int>::iterator>::difference_type dv_result = (int) bolt::cl::count(dv_trf_begin1, dv_trf_end1, val);
            /*Compute expected results*/
            std::iterator_traits<std::vector<int>::iterator>::difference_type expected_result = std::count(sv_trf_begin1, sv_trf_end1, val);
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }
        {/*Test case when the both are randomAccessIterator */
            bolt::cl::iterator_traits<bolt::cl::device_vector<int>::iterator>::difference_type sv_result = (int) bolt::cl::count(svIn1Vec.begin(), svIn1Vec.end(), val);
            bolt::cl::iterator_traits<bolt::cl::device_vector<int>::iterator>::difference_type dv_result = (int) bolt::cl::count(dvIn1Vec.begin(), dvIn1Vec.end(), val);
            /*Compute expected results*/
            std::iterator_traits<std::vector<int>::iterator>::difference_type expected_result = std::count(svIn1Vec.begin(), svIn1Vec.end(), val);
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }
        {/*Test case when the first input is constant iterator and the second is a counting iterator */
            bolt::cl::iterator_traits<bolt::cl::device_vector<int>::iterator>::difference_type sv_result = (int) bolt::cl::count(const_itr_begin, const_itr_end, val);
            bolt::cl::iterator_traits<bolt::cl::device_vector<int>::iterator>::difference_type dv_result = (int) bolt::cl::count(const_itr_begin, const_itr_end, val);
            /*Compute expected results*/
            std::vector<int> const_vector(length,1);
            std::iterator_traits<std::vector<int>::iterator>::difference_type expected_result = std::count(const_vector.begin(), const_vector.end(), val);
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }
        {/*Test case when the first input is constant iterator and the second is a counting iterator */
            bolt::cl::iterator_traits<bolt::cl::device_vector<int>::iterator>::difference_type sv_result = (int) bolt::cl::count(count_itr_begin, count_itr_end, val);
            bolt::cl::iterator_traits<bolt::cl::device_vector<int>::iterator>::difference_type dv_result = (int) bolt::cl::count(count_itr_begin, count_itr_end, val);
            /*Compute expected results*/
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;
            std::iterator_traits<std::vector<int>::iterator>::difference_type expected_result = std::count(count_vector.begin(), count_vector.end(), val);
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }
        global_id = 0; // Reset the global id counter
    }
}

TEST( TransformIterator, InnerProductRoutine)
{
    {
        const int length = 1<<10;
        std::vector< int > svIn1Vec( length );
        std::vector< int > svIn2Vec( length);
        bolt::BCKND::device_vector< int > dvIn1Vec( length );

        std::vector< int > stlOut( length );

        add_3 add3;
        bolt::cl::plus<int> plus;
        bolt::cl::minus<int> minus;

        gen_input gen;
        typedef std::vector< int >::const_iterator                                                     sv_itr;
        typedef bolt::BCKND::device_vector< int >::iterator                                            dv_itr;
        typedef bolt::BCKND::counting_iterator< int >                                                  counting_itr;
        typedef bolt::BCKND::constant_iterator< int >                                                  constant_itr;
        typedef bolt::BCKND::transform_iterator< add_3, std::vector< int >::const_iterator>            sv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_3, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add3;
       
        /*Create Iterators*/
        sv_trf_itr_add3 sv_trf_begin1 (svIn1Vec.begin(), add3), sv_trf_end1 (svIn1Vec.end(), add3);
        sv_trf_itr_add3 sv_trf_begin2 (svIn2Vec.begin(), add3);
        dv_trf_itr_add3 dv_trf_begin1 (dvIn1Vec.begin(), add3), dv_trf_end1 (dvIn1Vec.end(), add3);

        counting_itr count_itr_begin(0);
        counting_itr count_itr_end = count_itr_begin + length;
        counting_itr count_itr_begin2(10);
        constant_itr const_itr_begin(1);
        constant_itr const_itr_end = const_itr_begin + length;
        constant_itr const_itr_begin2(5);

        /*Generate inputs*/
        global_id = 0;
        std::generate(svIn1Vec.begin(), svIn1Vec.end(), gen);
        global_id = 0;
        std::generate(svIn2Vec.begin(), svIn2Vec.end(), rand);
        global_id = 0;
        bolt::BCKND::generate(dvIn1Vec.begin(), dvIn1Vec.end(), gen);
        global_id = 0;

        bolt::BCKND::device_vector< int > dvIn2Vec( svIn2Vec.begin(), svIn2Vec.end());
        dv_trf_itr_add3 dv_trf_begin2 (dvIn2Vec.begin(), add3);

        global_id = 0;

        int init = (int) rand();


        {/*Test case when both inputs are trf Iterators*/
            int sv_result = bolt::cl::inner_product(sv_trf_begin1, sv_trf_end1, sv_trf_begin2, init, plus, minus);
            int dv_result = bolt::cl::inner_product(dv_trf_begin1, dv_trf_end1, dv_trf_begin2, init, plus, minus);
            /*Compute expected results*/
            std::transform(sv_trf_begin1, sv_trf_end1, sv_trf_begin2, stlOut.begin(), minus);
            int expected_result = std::accumulate(stlOut.begin(), stlOut.end(), init, plus);
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }
        {/*Test case when the both inputs are randomAccessIterator */
            int sv_result = bolt::cl::inner_product(svIn1Vec.begin(), svIn1Vec.end(), svIn2Vec.begin(), init, plus, minus);
            int dv_result = bolt::cl::inner_product(dvIn1Vec.begin(), dvIn1Vec.end(), dvIn2Vec.begin(), init, plus, minus);
            /*Compute expected results*/
            int expected_result = std::inner_product(svIn1Vec.begin(), svIn1Vec.end(), svIn2Vec.begin(), init, std::plus<int>(), std::minus<int>());
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }
        {/*Test case when both inputs are constant iterator */
            int sv_result = bolt::cl::inner_product(const_itr_begin, const_itr_end, const_itr_begin2, init, plus, minus);
            int dv_result = bolt::cl::inner_product(const_itr_begin, const_itr_end, const_itr_begin2, init, plus, minus);
            /*Compute expected results*/
            std::vector<int> const_vector(length,1);
            std::vector<int> const_vector2(length,5);
            int expected_result = std::inner_product(const_vector.begin(), const_vector.end(), const_vector2.begin(), init, std::plus<int>(), std::minus<int>());
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }
        {/*Test case when both inputs are counting iterator */
            int sv_result = bolt::cl::inner_product(count_itr_begin, count_itr_end, count_itr_begin2, init, plus, minus);
            int dv_result = bolt::cl::inner_product(count_itr_begin, count_itr_end, count_itr_begin2, init, plus, minus);
            /*Compute expected results*/
            std::vector<int> count_vector(length);
            std::vector<int> count_vector2(length);
            for (int index=0;index<length;index++)
            {
                count_vector[index] = index;
                count_vector2[index] = 10 + index;
            }
            int expected_result = std::inner_product(count_vector.begin(), count_vector.end(), count_vector2.begin(), init, std::plus<int>(), std::minus<int>());
            /*Check the results*/
            EXPECT_EQ( expected_result, sv_result );
            EXPECT_EQ( expected_result, dv_result );
        }
        global_id = 0; // Reset the global id counter
    }
}


TEST( TransformIterator, ScatterRoutine)
{
    {
        const int length = 1<<10;
        std::vector< int > svIn1Vec( length );
        std::vector< int > svIn2Vec( length );
        std::vector< int > svOutVec( length );
        std::vector< int > stlOut( length );
        bolt::BCKND::device_vector< int > dvIn1Vec( length );
        bolt::BCKND::device_vector< int > dvIn2Vec( length );
        bolt::BCKND::device_vector< int > dvOutVec( length );

        add_3 add3;
        //add_4 add4;
		add_0 add0;

        gen_input gen;
        typedef std::vector< int >::const_iterator                                                     sv_itr;
        typedef bolt::BCKND::device_vector< int >::iterator                                            dv_itr;
        typedef bolt::BCKND::counting_iterator< int >                                                  counting_itr;
        typedef bolt::BCKND::constant_iterator< int >                                                  constant_itr;
        typedef bolt::BCKND::transform_iterator< add_3, std::vector< int >::const_iterator>            sv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_3, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_0, std::vector< int >::const_iterator>            sv_trf_itr_add4;
		//typedef bolt::BCKND::transform_iterator< add_0, std::vector< int >::iterator>            sv_trf_itr_add4;
        typedef bolt::BCKND::transform_iterator< add_0, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add4;    
        /*Create Iterators*/
        sv_trf_itr_add3 sv_trf_begin1 (svIn1Vec.begin(), add3), sv_trf_end1 (svIn1Vec.end(), add3);
        sv_trf_itr_add4 sv_trf_begin2 (svIn2Vec.begin(), add0);
        dv_trf_itr_add3 dv_trf_begin1 (dvIn1Vec.begin(), add3), dv_trf_end1 (dvIn1Vec.end(), add3);
        dv_trf_itr_add4 dv_trf_begin2 (dvIn2Vec.begin(), add0);
        counting_itr count_itr_begin(0);
        counting_itr count_itr_end = count_itr_begin + length;
        constant_itr const_itr_begin(1);
        constant_itr const_itr_end = const_itr_begin + length;


        /*Generate inputs*/
        global_id = 0;
        std::generate(svIn1Vec.begin(), svIn1Vec.end(), gen);
        global_id = 0;
        std::generate(svIn2Vec.begin(), svIn2Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn1Vec.begin(), dvIn1Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn2Vec.begin(), dvIn2Vec.end(), gen);
        global_id = 0;


        {/*Test case when both inputs are trf Iterators*/
            bolt::cl::scatter(sv_trf_begin1, sv_trf_end1, sv_trf_begin2, svOutVec.begin());
            bolt::cl::scatter(dv_trf_begin1, dv_trf_end1, dv_trf_begin2, dvOutVec.begin());
            /*Compute expected results*/
            Serial_scatter(sv_trf_begin1, sv_trf_end1, sv_trf_begin2, stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

        {/*Test case when the first input is trf_itr and the second is a randomAccessIterator */
            bolt::cl::scatter(sv_trf_begin1, sv_trf_end1, svIn2Vec.begin(), svOutVec.begin());
            bolt::cl::scatter(dv_trf_begin1, dv_trf_end1, dvIn2Vec.begin(), dvOutVec.begin());
            /*Compute expected results*/
            Serial_scatter(sv_trf_begin1, sv_trf_end1, svIn2Vec.begin(), stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

        {/*Test case when the first input is a randomAccessIterator  and second is a trf_itr */
            bolt::cl::scatter(svIn1Vec.begin(), svIn1Vec.end(), sv_trf_begin2, svOutVec.begin());
            bolt::cl::scatter(dvIn1Vec.begin(), dvIn1Vec.end(), dv_trf_begin2, dvOutVec.begin());
            /*Compute expected results*/
            Serial_scatter(svIn1Vec.begin(), svIn1Vec.end(), sv_trf_begin2, stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

        {/*Test case when the both are randomAccessIterator */
            bolt::cl::scatter(svIn1Vec.begin(), svIn1Vec.end(), svIn2Vec.begin(), svOutVec.begin());
            bolt::cl::scatter(dvIn1Vec.begin(), dvIn1Vec.end(), dvIn2Vec.begin(), dvOutVec.begin());
            /*Compute expected results*/
            Serial_scatter(svIn1Vec.begin(), svIn1Vec.end(), svIn2Vec.begin(), stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
        // Map cannot be a constant iterator! 

        //{/*Test case when the first input is trf_itr and the second is a constant iterator */
        //    bolt::cl::scatter(sv_trf_begin1, sv_trf_end1, const_itr_begin, svOutVec.begin());
        //    bolt::cl::scatter(dv_trf_begin1, dv_trf_end1, const_itr_begin, dvOutVec.begin());
        //    /*Compute expected results*/
        //    std::vector<int> const_vector(length,1);
        //    Serial_scatter(sv_trf_begin1, sv_trf_end1, const_vector.begin(), stlOut.begin());
        //    /*Check the results*/
        //    cmpArrays(svOutVec, stlOut, length);
        //    cmpArrays(dvOutVec, stlOut, length);
        //}

        {/*Test case when the first input is trf_itr and the second is a counting iterator */
            bolt::cl::scatter(sv_trf_begin1, sv_trf_end1, count_itr_begin, svOutVec.begin());
            bolt::cl::scatter(dv_trf_begin1, dv_trf_end1, count_itr_begin, dvOutVec.begin());
            /*Compute expected results*/
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;
            Serial_scatter(sv_trf_begin1, sv_trf_end1, count_vector.begin(), stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }


		{/*Test case when the first input is constant iterator and the second is a randomAccessIterator */
            bolt::cl::scatter(const_itr_begin, const_itr_end, svIn2Vec.begin(), svOutVec.begin());
            bolt::cl::scatter(const_itr_begin, const_itr_end, dvIn2Vec.begin(), dvOutVec.begin());
            /*Compute expected results*/
            std::vector<int> const_vector(length,1);          
            Serial_scatter(const_vector.begin(), const_vector.end(), svIn2Vec.begin(), stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }


        {/*Test case when the first input is constant iterator and the second is a counting iterator */
            bolt::cl::scatter(const_itr_begin, const_itr_end, count_itr_begin, svOutVec.begin());
            bolt::cl::scatter(const_itr_begin, const_itr_end, count_itr_begin, dvOutVec.begin());
            /*Compute expected results*/
            std::vector<int> const_vector(length,1);
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;            
            Serial_scatter(const_vector.begin(), const_vector.end(), count_vector.begin(), stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

		 {/*Test case when the first input is a randomAccessIterator and the second is a counting iterator */
            bolt::cl::scatter(svIn1Vec.begin(), svIn1Vec.end(), count_itr_begin, svOutVec.begin());
            bolt::cl::scatter(dvIn1Vec.begin(), dvIn1Vec.end(), count_itr_begin, dvOutVec.begin());
            /*Compute expected results*/
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;            
            Serial_scatter(svIn1Vec.begin(), svIn1Vec.end(), count_vector.begin(), stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }


        global_id = 0; // Reset the global id counter
    }
 }


 TEST( TransformIterator, ScatterIfRoutine)
 {
    {
        const int length = 1<<10;
        std::vector< int > svIn1Vec( length ); // input
        std::vector< int > svIn2Vec( length ); // map
		std::vector< int > svIn3Vec( length ); // stencil
        std::vector< int > svOutVec( length );
        std::vector< int > stlOut( length );
        bolt::BCKND::device_vector< int > dvIn1Vec( length );
        bolt::BCKND::device_vector< int > dvIn2Vec( length );
        bolt::BCKND::device_vector< int > dvOutVec( length );

		for(int i=0; i<length; i++)
		{
			if(i%2 == 0)
				svIn3Vec[i] = 0;
			else
				svIn3Vec[i] = 1;
		}
		bolt::BCKND::device_vector< int > dvIn3Vec( svIn3Vec.begin(), svIn3Vec.end() );

        add_3 add3;
		add_0 add0;

        gen_input gen;
        typedef std::vector< int >::const_iterator                                                     sv_itr;
        typedef bolt::BCKND::device_vector< int >::iterator                                            dv_itr;
        typedef bolt::BCKND::counting_iterator< int >                                                  counting_itr;
        typedef bolt::BCKND::constant_iterator< int >                                                  constant_itr;
        typedef bolt::BCKND::transform_iterator< add_3, std::vector< int >::const_iterator>            sv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_3, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_0, std::vector< int >::const_iterator>            sv_trf_itr_add4;
        typedef bolt::BCKND::transform_iterator< add_0, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add4;  
		typedef bolt::BCKND::transform_iterator< add_0, std::vector< int >::const_iterator>            sv_trf_itr_add0;
        typedef bolt::BCKND::transform_iterator< add_0, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add0; 

        /*Create Iterators*/
        sv_trf_itr_add3 sv_trf_begin1 (svIn1Vec.begin(), add3), sv_trf_end1 (svIn1Vec.end(), add3);
        sv_trf_itr_add4 sv_trf_begin2 (svIn2Vec.begin(), add0);
		sv_trf_itr_add0 sv_trf_begin3 (svIn3Vec.begin(), add0);
        dv_trf_itr_add3 dv_trf_begin1 (dvIn1Vec.begin(), add3), dv_trf_end1 (dvIn1Vec.end(), add3);
        dv_trf_itr_add4 dv_trf_begin2 (dvIn2Vec.begin(), add0);
		dv_trf_itr_add0 dv_trf_begin3 (dvIn3Vec.begin(), add0);

        counting_itr count_itr_begin(0);
        counting_itr count_itr_end = count_itr_begin + length;
        constant_itr const_itr_begin(1);
        constant_itr const_itr_end = const_itr_begin + length;
		constant_itr stencil_itr_begin(1);
        constant_itr stencil_itr_end = stencil_itr_begin + length;

        /*Generate inputs*/
        global_id = 0;
        std::generate(svIn1Vec.begin(), svIn1Vec.end(), gen);
        global_id = 0;
        std::generate(svIn2Vec.begin(), svIn2Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn1Vec.begin(), dvIn1Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn2Vec.begin(), dvIn2Vec.end(), gen);
        global_id = 0;

		is_even iepred;

        {/*Test case when both inputs are trf Iterators*/
            bolt::cl::scatter_if(sv_trf_begin1, sv_trf_end1, sv_trf_begin2, sv_trf_begin3, svOutVec.begin(), iepred);
            bolt::cl::scatter_if(dv_trf_begin1, dv_trf_end1, dv_trf_begin2, dv_trf_begin3, dvOutVec.begin(), iepred);
            /*Compute expected results*/
            Serial_scatter_if(sv_trf_begin1, sv_trf_end1, sv_trf_begin2, sv_trf_begin3, stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

        {/*Test case when the first input is trf_itr and the second is a randomAccessIterator */
            bolt::cl::scatter_if(sv_trf_begin1, sv_trf_end1, svIn2Vec.begin(), sv_trf_begin3, svOutVec.begin(), iepred);
            bolt::cl::scatter_if(dv_trf_begin1, dv_trf_end1, dvIn2Vec.begin(), dv_trf_begin3, dvOutVec.begin(), iepred);
            /*Compute expected results*/
            Serial_scatter_if(sv_trf_begin1, sv_trf_end1, svIn2Vec.begin(), sv_trf_begin3, stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

		{/*Test case when the first input is trf_itr and the second is a randomAccessIterator */
            bolt::cl::scatter_if(sv_trf_begin1, sv_trf_end1, svIn2Vec.begin(), svIn3Vec.begin(), svOutVec.begin(), iepred);
            bolt::cl::scatter_if(dv_trf_begin1, dv_trf_end1, dvIn2Vec.begin(), dvIn3Vec.begin(), dvOutVec.begin(), iepred);
            /*Compute expected results*/
            Serial_scatter_if(sv_trf_begin1, sv_trf_end1, svIn2Vec.begin(), svIn3Vec.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

        {/*Test case when the first input is a randomAccessIterator  and second is a trf_itr */
            bolt::cl::scatter_if(svIn1Vec.begin(), svIn1Vec.end(), sv_trf_begin2, sv_trf_begin3, svOutVec.begin(), iepred);
            bolt::cl::scatter_if(dvIn1Vec.begin(), dvIn1Vec.end(), dv_trf_begin2, dv_trf_begin3, dvOutVec.begin(), iepred);
            /*Compute expected results*/
            Serial_scatter_if(svIn1Vec.begin(), svIn1Vec.end(), sv_trf_begin2, sv_trf_begin3, stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

		{/*Test case when the first input is a randomAccessIterator  and second is a trf_itr */
            bolt::cl::scatter_if(svIn1Vec.begin(), svIn1Vec.end(), sv_trf_begin2, svIn3Vec.begin(), svOutVec.begin(), iepred);
            bolt::cl::scatter_if(dvIn1Vec.begin(), dvIn1Vec.end(), dv_trf_begin2, dvIn3Vec.begin(), dvOutVec.begin(), iepred);
            /*Compute expected results*/
            Serial_scatter_if(svIn1Vec.begin(), svIn1Vec.end(), sv_trf_begin2, svIn3Vec.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }


		{/*Test case when the first input is trf_itr and the second is a counting iterator */
            bolt::cl::scatter_if(sv_trf_begin1, sv_trf_end1, count_itr_begin, svIn3Vec.begin(), svOutVec.begin(), iepred);
            bolt::cl::scatter_if(dv_trf_begin1, dv_trf_end1, count_itr_begin, dvIn3Vec.begin(), dvOutVec.begin(), iepred);
            /*Compute expected results*/
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;
            Serial_scatter_if(sv_trf_begin1, sv_trf_end1, count_vector.begin(), svIn3Vec.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

		{/*Test case when the first input is trf_itr and the second is a counting iterator */
            bolt::cl::scatter_if(sv_trf_begin1, sv_trf_end1, count_itr_begin, stencil_itr_begin, svOutVec.begin(), iepred);
            bolt::cl::scatter_if(dv_trf_begin1, dv_trf_end1, count_itr_begin, stencil_itr_begin, dvOutVec.begin(), iepred);
            /*Compute expected results*/
            std::vector<int> count_vector(length), stencil_vector(length);
            for (int index=0;index<length;index++)
			{
                count_vector[index] = index;
				stencil_vector[index] = 1;
			}
            Serial_scatter_if(sv_trf_begin1, sv_trf_end1, count_vector.begin(), stencil_vector.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }


        {/*Test case when the both are randomAccessIterator */
            bolt::cl::scatter_if(svIn1Vec.begin(), svIn1Vec.end(), svIn2Vec.begin(), svIn3Vec.begin(), svOutVec.begin(), iepred);
            bolt::cl::scatter_if(dvIn1Vec.begin(), dvIn1Vec.end(), dvIn2Vec.begin(), dvIn3Vec.begin(), dvOutVec.begin(), iepred);
            /*Compute expected results*/
            Serial_scatter_if(svIn1Vec.begin(), svIn1Vec.end(), svIn2Vec.begin(), svIn3Vec.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
       
		{/*Test case when the both are randomAccessIterator */
            bolt::cl::scatter_if(svIn1Vec.begin(), svIn1Vec.end(), svIn2Vec.begin(), stencil_itr_begin, svOutVec.begin(), iepred);
            bolt::cl::scatter_if(dvIn1Vec.begin(), dvIn1Vec.end(), dvIn2Vec.begin(), stencil_itr_begin, dvOutVec.begin(), iepred);
            /*Compute expected results*/
			std::vector<int> stencil_vector(length);
            for (int index=0;index<length;index++)
			{
				stencil_vector[index] = 1;
			}
            Serial_scatter_if(svIn1Vec.begin(), svIn1Vec.end(), svIn2Vec.begin(), stencil_vector.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }


		{/*Test case when the first input is constant iterator and the second is a randomAccessIterator */
            bolt::cl::scatter_if(const_itr_begin, const_itr_end, svIn2Vec.begin(), svIn3Vec.begin(), svOutVec.begin(), iepred);
            bolt::cl::scatter_if(const_itr_begin, const_itr_end, dvIn2Vec.begin(), dvIn3Vec.begin(), dvOutVec.begin(), iepred);
            /*Compute expected results*/
            std::vector<int> const_vector(length,1);          
            Serial_scatter_if(const_vector.begin(), const_vector.end(), svIn2Vec.begin(), svIn3Vec.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

		{/*Test case when the first input is constant iterator and the second is a randomAccessIterator */
            bolt::cl::scatter_if(const_itr_begin, const_itr_end, svIn2Vec.begin(), stencil_itr_begin, svOutVec.begin(), iepred);
            bolt::cl::scatter_if(const_itr_begin, const_itr_end, dvIn2Vec.begin(), stencil_itr_begin, dvOutVec.begin(), iepred);
            /*Compute expected results*/
            std::vector<int> const_vector(length,1); 
			std::vector<int> stencil_vector(length);
            for (int index=0;index<length;index++)
			{
				stencil_vector[index] = 1;
			}
            Serial_scatter_if(const_vector.begin(), const_vector.end(), svIn2Vec.begin(), stencil_vector.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }


        {/*Test case when the first input is constant iterator and the second is a counting iterator */
            bolt::cl::scatter_if(const_itr_begin, const_itr_end, count_itr_begin, svIn3Vec.begin(), svOutVec.begin(), iepred);
            bolt::cl::scatter_if(const_itr_begin, const_itr_end, count_itr_begin, dvIn3Vec.begin(), dvOutVec.begin(), iepred);
            /*Compute expected results*/
            std::vector<int> const_vector(length,1);
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;            
            Serial_scatter_if(const_vector.begin(), const_vector.end(), count_vector.begin(), svIn3Vec.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

		{/*Test case when the first input is constant iterator and the second is a counting iterator */
            bolt::cl::scatter_if(const_itr_begin, const_itr_end, count_itr_begin, stencil_itr_begin, svOutVec.begin(), iepred);
            bolt::cl::scatter_if(const_itr_begin, const_itr_end, count_itr_begin, stencil_itr_begin, dvOutVec.begin(), iepred);
            /*Compute expected results*/
            std::vector<int> const_vector(length,1);
            std::vector<int> count_vector(length);
			std::vector<int> stencil_vector(length);
            for (int index=0;index<length;index++)
			{
				stencil_vector[index] = 1;
				count_vector[index] = index; 
			}
                           
            Serial_scatter_if(const_vector.begin(), const_vector.end(), count_vector.begin(),  stencil_vector.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }


		{/*Test case when the first input is a randomAccessIterator and the second is a counting iterator */
            bolt::cl::scatter_if(svIn1Vec.begin(), svIn1Vec.end(), count_itr_begin, svIn3Vec.begin(), svOutVec.begin(), iepred);
            bolt::cl::scatter_if(dvIn1Vec.begin(), dvIn1Vec.end(), count_itr_begin, dvIn3Vec.begin(), dvOutVec.begin(), iepred);
            /*Compute expected results*/
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;            
            Serial_scatter_if(svIn1Vec.begin(), svIn1Vec.end(), count_vector.begin(), svIn3Vec.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

		{/*Test case when the first input is a randomAccessIterator and the second is a counting iterator */
            bolt::cl::scatter_if(svIn1Vec.begin(), svIn1Vec.end(), count_itr_begin, stencil_itr_begin, svOutVec.begin(), iepred);
            bolt::cl::scatter_if(dvIn1Vec.begin(), dvIn1Vec.end(), count_itr_begin, stencil_itr_begin, dvOutVec.begin(), iepred);
            /*Compute expected results*/
            std::vector<int> count_vector(length);
			std::vector<int> stencil_vector(length);
            for (int index=0;index<length;index++)
			{
				stencil_vector[index] = 1;
				count_vector[index] = index; 
			}
            Serial_scatter_if(svIn1Vec.begin(), svIn1Vec.end(), count_vector.begin(), stencil_vector.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

        global_id = 0; // Reset the global id counter
    }
 }


 TEST( TransformIterator, GatherRoutine)
{
    {
        const int length = 1<<10;
        std::vector< int > svIn1Vec( length );
        std::vector< int > svIn2Vec( length );
        std::vector< int > svOutVec( length );
        std::vector< int > stlOut( length );
        bolt::BCKND::device_vector< int > dvIn1Vec( length );
        bolt::BCKND::device_vector< int > dvIn2Vec( length );
        bolt::BCKND::device_vector< int > dvOutVec( length );

        add_3 add3;
        //add_4 add4;
		add_0 add0;

        gen_input gen;
        typedef std::vector< int >::const_iterator                                                     sv_itr;
        typedef bolt::BCKND::device_vector< int >::iterator                                            dv_itr;
        typedef bolt::BCKND::counting_iterator< int >                                                  counting_itr;
        typedef bolt::BCKND::constant_iterator< int >                                                  constant_itr;
        typedef bolt::BCKND::transform_iterator< add_3, std::vector< int >::const_iterator>            sv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_3, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_0, std::vector< int >::const_iterator>            sv_trf_itr_add4;
		//typedef bolt::BCKND::transform_iterator< add_0, std::vector< int >::iterator>            sv_trf_itr_add4;
        typedef bolt::BCKND::transform_iterator< add_0, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add4;    
        /*Create Iterators*/
        sv_trf_itr_add3 sv_trf_begin1 (svIn1Vec.begin(), add3); 
        sv_trf_itr_add4 sv_trf_begin2 (svIn2Vec.begin(), add0), sv_trf_end2 (svIn2Vec.end(), add0);
        dv_trf_itr_add3 dv_trf_begin1 (dvIn1Vec.begin(), add3);
        dv_trf_itr_add4 dv_trf_begin2 (dvIn2Vec.begin(), add0), dv_trf_end2 (dvIn2Vec.end(), add0);
        counting_itr count_itr_begin(0);
        counting_itr count_itr_end = count_itr_begin + length;
        constant_itr const_itr_begin(1);
        constant_itr const_itr_end = const_itr_begin + length;


        /*Generate inputs*/
        global_id = 0;
        std::generate(svIn1Vec.begin(), svIn1Vec.end(), gen);
        global_id = 0;
        std::generate(svIn2Vec.begin(), svIn2Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn1Vec.begin(), dvIn1Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn2Vec.begin(), dvIn2Vec.end(), gen);
        global_id = 0;


        {/*Test case when both inputs are trf Iterators*/
            bolt::cl::gather(sv_trf_begin2, sv_trf_end2, sv_trf_begin1, svOutVec.begin());
            bolt::cl::gather(dv_trf_begin2, dv_trf_end2, dv_trf_begin1, dvOutVec.begin());
            /*Compute expected results*/
            Serial_gather(sv_trf_begin2, sv_trf_end2, sv_trf_begin1, stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

        {/*Test case when the first input is trf_itr and map is a randomAccessIterator */
            bolt::cl::gather(sv_trf_begin2, sv_trf_end2, svIn1Vec.begin(), svOutVec.begin());
            bolt::cl::gather(dv_trf_begin2, dv_trf_end2, dvIn1Vec.begin(), dvOutVec.begin());
            /*Compute expected results*/
            Serial_gather(sv_trf_begin2, sv_trf_end2, svIn1Vec.begin(), stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

        {/*Test case when the first input is a randomAccessIterator  and map is a trf_itr */
            bolt::cl::gather(svIn2Vec.begin(), svIn2Vec.end(), sv_trf_begin1, svOutVec.begin());
            bolt::cl::gather(dvIn2Vec.begin(), dvIn2Vec.end(), dv_trf_begin1, dvOutVec.begin());
            /*Compute expected results*/
            Serial_gather(svIn2Vec.begin(), svIn2Vec.end(), sv_trf_begin1, stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

        {/*Test case when the both are randomAccessIterator */
            bolt::cl::gather(svIn2Vec.begin(), svIn2Vec.end(), svIn1Vec.begin(), svOutVec.begin());
            bolt::cl::gather(dvIn2Vec.begin(), dvIn2Vec.end(), dvIn1Vec.begin(), dvOutVec.begin());
            /*Compute expected results*/
            Serial_gather(svIn2Vec.begin(), svIn2Vec.end(), svIn1Vec.begin(), stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

        {/*Test case when the first input is trf_itr and map is a counting iterator */
            bolt::cl::gather(count_itr_begin, count_itr_end, sv_trf_begin1, svOutVec.begin());
            bolt::cl::gather(count_itr_begin, count_itr_end, dv_trf_begin1, dvOutVec.begin());
            /*Compute expected results*/
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;
            Serial_gather(count_vector.begin(), count_vector.end(), sv_trf_begin1, stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }


		{/*Test case when the first input is constant iterator and map is a randomAccessIterator */
            bolt::cl::gather(svIn2Vec.begin(), svIn2Vec.end(), const_itr_begin, svOutVec.begin());
            bolt::cl::gather(dvIn2Vec.begin(), dvIn2Vec.end(), const_itr_begin, dvOutVec.begin());
            /*Compute expected results*/
            std::vector<int> const_vector(length,1);          
            Serial_gather(svIn2Vec.begin(), svIn2Vec.end(), const_vector.begin(), stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }


        {/*Test case when the first input is constant iterator and map is a counting iterator */
            bolt::cl::gather(count_itr_begin,  count_itr_end, const_itr_begin, svOutVec.begin());
            bolt::cl::gather(count_itr_begin,  count_itr_end, const_itr_begin, dvOutVec.begin());
            /*Compute expected results*/
            std::vector<int> const_vector(length,1);
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;            
            Serial_gather( count_vector.begin(), count_vector.end(), const_vector.begin(), stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

		 {/*Test case when the first input is a randomAccessIterator and map is a counting iterator */
            bolt::cl::gather(count_itr_begin, count_itr_end, svIn1Vec.begin(), svOutVec.begin());
            bolt::cl::gather(count_itr_begin, count_itr_end, dvIn1Vec.begin(), dvOutVec.begin());
            /*Compute expected results*/
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;            
            Serial_gather(count_vector.begin(), count_vector.end(), svIn1Vec.begin(),stlOut.begin());
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }


        global_id = 0; // Reset the global id counter
    }
 }


 TEST( TransformIterator, GatherIfRoutine)
 {
    {
        const int length = 1<<10;
        std::vector< int > svIn1Vec( length ); // input
        std::vector< int > svIn2Vec( length ); // map
		std::vector< int > svIn3Vec( length ); // stencil
        std::vector< int > svOutVec( length );
        std::vector< int > stlOut( length );
        bolt::BCKND::device_vector< int > dvIn1Vec( length );
        bolt::BCKND::device_vector< int > dvIn2Vec( length );
        bolt::BCKND::device_vector< int > dvOutVec( length );

		for(int i=0; i<length; i++)
		{
			if(i%2 == 0)
				svIn3Vec[i] = 0;
			else
				svIn3Vec[i] = 1;
		}
		bolt::BCKND::device_vector< int > dvIn3Vec( svIn3Vec.begin(), svIn3Vec.end() );

        add_3 add3;
		add_0 add0;

        gen_input gen;
        typedef std::vector< int >::const_iterator                                                     sv_itr;
        typedef bolt::BCKND::device_vector< int >::iterator                                            dv_itr;
        typedef bolt::BCKND::counting_iterator< int >                                                  counting_itr;
        typedef bolt::BCKND::constant_iterator< int >                                                  constant_itr;
        typedef bolt::BCKND::transform_iterator< add_3, std::vector< int >::const_iterator>            sv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_3, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_0, std::vector< int >::const_iterator>            sv_trf_itr_add4;
        typedef bolt::BCKND::transform_iterator< add_0, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add4;  
		typedef bolt::BCKND::transform_iterator< add_0, std::vector< int >::const_iterator>            sv_trf_itr_add0;
        typedef bolt::BCKND::transform_iterator< add_0, bolt::BCKND::device_vector< int >::iterator>   dv_trf_itr_add0; 

        /*Create Iterators*/
        sv_trf_itr_add3 sv_trf_begin1 (svIn1Vec.begin(), add3); // Input
        sv_trf_itr_add4 sv_trf_begin2 (svIn2Vec.begin(), add0), sv_trf_end2 (svIn2Vec.end(), add0); //Map
		sv_trf_itr_add0 sv_trf_begin3 (svIn3Vec.begin(), add0); // Stencil
        dv_trf_itr_add3 dv_trf_begin1 (dvIn1Vec.begin(), add3);
        dv_trf_itr_add4 dv_trf_begin2 (dvIn2Vec.begin(), add0), dv_trf_end2 (dvIn2Vec.end(), add0);
		dv_trf_itr_add0 dv_trf_begin3 (dvIn3Vec.begin(), add0);

        counting_itr count_itr_begin(0);
        counting_itr count_itr_end = count_itr_begin + length;
        constant_itr const_itr_begin(1);
        constant_itr const_itr_end = const_itr_begin + length;
		constant_itr stencil_itr_begin(1);
        constant_itr stencil_itr_end = stencil_itr_begin + length;

        /*Generate inputs*/
        global_id = 0;
        std::generate(svIn1Vec.begin(), svIn1Vec.end(), gen);
        global_id = 0;
        std::generate(svIn2Vec.begin(), svIn2Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn1Vec.begin(), dvIn1Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn2Vec.begin(), dvIn2Vec.end(), gen);
        global_id = 0;

		is_even iepred;

        {/*Test case when both inputs are trf Iterators*/
            bolt::cl::gather_if(sv_trf_begin2, sv_trf_end2, sv_trf_begin3, sv_trf_begin1, svOutVec.begin(), iepred);
            bolt::cl::gather_if(dv_trf_begin2, dv_trf_end2, dv_trf_begin3, dv_trf_begin1, dvOutVec.begin(), iepred);
            /*Compute expected results*/
            Serial_gather_if(sv_trf_begin2, sv_trf_end2, sv_trf_begin3, sv_trf_begin1, stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

        {/*Test case when the first input is trf_itr and the second is a randomAccessIterator */
            bolt::cl::gather_if(svIn2Vec.begin(), svIn2Vec.end(), sv_trf_begin3, sv_trf_begin1, svOutVec.begin(), iepred);
            bolt::cl::gather_if(dvIn2Vec.begin(), dvIn2Vec.end(), dv_trf_begin3, dv_trf_begin1, dvOutVec.begin(), iepred);
            /*Compute expected results*/
            Serial_gather_if(svIn2Vec.begin(), svIn2Vec.end(), sv_trf_begin3, sv_trf_begin1, stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
		{/*Test case when the first input is trf_itr and the second is a randomAccessIterator */
            bolt::cl::gather_if(svIn2Vec.begin(), svIn2Vec.end(), svIn3Vec.begin(), sv_trf_begin1, svOutVec.begin(), iepred);
            bolt::cl::gather_if(dvIn2Vec.begin(), dvIn2Vec.end(), dvIn3Vec.begin(), dv_trf_begin1, dvOutVec.begin(), iepred);
            /*Compute expected results*/
            Serial_gather_if(svIn2Vec.begin(), svIn2Vec.end(), svIn3Vec.begin(), sv_trf_begin1, stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

        {/*Test case when the first input is a randomAccessIterator  and second is a trf_itr */
            bolt::cl::gather_if(sv_trf_begin2, sv_trf_end2, sv_trf_begin3, svIn1Vec.begin(), svOutVec.begin(), iepred);
            bolt::cl::gather_if( dv_trf_begin2, dv_trf_end2, dv_trf_begin3, dvIn1Vec.begin(), dvOutVec.begin(), iepred);
            /*Compute expected results*/
            Serial_gather_if(sv_trf_begin2, sv_trf_end2, sv_trf_begin3, svIn1Vec.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

		{/*Test case when the first input is a randomAccessIterator  and second is a trf_itr */
            bolt::cl::gather_if(sv_trf_begin2, sv_trf_end2, svIn3Vec.begin(), svIn1Vec.begin(), svOutVec.begin(), iepred);
            bolt::cl::gather_if(dv_trf_begin2, dv_trf_end2, dvIn3Vec.begin(), dvIn1Vec.begin(), dvOutVec.begin(), iepred);
            /*Compute expected results*/
            Serial_gather_if(sv_trf_begin2, sv_trf_end2, svIn3Vec.begin(), svIn1Vec.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

		{/*Test case when the first input is trf_itr and the second is a counting iterator */
            bolt::cl::gather_if(count_itr_begin, count_itr_end, svIn3Vec.begin(), sv_trf_begin1, svOutVec.begin(), iepred);
            bolt::cl::gather_if(count_itr_begin, count_itr_end, dvIn3Vec.begin(), dv_trf_begin1, dvOutVec.begin(), iepred);
            /*Compute expected results*/
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;
            Serial_gather_if(count_vector.begin(), count_vector.end(), svIn3Vec.begin(), sv_trf_begin1, stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

		{/*Test case when the first input is trf_itr and the second is a counting iterator */
            bolt::cl::gather_if(count_itr_begin, count_itr_end, stencil_itr_begin, sv_trf_begin1, svOutVec.begin(), iepred);
            bolt::cl::gather_if(count_itr_begin, count_itr_end, stencil_itr_begin, dv_trf_begin1, dvOutVec.begin(), iepred);
            /*Compute expected results*/
            std::vector<int> count_vector(length), stencil_vector(length);
            for (int index=0;index<length;index++)
			{
                count_vector[index] = index;
				stencil_vector[index] = 1;
			}
            Serial_gather_if(count_vector.begin(), count_vector.end(), stencil_vector.begin(), sv_trf_begin1, stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }


        {/*Test case when the both are randomAccessIterator */
            bolt::cl::gather_if(svIn2Vec.begin(), svIn2Vec.end(), svIn3Vec.begin(), svIn1Vec.begin(), svOutVec.begin(), iepred);
            bolt::cl::gather_if(dvIn2Vec.begin(), dvIn2Vec.end(), dvIn3Vec.begin(), dvIn1Vec.begin(), dvOutVec.begin(), iepred);
            /*Compute expected results*/
            Serial_gather_if(svIn2Vec.begin(), svIn2Vec.end(), svIn3Vec.begin(), svIn1Vec.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }
       
		{/*Test case when the both are randomAccessIterator */
            bolt::cl::gather_if(svIn2Vec.begin(), svIn2Vec.end(), stencil_itr_begin, svIn1Vec.begin(), svOutVec.begin(), iepred);
            bolt::cl::gather_if(dvIn2Vec.begin(), dvIn2Vec.end(), stencil_itr_begin,  dvIn1Vec.begin(), dvOutVec.begin(), iepred);
            /*Compute expected results*/
			std::vector<int> stencil_vector(length);
            for (int index=0;index<length;index++)
			{
				stencil_vector[index] = 1;
			}
            Serial_gather_if(svIn2Vec.begin(), svIn2Vec.end(), stencil_vector.begin(), svIn1Vec.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

		{/*Test case when the first input is constant iterator and the second is a randomAccessIterator */
            bolt::cl::gather_if(svIn2Vec.begin(), svIn2Vec.end(), svIn3Vec.begin(), const_itr_begin, svOutVec.begin(), iepred);
            bolt::cl::gather_if(dvIn2Vec.begin(), dvIn2Vec.end(), dvIn3Vec.begin(), const_itr_begin, dvOutVec.begin(), iepred);
            /*Compute expected results*/
            std::vector<int> const_vector(length,1);          
            Serial_gather_if(svIn2Vec.begin(), svIn2Vec.end(), svIn3Vec.begin(), const_vector.begin(),stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

		{/*Test case when the first input is constant iterator and the second is a randomAccessIterator */
            bolt::cl::gather_if(svIn2Vec.begin(), svIn2Vec.end(), stencil_itr_begin, const_itr_begin, svOutVec.begin(), iepred);
            bolt::cl::gather_if(dvIn2Vec.begin(), dvIn2Vec.end(), stencil_itr_begin, const_itr_begin, dvOutVec.begin(), iepred);
            /*Compute expected results*/
            std::vector<int> const_vector(length,1); 
			std::vector<int> stencil_vector(length);
            for (int index=0;index<length;index++)
			{
				stencil_vector[index] = 1;
			}
            Serial_gather_if(svIn2Vec.begin(), svIn2Vec.end(), stencil_vector.begin(), const_vector.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }


        {/*Test case when the first input is constant iterator and the second is a counting iterator */
            bolt::cl::gather_if( count_itr_begin, count_itr_end, svIn3Vec.begin(), const_itr_begin, svOutVec.begin(), iepred);
            bolt::cl::gather_if( count_itr_begin, count_itr_end, dvIn3Vec.begin(), const_itr_begin, dvOutVec.begin(), iepred);
            /*Compute expected results*/
            std::vector<int> const_vector(length,1);
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;            
            Serial_gather_if(count_vector.begin(), count_vector.end(), svIn3Vec.begin(), const_vector.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

		{/*Test case when the first input is constant iterator and the second is a counting iterator */
            bolt::cl::gather_if(count_itr_begin, count_itr_end, stencil_itr_begin, const_itr_begin, svOutVec.begin(), iepred);
            bolt::cl::gather_if(count_itr_begin, count_itr_end, stencil_itr_begin, count_itr_begin, dvOutVec.begin(), iepred);
            /*Compute expected results*/
            std::vector<int> const_vector(length,1);
            std::vector<int> count_vector(length);
			std::vector<int> stencil_vector(length);
            for (int index=0;index<length;index++)
			{
				stencil_vector[index] = 1;
				count_vector[index] = index; 
			}
                           
            Serial_gather_if(count_vector.begin(), count_vector.end(), stencil_vector.begin(), const_vector.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }


		{/*Test case when the first input is a randomAccessIterator and the second is a counting iterator */
            bolt::cl::gather_if( count_itr_begin, count_itr_end, svIn3Vec.begin(), svIn1Vec.begin(), svOutVec.begin(), iepred);
            bolt::cl::gather_if( count_itr_begin, count_itr_end, dvIn3Vec.begin(), dvIn1Vec.begin(), dvOutVec.begin(), iepred);
            /*Compute expected results*/
            std::vector<int> count_vector(length);
            for (int index=0;index<length;index++)
                count_vector[index] = index;            
            Serial_gather_if(count_vector.begin(), count_vector.end(), svIn3Vec.begin(), svIn1Vec.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

		{/*Test case when the first input is a randomAccessIterator and the second is a counting iterator */
            bolt::cl::gather_if(count_itr_begin, count_itr_end, stencil_itr_begin, svIn1Vec.begin(), svOutVec.begin(), iepred);
            bolt::cl::gather_if(count_itr_begin, count_itr_end, stencil_itr_begin, dvIn1Vec.begin(), dvOutVec.begin(), iepred);
            /*Compute expected results*/
            std::vector<int> count_vector(length);
			std::vector<int> stencil_vector(length);
            for (int index=0;index<length;index++)
			{
				stencil_vector[index] = 1;
				count_vector[index] = index; 
			}
            Serial_gather_if(count_vector.begin(), count_vector.end(), stencil_vector.begin(), svIn1Vec.begin(), stlOut.begin(), iepred);
            /*Check the results*/
            cmpArrays(svOutVec, stlOut, length);
            cmpArrays(dvOutVec, stlOut, length);
        }

        global_id = 0; // Reset the global id counter
    }
 }


TEST( TransformIterator, ReduceByKeyRoutine)
{
    {
        const int length = 1<<10;
        std::vector< int > svIn1Vec( length );
        std::vector< int > svIn2Vec( length );
        std::vector< int > svOutVec1( length );
		std::vector< int > svOutVec2( length );
        std::vector< int > stlOut1( length );
		std::vector< int > stlOut2( length );
        bolt::BCKND::device_vector< int > dvIn1Vec( length );
        bolt::BCKND::device_vector< int > dvIn2Vec( length );
        bolt::BCKND::device_vector< int > dvOutVec1( length );
		bolt::BCKND::device_vector< int > dvOutVec2( length );

        add_3 add3;
        add_4 add4;
        bolt::cl::equal_to<int> binary_predictor;
        bolt::cl::plus<int> binary_operator;

        gen_input gen;

        typedef std::vector< int >::const_iterator                                                         sv_itr;
        typedef bolt::BCKND::device_vector< int >::iterator                                                dv_itr;
        typedef bolt::BCKND::counting_iterator< int >                                                      counting_itr;
        typedef bolt::BCKND::constant_iterator< int >                                                      constant_itr;
        typedef bolt::BCKND::transform_iterator< add_3, std::vector< int >::const_iterator>                sv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_3, bolt::BCKND::device_vector< int >::iterator>       dv_trf_itr_add3;
        typedef bolt::BCKND::transform_iterator< add_4, std::vector< int >::const_iterator>                sv_trf_itr_add4;
        typedef bolt::BCKND::transform_iterator< add_4, bolt::BCKND::device_vector< int >::iterator>       dv_trf_itr_add4;    
        /*Create Iterators*/
        sv_trf_itr_add3 sv_trf_begin1 (svIn1Vec.begin(), add3), sv_trf_end1 (svIn1Vec.end(), add3);
        sv_trf_itr_add4 sv_trf_begin2 (svIn2Vec.begin(), add4);
        dv_trf_itr_add3 dv_trf_begin1 (dvIn1Vec.begin(), add3), dv_trf_end1 (dvIn1Vec.end(), add3);
        dv_trf_itr_add4 dv_trf_begin2 (dvIn2Vec.begin(), add4);
        counting_itr count_itr_begin(0);
        counting_itr count_itr_end = count_itr_begin + length;
        constant_itr const_itr_begin(1);
        constant_itr const_itr_end = const_itr_begin + length;

        /*Generate inputs*/
        global_id = 0;
        std::generate(svIn1Vec.begin(), svIn1Vec.end(), gen);
        global_id = 0;
        std::generate(svIn2Vec.begin(), svIn2Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn1Vec.begin(), dvIn1Vec.end(), gen);
        global_id = 0;
        bolt::BCKND::generate(dvIn2Vec.begin(), dvIn2Vec.end(), gen);
        global_id = 0;

		std::vector< int > testInput1( svIn1Vec.begin(), svIn1Vec.end() );
		std::vector< int > testInput2( svIn2Vec.begin(), svIn2Vec.end() );
		for(int i=0; i<length; i++)
		{
			testInput1[i] = testInput1[i] + 3;
			testInput2[i] = testInput2[i] + 4;
		}

		std::vector< int > constVector(length, 1);
		std::vector< int > countVector(length);
		for(int i=0; i<length; i++)
		{
			countVector[i]=i;
		}

        {/*Test case when inputs are trf Iterators*/
            auto sv_result = bolt::cl::reduce_by_key(sv_trf_begin1, sv_trf_end1, sv_trf_begin2, svOutVec1.begin(), svOutVec2.begin(), binary_predictor, binary_operator);
            auto dv_result = bolt::cl::reduce_by_key(dv_trf_begin1, dv_trf_end1, dv_trf_begin2, dvOutVec1.begin(), dvOutVec2.begin(), binary_predictor, binary_operator);
            /*Compute expected results*/
            unsigned int n= Serial_reduce_by_key<int, int, int, int, bolt::cl::plus< int >> (&testInput1[0], &testInput2[0], &stlOut1[0], &stlOut2[0], binary_operator, length);
            /*Check the results*/
            cmpArrays(svOutVec1, stlOut1, length);
			cmpArrays(svOutVec2, stlOut2, length);
            cmpArrays(dvOutVec1, stlOut1, length);
			cmpArrays(dvOutVec2, stlOut2, length);
        }
        {/*Test case when the first input is trf_itr and the second is a randomAccessIterator */
            auto sv_result = bolt::cl::reduce_by_key(sv_trf_begin1, sv_trf_end1, svIn2Vec.begin(), svOutVec1.begin(), svOutVec2.begin(), binary_predictor, binary_operator);
            auto dv_result = bolt::cl::reduce_by_key(dv_trf_begin1, dv_trf_end1, dvIn2Vec.begin(), dvOutVec1.begin(), dvOutVec2.begin(), binary_predictor, binary_operator);
            /*Compute expected results*/
            unsigned int n= Serial_reduce_by_key<int, int, int, int, bolt::cl::plus< int >> (&testInput1[0], &svIn2Vec[0], &stlOut1[0], &stlOut2[0], binary_operator, length);
            /*Check the results*/
            cmpArrays(svOutVec1, stlOut1, length);
			cmpArrays(svOutVec2, stlOut2, length);
            cmpArrays(dvOutVec1, stlOut1, length);
			cmpArrays(dvOutVec2, stlOut2, length);
        }

		 {/*Test case when the first input is randomAccessIterator and the second is a trf_itr*/
            auto sv_result = bolt::cl::reduce_by_key(svIn1Vec.begin(), svIn1Vec.end(), sv_trf_begin2, svOutVec1.begin(), svOutVec2.begin(), binary_predictor, binary_operator);
            auto dv_result = bolt::cl::reduce_by_key(dvIn1Vec.begin(), dvIn1Vec.end(), dv_trf_begin2, dvOutVec1.begin(), dvOutVec2.begin(), binary_predictor, binary_operator);
            /*Compute expected results*/
            unsigned int n= Serial_reduce_by_key<int, int, int, int, bolt::cl::plus< int >> (&svIn1Vec[0], &testInput2[0], &stlOut1[0], &stlOut2[0], binary_operator, length);
            /*Check the results*/
            cmpArrays(svOutVec1, stlOut1, length);
			cmpArrays(svOutVec2, stlOut2, length);
            cmpArrays(dvOutVec1, stlOut1, length);
			cmpArrays(dvOutVec2, stlOut2, length);
        }

        {/*Test case when the first input is trf_itr and the second is a constant iterator */
            auto sv_result = bolt::cl::reduce_by_key(sv_trf_begin1, sv_trf_end1, const_itr_begin, svOutVec1.begin(), svOutVec2.begin(), binary_predictor, binary_operator);
            auto dv_result = bolt::cl::reduce_by_key(dv_trf_begin1, dv_trf_end1, const_itr_begin, dvOutVec1.begin(), dvOutVec2.begin(), binary_predictor, binary_operator);
            /*Compute expected results*/
            unsigned int n= Serial_reduce_by_key<int, int, int, int, bolt::cl::plus< int >> (&testInput1[0], &constVector[0], &stlOut1[0], &stlOut2[0], binary_operator, length);
            /*Check the results*/
            cmpArrays(svOutVec1, stlOut1, length);
			cmpArrays(svOutVec2, stlOut2, length);
            cmpArrays(dvOutVec1, stlOut1, length);
			cmpArrays(dvOutVec2, stlOut2, length);
        }

		{/*Test case when the first input is constant iterator and the second is a  trf_itr */
            auto sv_result = bolt::cl::reduce_by_key(const_itr_begin, const_itr_end, sv_trf_begin2, svOutVec1.begin(), svOutVec2.begin(), binary_predictor, binary_operator);
            auto dv_result = bolt::cl::reduce_by_key(const_itr_begin, const_itr_end, dv_trf_begin2, dvOutVec1.begin(), dvOutVec2.begin(), binary_predictor, binary_operator);
            /*Compute expected results*/
            unsigned int n= Serial_reduce_by_key<int, int, int, int, bolt::cl::plus< int >> (&constVector[0], &testInput2[0], &stlOut1[0], &stlOut2[0], binary_operator, length);
            /*Check the results*/
            cmpArrays(svOutVec1, stlOut1, length);
			cmpArrays(svOutVec2, stlOut2, length);
            cmpArrays(dvOutVec1, stlOut1, length);
			cmpArrays(dvOutVec2, stlOut2, length);
        }

        {/*Test case when the first input is trf_itr and the second is a counting iterator */      
			auto sv_result = bolt::cl::reduce_by_key(sv_trf_begin1, sv_trf_end1, count_itr_begin, svOutVec1.begin(), svOutVec2.begin(), binary_predictor, binary_operator);
            auto dv_result = bolt::cl::reduce_by_key(dv_trf_begin1, dv_trf_end1, count_itr_begin, dvOutVec1.begin(), dvOutVec2.begin(), binary_predictor, binary_operator);
            /*Compute expected results*/
            unsigned int n= Serial_reduce_by_key<int, int, int, int, bolt::cl::plus< int >> (&testInput1[0], &countVector[0], &stlOut1[0], &stlOut2[0], binary_operator, length);
            /*Check the results*/
            cmpArrays(svOutVec1, stlOut1, length);
			cmpArrays(svOutVec2, stlOut2, length);
            cmpArrays(dvOutVec1, stlOut1, length);
			cmpArrays(dvOutVec2, stlOut2, length);
        }

		{/*Test case when the first input is counting iterator and the second is a trf_itr */
            auto sv_result = bolt::cl::reduce_by_key(count_itr_begin, count_itr_end, sv_trf_begin2, svOutVec1.begin(), svOutVec2.begin(), binary_predictor, binary_operator);
            auto dv_result = bolt::cl::reduce_by_key(count_itr_begin, count_itr_end, dv_trf_begin2, dvOutVec1.begin(), dvOutVec2.begin(), binary_predictor, binary_operator);
            /*Compute expected results*/
            unsigned int n= Serial_reduce_by_key<int, int, int, int, bolt::cl::plus< int >> (&countVector[0], &testInput2[0], &stlOut1[0], &stlOut2[0], binary_operator, length);
            /*Check the results*/
            cmpArrays(svOutVec1, stlOut1, length);
			cmpArrays(svOutVec2, stlOut2, length);
            cmpArrays(dvOutVec1, stlOut1, length);
			cmpArrays(dvOutVec2, stlOut2, length);
        }


		 {/*Test case when the both inputs are randomAccessIterators*/
            auto sv_result = bolt::cl::reduce_by_key(svIn1Vec.begin(), svIn1Vec.end(), svIn2Vec.begin(), svOutVec1.begin(), svOutVec2.begin(), binary_predictor, binary_operator);
            auto dv_result = bolt::cl::reduce_by_key(dvIn1Vec.begin(), dvIn1Vec.end(), dvIn2Vec.begin(), dvOutVec1.begin(), dvOutVec2.begin(), binary_predictor, binary_operator);
            /*Compute expected results*/
            unsigned int n= Serial_reduce_by_key<int, int, int, int, bolt::cl::plus< int >> (&svIn1Vec[0], &svIn2Vec[0], &stlOut1[0], &stlOut2[0], binary_operator, length);
            /*Check the results*/
            cmpArrays(svOutVec1, stlOut1, length);
			cmpArrays(svOutVec2, stlOut2, length);
            cmpArrays(dvOutVec1, stlOut1, length);
			cmpArrays(dvOutVec2, stlOut2, length);
        }

		{/*Test case when the first input is constant iterator and the second is a counting iterator */
            auto sv_result = bolt::cl::reduce_by_key(const_itr_begin, const_itr_end, count_itr_begin, svOutVec1.begin(), svOutVec2.begin(), binary_predictor, binary_operator);
            auto dv_result = bolt::cl::reduce_by_key(const_itr_begin, const_itr_end, count_itr_begin, dvOutVec1.begin(), dvOutVec2.begin(), binary_predictor, binary_operator);
            /*Compute expected results*/
            unsigned int n= Serial_reduce_by_key<int, int, int, int, bolt::cl::plus< int >> (&constVector[0], &countVector[0], &stlOut1[0], &stlOut2[0], binary_operator, length);
            /*Check the results*/
            cmpArrays(svOutVec1, stlOut1, length);
			cmpArrays(svOutVec2, stlOut2, length);
            cmpArrays(dvOutVec1, stlOut1, length);
			cmpArrays(dvOutVec2, stlOut2, length);
        }

		 {/*Test case when the first input is counting iterator and the second is a constant iterator */
            auto sv_result = bolt::cl::reduce_by_key(count_itr_begin, count_itr_end, const_itr_begin, svOutVec1.begin(), svOutVec2.begin(), binary_predictor, binary_operator);
            auto dv_result = bolt::cl::reduce_by_key(count_itr_begin, count_itr_end, const_itr_begin, dvOutVec1.begin(), dvOutVec2.begin(), binary_predictor, binary_operator);
            /*Compute expected results*/
            unsigned int n= Serial_reduce_by_key<int, int, int, int, bolt::cl::plus< int >> (&countVector[0], &constVector[0], &stlOut1[0], &stlOut2[0], binary_operator, length);
            /*Check the results*/
            cmpArrays(svOutVec1, stlOut1, length);
			cmpArrays(svOutVec2, stlOut2, length);
            cmpArrays(dvOutVec1, stlOut1, length);
			cmpArrays(dvOutVec2, stlOut2, length);
        }

        global_id = 0; // Reset the global id counter
    }
}


/* /brief List of possible tests
 * Two input transform with first input a constant iterator
 * One input transform with a constant iterator
*/
int _tmain(int argc, _TCHAR* argv[])
{
    //  Register our minidump generating logic
    //bolt::miniDumpSingleton::enableMiniDumps( );

    //  Initialize googletest; this removes googletest specific flags from command line
    ::testing::InitGoogleTest( &argc, &argv[ 0 ] );

    bool print_clInfo = false;
    cl_uint userPlatform = 0;
    cl_uint userDevice = 0;
    cl_device_type deviceType = CL_DEVICE_TYPE_DEFAULT;

    try
    {
        // Declare supported options below, describe what they do
        po::options_description desc( "Scan GoogleTest command line options" );
        desc.add_options()
            ( "help,h",         "produces this help message" )
            ( "queryOpenCL,q",  "Print queryable platform and device info and return" )
            ( "platform,p",     po::value< cl_uint >( &userPlatform )->default_value( 0 ),    
            "Specify the platform under test" )
            ( "device,d",       po::value< cl_uint >( &userDevice )->default_value( 0 ),    
            "Specify the device under test" )
            //( "gpu,g",         "Force instantiation of all OpenCL GPU device" )
            //( "cpu,c",         "Force instantiation of all OpenCL CPU device" )
            //( "all,a",         "Force instantiation of all OpenCL devices" )
            ;

        ////  All positional options (un-named) should be interpreted as kernelFiles
        //po::positional_options_description p;
        //p.add("kernelFiles", -1);

        //po::variables_map vm;
        //po::store( po::command_line_parser( argc, argv ).options( desc ).positional( p ).run( ), vm );
        //po::notify( vm );

        po::variables_map vm;
        po::store( po::parse_command_line( argc, argv, desc ), vm );
        po::notify( vm );

        if( vm.count( "help" ) )
        {
            //    This needs to be 'cout' as program-options does not support wcout yet
            std::cout << desc << std::endl;
            return 0;
        }

        if( vm.count( "queryOpenCL" ) )
        {
            print_clInfo = true;
        }

        //  The following 3 options are not implemented yet; they are meant to be used with ::clCreateContextFromType()
        if( vm.count( "gpu" ) )
        {
            deviceType    = CL_DEVICE_TYPE_GPU;
        }

        if( vm.count( "cpu" ) )
        {
            deviceType    = CL_DEVICE_TYPE_CPU;
        }

        if( vm.count( "all" ) )
        {
            deviceType    = CL_DEVICE_TYPE_ALL;
        }

    }
    catch( std::exception& e )
    {
        std::cout << _T( "Scan GoogleTest error condition reported:" ) << std::endl << e.what() << std::endl;
        return 1;
    }

    //  Query OpenCL for available platforms
    cl_int err = CL_SUCCESS;

    // Platform vector contains all available platforms on system
    std::vector< cl::Platform > platforms;
    //std::cout << "HelloCL!\nGetting Platform Information\n";
    bolt::cl::V_OPENCL( cl::Platform::get( &platforms ), "Platform::get() failed" );

    if( print_clInfo )
    {
        bolt::cl::control::printPlatforms( );
        return 0;
    }

    //  Do stuff with the platforms
    std::vector<cl::Platform>::iterator i;
    if(platforms.size() > 0)
    {
        for(i = platforms.begin(); i != platforms.end(); ++i)
        {
            if(!strcmp((*i).getInfo<CL_PLATFORM_VENDOR>(&err).c_str(), "Advanced Micro Devices, Inc."))
            {
                break;
            }
        }
    }
    bolt::cl::V_OPENCL( err, "Platform::getInfo() failed" );

    // Device info
    std::vector< cl::Device > devices;
    bolt::cl::V_OPENCL( platforms.front( ).getDevices( CL_DEVICE_TYPE_ALL, &devices ),"Platform::getDevices()failed" );

    cl::Context myContext( devices.at( userDevice ) );
    cl::CommandQueue myQueue( myContext, devices.at( userDevice ) );
    bolt::cl::control::getDefault( ).setCommandQueue( myQueue );

    std::string strDeviceName = bolt::cl::control::getDefault( ).getDevice( ).getInfo< CL_DEVICE_NAME >( &err );
    bolt::cl::V_OPENCL( err, "Device::getInfo< CL_DEVICE_NAME > failed" );

    std::cout << "Device under test : " << strDeviceName << std::endl;

    int retVal = RUN_ALL_TESTS( );

    //  Reflection code to inspect how many tests failed in gTest
    ::testing::UnitTest& unitTest = *::testing::UnitTest::GetInstance( );

    unsigned int failedTests = 0;
    for( int i = 0; i < unitTest.total_test_case_count( ); ++i )
    {
        const ::testing::TestCase& testCase = *unitTest.GetTestCase( i );
        for( int j = 0; j < testCase.total_test_count( ); ++j )
        {
            const ::testing::TestInfo& testInfo = *testCase.GetTestInfo( j );
            if( testInfo.result( )->Failed( ) )
                ++failedTests;
        }
    }

    //  Print helpful message at termination if we detect errors, to help users figure out what to do next
    if( failedTests )
    {
        bolt::tout << _T( "\nFailed tests detected in test pass; please run test again with:" ) << std::endl;
        bolt::tout << _T( "\t--gtest_filter=<XXX> to select a specific failing test of interest" ) << std::endl;
        bolt::tout << _T( "\t--gtest_catch_exceptions=0 to generate minidump of failing test, or" ) << std::endl;
        bolt::tout << _T( "\t--gtest_break_on_failure to debug interactively with debugger" ) << std::endl;
        bolt::tout << _T( "\t    (only on googletest assertion failures, not SEH exceptions)" ) << std::endl;
    }

    return retVal;
}
