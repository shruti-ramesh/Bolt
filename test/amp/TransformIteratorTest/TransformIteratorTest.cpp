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

#include "common/stdafx.h"
#include "bolt/amp/transform.h"
#include "bolt/unicode.h"
#include "bolt/miniDump.h"
#include <gtest/gtest.h>
#include <array>

#include "bolt/amp/copy.h"
#include "bolt/amp/count.h"
#include "bolt/amp/fill.h"
#include "bolt/amp/functional.h"
#include "bolt/amp/gather.h"
#include "bolt/amp/inner_product.h"
#include "bolt/amp/transform.h"
#include "bolt/amp/reduce.h"
#include "bolt/amp/reduce_by_key.h"
#include "bolt/amp/scan.h"
#include "bolt/amp/scan_by_key.h"
#include "bolt/amp/scatter.h"
#include "bolt/amp/transform_reduce.h"
#include "bolt/amp/transform_scan.h"

#include "common/test_common.h"
#include <bolt/amp/iterator/constant_iterator.h>
#include <bolt/amp/iterator/counting_iterator.h>
#include <bolt/amp/iterator/permutation_iterator.h>
#include <bolt/amp/iterator/transform_iterator.h>

#include <boost/shared_array.hpp>
#include <amp_math.h>

#include <boost/program_options.hpp>
#include <boost/iterator/permutation_iterator.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/algorithm/cxx11/iota.hpp>
namespace po = boost::program_options;

#define WAVEFRNT_SIZE 256
static const unsigned int size = WAVEFRNT_SIZE;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Gold helper algorithms
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace gold
{
        template<
        typename InputIterator1,
        typename InputIterator2,
        typename OutputIterator,
        typename BinaryFunction>
    OutputIterator
    scan_by_key(
        InputIterator1 firstKey,
        InputIterator1 lastKey,
        InputIterator2 values,
        OutputIterator result,
        BinaryFunction binary_op)
    {
        if(std::distance(firstKey,lastKey) < 1)
             return result;
        typedef typename std::iterator_traits< InputIterator1 >::value_type kType;
        typedef typename std::iterator_traits< InputIterator2 >::value_type vType;
        typedef typename std::iterator_traits< OutputIterator >::value_type oType;

        static_assert( std::is_convertible< vType, oType >::value,
            "InputIterator2 and OutputIterator's value types are not convertible." );

        if(std::distance(firstKey,lastKey) < 1)
             return result;
        // do zeroeth element
        *result = *values; // assign value

        // scan oneth element and beyond
        for ( InputIterator1 key = (firstKey+1); key != lastKey; key++)
        {
            // move on to next element
            values++;
            result++;

            // load keys
            kType currentKey  = *(key);
            kType previousKey = *(key-1);

            // load value
            oType currentValue = *values; // convertible
            oType previousValue = *(result-1);

            // within segment
            if (currentKey == previousKey)
            {
                //std::cout << "continuing segment" << std::endl;
                oType r = binary_op( previousValue, currentValue);
                *result = r;
            }
            else // new segment
            {
                //std::cout << "new segment" << std::endl;
                *result = currentValue;
            }
        }

        return result;
    }

    template<
        typename InputIterator1,
        typename InputIterator2,
        typename OutputIterator1,
        typename OutputIterator2,
        typename BinaryPredicate,
        typename BinaryFunction>
    //std::pair<OutputIterator1, OutputIterator2>
    unsigned int
    reduce_by_key( InputIterator1 keys_first,
                   InputIterator1 keys_last,
                   InputIterator2 values_first,
                   OutputIterator1 keys_output,
                   OutputIterator2 values_output,
                   const BinaryPredicate binary_pred,
                   const BinaryFunction binary_op )
    {
        typedef typename std::iterator_traits< InputIterator1 >::value_type kType;
        typedef typename std::iterator_traits< InputIterator2 >::value_type vType;
        typedef typename std::iterator_traits< OutputIterator1 >::value_type koType;
        typedef typename std::iterator_traits< OutputIterator2 >::value_type voType;
        static_assert( std::is_convertible< vType, voType >::value,
                       "InputIterator2 and OutputIterator's value types are not convertible." );

       int numElements = static_cast< int >( std::distance( keys_first, keys_last ) );

        // do zeroeth element
        *values_output = *values_first;
        *keys_output = *keys_first;
        unsigned int count = 1;
        // rbk oneth element and beyond

        values_first++;
        for ( InputIterator1 key = (keys_first+1); key != keys_last; key++)
        {
            // load keys
            kType currentKey  = *(key);
            kType previousKey = *(key-1);

            // load value
            voType currentValue = *values_first;
            voType previousValue = *values_output;

            previousValue = *values_output;
            // within segment
            if (binary_pred(currentKey, previousKey))
            {
                voType r = binary_op( previousValue, currentValue);
                *values_output = r;
                *keys_output = currentKey;

            }
            else // new segment
            {
                values_output++;
                keys_output++;
                *values_output = currentValue;
                *keys_output = currentKey;
                count++; //To count the number of elements in the output array
            }
            values_first++;
        }

        //return std::pair(keys_output+1, values_output+1);
        return count;
    }

};



///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Boost tests
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct square_root
{
    float operator( ) (float x) const restrict ( cpu, amp )
    {
      return concurrency::fast_math::sqrt(x);
    }
    typedef float result_type;
};


struct my_negate
{
    int operator( ) (int x) const restrict (cpu,amp)
    {
      return -x;
    }
    typedef int result_type;
};



TEST(TransformIterator, boost)
{

    typedef float etype;
    typedef std::vector<etype> fev;

    etype elements[10] = {0,1,2,3,4,5,6,7,8,9};
    fev e( elements, elements + 10 );
    fev r( 10 );
    size_t esize = sizeof(elements)/sizeof(etype);

    std::transform( boost::make_transform_iterator( e.begin( ), square_root( ) ),
                    boost::make_transform_iterator( e.end( ),   square_root( ) ),
                    r.begin( ),
                    std::negate<float>( )
                  );

    float check[10] = { -0.000000000f,
                        -1.00000000f,
                        -1.41421354f,
                        -1.73205078f,
                        -2.00000000f,
                        -2.23606801f,
                        -2.44948983f,
                        -2.64575124f,
                        -2.82842708f,
                        -3.00000000f };
    fev c( check, check + 10 );

    cmpArrays(c, r);
}


template<typename T>
struct multby2
{
    T operator( ) (T x) const 
    {
      return x*T(2);
    }
    typedef T result_type;
};

template<typename T>
struct add4
{
    T operator( ) (T x) const restrict ( cpu, amp )
    {
      return x+T(4);
    }
    typedef T result_type;
};

//Boost test
TEST( TransformIterator, BoostBolt )
{
    int x[] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    const int N = sizeof(x)/sizeof(int);

    bolt::amp::device_vector<int> dx(x, x+N);
    bolt::amp::device_vector<int> dout(x, x+N, true);
    typedef  multby2<int> Function;
    typedef bolt::amp::transform_iterator<Function, bolt::amp::device_vector<int>::iterator> doubling_iterator;

    doubling_iterator i( dx.begin( ), multby2< int >( ) ),
      i_end( dx.end( ), multby2< int >( ) );

    std::cout << "multiplying the array by 2:" << std::endl;
    while (i != i_end)
      std::cout << *i++ << " ";
    std::cout << std::endl;

    std::cout << "adding 4 to each element in the array:" << std::endl;
    bolt::amp::copy( bolt::amp::make_transform_iterator( dx.begin( ), add4< int >( ) ),
                     bolt::amp::make_transform_iterator( dx.end( ), add4< int >( ) ),
                     dout.begin( ) );
    bolt::amp::device_vector< int >::iterator di = dout.begin( );
    while( di != dout.end( ) )
      std::cout << *di++ << " ";
    std::cout<<std::endl;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Transform tests
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct UDD
{
    int a; 
    int b;

    operator UDD( ) { return UDD( ); }

    UDD operator( ) (const UDD& lhs, const UDD& rhs) const restrict(amp,cpu){
        return (rhs);
    } 

    //UDD operator( ) ( const UDD& lhs ) const restrict(amp,cpu){
    //    return lhs;
    //} 

    bool operator < (const UDD& other) const restrict(amp,cpu){
        return ((a+b) < (other.a+other.b));
    }
    bool operator > (const UDD& other) const restrict(amp,cpu){
        return ((a+b) > (other.a+other.b));
    }
    bool operator == (const UDD& other) const restrict(amp,cpu) {
        return ((a+b) == (other.a+other.b));
    }

    UDD operator + (const UDD &rhs) const restrict(amp,cpu)
    {
      UDD _result;
      _result.a = a + rhs.a;
      _result.b = b + rhs.b;
      return _result;
    }


    UDD( ) restrict(amp,cpu)
        : a(0),b(0) { }
    UDD(int _in) restrict(amp,cpu)
        : a(_in), b(_in +1)  { }
    typedef UDD result_type;
};

struct UDDnegate
{
   UDD operator( ) (const UDD &lhs) const restrict ( cpu, amp )
   {
     UDD _result;
     _result.a = -lhs.a;
     _result.b = -lhs.b;
     return _result;
   }
   typedef UDD result_type;
};

TEST(TransformIterator, Transform)
{

    typedef int etype;
    etype elements[WAVEFRNT_SIZE];
    etype empty[WAVEFRNT_SIZE];

    size_t view_size = WAVEFRNT_SIZE;

    std::iota(elements, elements+WAVEFRNT_SIZE, 1000);
    std::fill(empty, empty+WAVEFRNT_SIZE, 0);

    bolt::amp::device_vector<int, concurrency::array_view> dve(elements, elements + WAVEFRNT_SIZE);
    bolt::amp::device_vector<int, concurrency::array_view> dumpV(empty, empty + WAVEFRNT_SIZE);
    bolt::amp::device_vector<int, concurrency::array_view> dumpCheckV(empty, empty + WAVEFRNT_SIZE);

    std::vector<int> el(elements, elements+WAVEFRNT_SIZE);
    std::vector<int> check(empty, empty + WAVEFRNT_SIZE);

    auto dvebegin = dve.begin( );
    auto dveend = dve.end( );

    bolt::amp::control ctl;
    concurrency::accelerator_view av = ctl.getAccelerator( ).default_view;

    typedef bolt::amp::transform_iterator<my_negate, bolt::amp::device_vector<int>::iterator> transf_iter;

    transf_iter tbegin(dvebegin, my_negate( ));
    transf_iter tend(dveend, my_negate( ));

    std::transform( boost::make_transform_iterator(el.begin( ), my_negate( )), 
                    boost::make_transform_iterator(el.end( ), my_negate( )),
                    check.begin( ),
                    std::identity<int>( )
                    );

    bolt::amp::transform( bolt::amp::make_transform_iterator(dvebegin, my_negate( )), 
                          bolt::amp::make_transform_iterator(dveend, my_negate( )),
                          dumpV.begin( ),
                          bolt::amp::identity<int>( )
                          );

    cmpArrays(check,dumpV);

}

TEST(TransformIterator, TransformUDD)
{

    typedef UDD etype;
    etype elements[WAVEFRNT_SIZE];
    etype empty[WAVEFRNT_SIZE];

    size_t view_size = WAVEFRNT_SIZE;

    std::iota(elements, elements+WAVEFRNT_SIZE, 1000);
    std::fill(empty, empty+WAVEFRNT_SIZE, 0);

    bolt::amp::device_vector<UDD, concurrency::array_view> dve(elements, elements + WAVEFRNT_SIZE);
    bolt::amp::device_vector<UDD, concurrency::array_view> dumpV(empty, empty + WAVEFRNT_SIZE);

    std::vector<UDD> el(elements, elements+WAVEFRNT_SIZE);
    std::vector<UDD> check(empty, empty + WAVEFRNT_SIZE);

    auto dvebegin = dve.begin( );
    auto dveend = dve.end( );

    std::transform( boost::make_transform_iterator(el.begin( ), UDDnegate( ) ), 
                    boost::make_transform_iterator(el.end( ), UDDnegate( ) ),
                    check.begin( ),
                    std::identity<UDD>( )
                    );

    bolt::amp::transform( bolt::amp::make_transform_iterator(dvebegin, UDDnegate( )), 
                          bolt::amp::make_transform_iterator(dveend, UDDnegate( )),
                          dumpV.begin( ),
                          bolt::amp::identity<UDD>( )
                          );

    cmpArrays(check,dumpV);

}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Reduce tests
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST(TransformIterator, Reduce)
{
    size_t view_size = WAVEFRNT_SIZE;

    typedef int etype;
    etype elements[WAVEFRNT_SIZE];
    etype e2[WAVEFRNT_SIZE];



    std::iota(elements, elements + WAVEFRNT_SIZE, 1000);
    std::fill( e2, e2 + WAVEFRNT_SIZE, 100 );

    bolt::amp::device_vector<int, concurrency::array_view> dve(elements, elements + WAVEFRNT_SIZE);
    bolt::amp::device_vector<int, concurrency::array_view> dve2(e2, e2 + WAVEFRNT_SIZE);

    std::vector<int> el(elements, elements + WAVEFRNT_SIZE);
    std::vector<int> el2(e2, e2 + WAVEFRNT_SIZE);

    auto dvebegin = dve.begin( );
    auto dveend = dve.end( );

    bolt::amp::control ctl;
    concurrency::accelerator_view av = ctl.getAccelerator( ).default_view;

    typedef bolt::amp::transform_iterator<my_negate, bolt::amp::device_vector<int>::iterator> transf_iter;

    transf_iter tbegin(dvebegin, my_negate( ));
    transf_iter tend(dveend, my_negate( ));

    int out = std::accumulate( boost::make_transform_iterator(el.begin( ),  my_negate( )), 
                               boost::make_transform_iterator(el.end( ),  my_negate( )),
                               0,
                               std::plus<int>( )
                             );

    int bolt_out = bolt::amp::reduce( bolt::amp::make_transform_iterator(dvebegin, my_negate( )), 
                                      bolt::amp::make_transform_iterator(dveend, my_negate( )),
                                      0,
                                      bolt::amp::plus<int>( )
                                     );
    EXPECT_EQ( out, bolt_out );

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TransformReduce tests
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST(TransformIterator, TransformReduce)
{

    typedef int etype;
    etype elements[WAVEFRNT_SIZE];
    etype empty[WAVEFRNT_SIZE];

    size_t view_size = WAVEFRNT_SIZE;

    std::iota(elements, elements+WAVEFRNT_SIZE, 1000);
    std::fill(empty, empty+WAVEFRNT_SIZE, 0);

    bolt::amp::device_vector<int, concurrency::array_view> dve(elements, elements + WAVEFRNT_SIZE);

    std::vector<int> el(elements, elements+WAVEFRNT_SIZE);
    std::vector<int> check(empty, empty + WAVEFRNT_SIZE);

    auto dvebegin = dve.begin( );
    auto dveend = dve.end( );

    std::transform( boost::make_transform_iterator(el.begin( ), my_negate( )), 
                    boost::make_transform_iterator(el.end( ), my_negate( )),
                    check.begin( ),
                    std::identity<int>( )
                    );
    int acc = std::accumulate( check.begin( ), check.end( ), 0, std::plus< int >( ) );

    int bolt_acc = bolt::amp::transform_reduce( bolt::amp::make_transform_iterator(dvebegin, my_negate( )), 
                                                bolt::amp::make_transform_iterator(dveend, my_negate( )),
                                                bolt::amp::identity<int>( ),
                                                0,
                                                bolt::amp::plus<int>( )
                                              );

    EXPECT_EQ( acc, bolt_acc );

}

TEST(TransformIterator, TransformReduceFloat)
{

    typedef float etype;
    etype elements[WAVEFRNT_SIZE];
    etype empty[WAVEFRNT_SIZE];

    size_t view_size = WAVEFRNT_SIZE;

    std::iota(elements, elements+WAVEFRNT_SIZE, 1000.0f);
    std::fill(empty, empty+WAVEFRNT_SIZE, 0.0f);

    bolt::amp::device_vector<float, concurrency::array_view> dve(elements, elements + WAVEFRNT_SIZE);

    std::vector<float> el(elements, elements+WAVEFRNT_SIZE);
    std::vector<float> check(empty, empty + WAVEFRNT_SIZE);

    auto dvebegin = dve.begin( );
    auto dveend = dve.end( );

    std::transform( boost::make_transform_iterator( el.begin( ), square_root( ) ), 
                    boost::make_transform_iterator( el.end( ), square_root( )),
                    check.begin( ),
                    std::identity< float >( )
                    );
    float acc = std::accumulate( check.begin( ), check.end( ), 0.0f, std::plus< float >( ) );

    float bolt_acc = bolt::amp::transform_reduce( bolt::amp::make_transform_iterator(dvebegin, square_root( )), 
                                                  bolt::amp::make_transform_iterator(dveend, square_root( )),
                                                  bolt::amp::identity< float >( ),
                                                  0.0f,
                                                  bolt::amp::plus< float >( )
                                                );

    EXPECT_FLOAT_EQ( acc, bolt_acc );

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Inner Product tests
///////////////////////////////////////////////////////////////////////////////////////////////////////////////


TEST(TransformIterator, InnerProduct)
{

    typedef int etype;
    etype elements[WAVEFRNT_SIZE];
    etype elements2[WAVEFRNT_SIZE];
    etype empty[WAVEFRNT_SIZE];

    size_t view_size = WAVEFRNT_SIZE;

    std::iota(elements, elements+WAVEFRNT_SIZE, 10);
    std::iota(elements2, elements2+WAVEFRNT_SIZE, 99);
    std::fill(empty, empty+WAVEFRNT_SIZE, 0);

    bolt::amp::device_vector<int, concurrency::array_view> dve(elements, elements + WAVEFRNT_SIZE);
    bolt::amp::device_vector<int, concurrency::array_view> dve2(elements2, elements2 + WAVEFRNT_SIZE);

    std::vector<int> el(elements, elements+WAVEFRNT_SIZE);
    std::vector<int> el2(elements2, elements2+WAVEFRNT_SIZE);
    std::vector<int> check(empty, empty + WAVEFRNT_SIZE);

    auto dvebegin = dve.begin( );
    auto dveend = dve.end( );
    auto dve2begin = dve.begin( );

    std::transform( boost::make_transform_iterator(el.begin( ), my_negate( )), 
                    boost::make_transform_iterator(el.end( ), my_negate( )),
                    boost::make_transform_iterator(el2.begin( ), my_negate( )), 
                    check.begin( ),
                    std::plus<int>( )
                    );
    int acc = std::accumulate( check.begin( ), check.end( ), 0, std::plus< int >( ) );

    //int bolt_acc = bolt::amp::inner_product( bolt::amp::make_transform_iterator(dvebegin, my_negate( )), 
    //                                         bolt::amp::make_transform_iterator(dveend, my_negate( )),
    //                                         bolt::amp::make_transform_iterator(dve2begin, my_negate( )),
    //                                         0,
    //                                         bolt::amp::multiplies<int>( ),
    //                                         bolt::amp::plus<int>( )
    //                                        );

    int bolt_acc = bolt::amp::inner_product( dvebegin , 
                                             dveend   ,
                                             dve2begin,
                                             0,
                                             bolt::amp::plus<int>( ),
                                             bolt::amp::plus<int>( )
                                            );

    std::cout<<"Number = "<<bolt_acc;
    EXPECT_EQ( acc, bolt_acc );

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Copy tests
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST(TransformIterator, Copy)
{

    typedef int etype;
    etype elements[WAVEFRNT_SIZE];
    etype empty[WAVEFRNT_SIZE];

    size_t view_size = WAVEFRNT_SIZE;

    std::iota(elements, elements+WAVEFRNT_SIZE, 1000);
    std::fill(empty, empty+WAVEFRNT_SIZE, 0);

    bolt::amp::device_vector<int, concurrency::array_view> dve(elements, elements + WAVEFRNT_SIZE);
    bolt::amp::device_vector<int, concurrency::array_view> dumpV(empty, empty + WAVEFRNT_SIZE);

    std::vector<int> el(elements, elements+WAVEFRNT_SIZE);
    std::vector<int> check(empty, empty + WAVEFRNT_SIZE);

    auto dvebegin = dve.begin( );
    auto dveend = dve.end( );

    std::copy( boost::make_transform_iterator(el.begin( ), my_negate( )), 
               boost::make_transform_iterator(el.end( ), my_negate( )),
               check.begin( )
               );

    bolt::amp::copy( bolt::amp::make_transform_iterator(dvebegin, my_negate( )), 
                     bolt::amp::make_transform_iterator(dveend, my_negate( )),
                     dumpV.begin( )
                   );

    cmpArrays(check,dumpV);

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Count tests
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST (TransformIterator, CountIfTest)
{
    int aSize = 1024;
    std::vector<int> A(aSize);

    for (int i=0; i < aSize; i++) {
        A[i] = rand( ) % 10 + 1;
    }
    bolt::amp::device_vector< int > dA(A.begin( ), aSize);
    int intVal = 1;
    
    int stdInRangeCount =  static_cast<int>( std::count(
         boost::make_transform_iterator( A.begin( ), my_negate( ) ),
         boost::make_transform_iterator( A.end( ), my_negate( ) ),
         intVal ) );
    int boltInRangeCount = static_cast<int>( bolt::amp::count(
         bolt::amp::make_transform_iterator( dA.begin( ), my_negate( ) ),
         bolt::amp::make_transform_iterator( dA.end( ), my_negate( ) ),
         intVal ) );

    EXPECT_EQ(stdInRangeCount, boltInRangeCount);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Scatter tests
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct is_even{
    bool operator ( ) (int x) const restrict(cpu,amp)
    {
        return ( (x % 2)==0);
    }
};

TEST( TransformIterator, ScatterIf )
{
    int n_input[10] =  {-11,-1,-2,-3,-4,-5,-6,-7,-8,-9};
    int n_map[10] =  {9,8,7,6,5,4,3,2,1,0};
    int n_stencil[10] =  {0,1,0,1,0,1,0,1,0,1};

    std::vector<int> exp_result;
    {
        exp_result.push_back(-1);exp_result.push_back(8);
        exp_result.push_back(-1);exp_result.push_back(6);
        exp_result.push_back(-1);exp_result.push_back(4);
        exp_result.push_back(-1);exp_result.push_back(2);
        exp_result.push_back(-1);exp_result.push_back(11);
    }
    bolt::amp::device_vector<int> result ( 10, -1 );
    bolt::amp::device_vector<int> input ( n_input, n_input + 10 );
    bolt::amp::device_vector<int> map ( n_map, n_map + 10 );
    bolt::amp::device_vector<int> stencil ( n_stencil, n_stencil + 10 );

    
    is_even iepred;
    bolt::amp::scatter_if( bolt::amp::make_transform_iterator( input.begin( ), my_negate( ) ),
                           bolt::amp::make_transform_iterator( input.end( ), my_negate( ) ),
                           map.begin( ),
                           stencil.begin( ),
                           result.begin( ),
                           iepred );

    cmpArrays( exp_result, result );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Gather tests
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST( TransformIterator, GatherIf )
{
    int n_map[10]     =  {0,1,2,3,4,5,6,7,8,9};
    int n_input[10]   =  {-9,-8,-7,-6,-5,-4,-3,-2,-1,-0};
    int n_stencil[10] =  {0,1,0,1,0,1,0,1,0,1};

    std::vector<int> exp_result;
    {
        exp_result.push_back(9);exp_result.push_back(-1);
        exp_result.push_back(7);exp_result.push_back(-1);
        exp_result.push_back(5);exp_result.push_back(-1);
        exp_result.push_back(3);exp_result.push_back(-1);
        exp_result.push_back(1);exp_result.push_back(-1);
    }
    std::vector<int> result ( 10, -1 );
    std::vector<int> input ( n_input, n_input + 10 );
    std::vector<int> map ( n_map, n_map + 10 );
    std::vector<int> stencil ( n_stencil, n_stencil + 10 );

    bolt::amp::device_vector<int> dmap ( map.begin( ), map.end( ) );
    bolt::amp::device_vector<int> dinput ( input.begin( ), input.end( ) );
    bolt::amp::device_vector<int> dstencil ( stencil.begin( ), stencil.end( ) );
    bolt::amp::device_vector<int> dresult ( stencil.begin( ), stencil.end( ) );

    is_even iepred;
    bolt::amp::gather_if( map.begin( ),
                          map.end( ),
                          stencil.begin( ),
                          bolt::amp::make_transform_iterator( dinput.begin( ), my_negate( ) ),
                          result.begin( ),
                          iepred );

    EXPECT_EQ(exp_result, result);
}

TEST( TransformIterator, GatherIfDV )
{
    int n_map[10]     =  {0,1,2,3,4,5,6,7,8,9};
    int n_input[10]   =  {-9,-8,-7,-6,-5,-4,-3,-2,-1,-0};
    int n_stencil[10] =  {0,1,0,1,0,1,0,1,0,1};

    std::vector<int> exp_result;
    {
        exp_result.push_back(9);exp_result.push_back(-1);
        exp_result.push_back(7);exp_result.push_back(-1);
        exp_result.push_back(5);exp_result.push_back(-1);
        exp_result.push_back(3);exp_result.push_back(-1);
        exp_result.push_back(1);exp_result.push_back(-1);
    }
    std::vector<int> result ( 10, -1 );
    std::vector<int> input ( n_input, n_input + 10 );
    std::vector<int> map ( n_map, n_map + 10 );
    std::vector<int> stencil ( n_stencil, n_stencil + 10 );

    bolt::amp::device_vector<int> dmap ( map.begin( ), map.end( ) );
    bolt::amp::device_vector<int> dinput ( input.begin( ), input.end( ) );
    bolt::amp::device_vector<int> dstencil ( stencil.begin( ), stencil.end( ) );
    bolt::amp::device_vector<int> dresult ( result.begin( ), result.end( ) );

    is_even iepred;
    bolt::amp::gather_if( dmap.begin( ),
                          dmap.end( ),
                          dstencil.begin( ),
                          bolt::amp::make_transform_iterator( dinput.begin( ), my_negate( ) ),
                          dresult.begin( ),
                          iepred );

    cmpArrays(exp_result, dresult);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Scan tests
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

//TEST(TransformIterator, InclusiveScanFloat)
//{
//    int length = 1<<10;
//    std::vector< float > refInput( length );
//   
//    for(int i=0; i<length; i++)
//    {
//      refInput[i] = 1.0f + rand( )%3;
//    }
//	bolt::amp::device_vector< float > input( refInput.begin( ), refInput.end( ) );
//    bolt::amp::device_vector< float > output( refInput.begin( ), refInput.end( ), true );
//
//    bolt::amp::plus< float > ai2;
//
//	bolt::amp::inclusive_scan( bolt::amp::make_transform_iterator( input.begin( ), square_root( ) ),
//                               bolt::amp::make_transform_iterator( input.end( ), square_root( ) ),
//                               output.begin( ), ai2 );
//
//    ::std::partial_sum( boost::make_transform_iterator( refInput.begin( ), square_root( ) ),
//                        boost::make_transform_iterator( refInput.end( ), square_root( ) ),
//                        refInput.begin( ), ai2 );
//
//    // compare results
//    cmpArrays(refInput, output);
//}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Scan by Key tests
///////////////////////////////////////////////////////////////////////////////////////////////////////////////



//TEST( TransformIterator, ScanByKey )
//{
//    int keys[11] = { 7, 0, 0, 3, 3, 3, -5, -5, -5, -5, 3 }; 
//    int vals[11] = { 2, 2, 2, 2, 2, 2,  2,  2,  2,  2, 2 }; 
//    int out[11]; 
//   
//    bolt::amp::equal_to<int> eq; 
//    bolt::amp::multiplies<int> mult; 
//    bolt::amp::device_vector<int> dkeys ( keys, keys + 11 );
//    bolt::amp::device_vector<int> dvals ( vals, vals + 11 );
//    bolt::amp::device_vector<int> dout ( vals, vals + 11, true );
//   
//    bolt::amp::inclusive_scan_by_key( bolt::amp::make_transform_iterator( dkeys.begin( ), my_negate( ) ),
//                                      bolt::amp::make_transform_iterator( dkeys.end( ), my_negate( ) ),
//                                      dvals.begin( ), out, eq, mult ); 
//   
//    int arrToMatch[11] = { 2, 2, 4, 2, 4, 8, 2, 4, 8, 16, 2 };
//
//    // compare results
//    cmpArrays<int,11>( arrToMatch, out );
//}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TransformScan tests
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct uddtI2
{
    int a;
    int b;

    bool operator==(const uddtI2& rhs) const  restrict(cpu, amp)
    {
        bool equal = true;
        equal = ( a == rhs.a ) ? equal : false;
        equal = ( b == rhs.b ) ? equal : false;
        return equal;
    }
    uddtI2 operator-() const  restrict(cpu, amp)
    {
        uddtI2 r;
        r.a = -a;
        r.b = -b;
        return r;
    }
    uddtI2 operator*(const uddtI2& rhs)  restrict(cpu, amp)
    {
        uddtI2 r;
        r.a = a*a;
        r.b = b*b;
        return r;
    }
    typedef uddtI2 result_type;
};

struct AddI2
{
    uddtI2 operator()(const uddtI2 &lhs, const uddtI2 &rhs) const  restrict(cpu, amp)
    {
        uddtI2 _result;
        _result.a = lhs.a+rhs.a;
        _result.b = lhs.b+rhs.b;
        return _result;
    };
    typedef uddtI2 result_type;
};

uddtI2 identityAddI2 = {  0, 0 };
uddtI2 initialAddI2  = { -1, 2 };


 struct NegateI2
 {
     uddtI2 operator()(const uddtI2& rhs) const restrict(cpu, amp)
     {
         uddtI2 ret;
         ret.a = -rhs.a;
         ret.b = -rhs.b;
         return ret;
     }
    typedef uddtI2 result_type;
 };
NegateI2 nI2;

struct SquareI2
{
    uddtI2 operator()(const uddtI2& rhs) const  restrict(cpu, amp)
    {
        uddtI2 ret;
        ret.a = rhs.a*rhs.a;
        ret.b = rhs.b*rhs.b;
        return ret;
    }
    typedef uddtI2 result_type;
};

SquareI2 sI2;

TEST( TransformIterator, TransformScanUDD )
{
    //setup containers
    int length = (1<<16)+23;
    bolt::amp::device_vector< uddtI2 > input(  length, initialAddI2);
    std::vector< uddtI2 > refInput( length, initialAddI2 );

    // call transform_scan
    AddI2 aI2;
    bolt::amp::transform_inclusive_scan( bolt::amp::make_transform_iterator( input.begin( ), NegateI2( ) ),
                                         bolt::amp::make_transform_iterator( input.end( ), NegateI2( ) ),
                                         input.begin( ), nI2, aI2 );

    ::std::transform( boost::make_transform_iterator( refInput.begin( ), NegateI2( ) ),
                      boost::make_transform_iterator( refInput.end( ), NegateI2( ) ),
                      refInput.begin( ), nI2); // transform in-place
	::std::partial_sum( refInput.begin(), refInput.end(), refInput.begin(), aI2); // in-place scan

    // compare results
	cmpArrays(refInput, input);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Misc tests
///////////////////////////////////////////////////////////////////////////////////////////////////////////////


TEST(TransformIterator, ti)
{

    typedef int etype;
    etype elements[WAVEFRNT_SIZE];
    etype empty[WAVEFRNT_SIZE];

    size_t view_size = WAVEFRNT_SIZE;

    std::iota(elements, elements+WAVEFRNT_SIZE, 1000);
    std::fill(empty, empty+WAVEFRNT_SIZE, 0);

    bolt::amp::device_vector<int, concurrency::array_view> dve(elements, elements + WAVEFRNT_SIZE);
    bolt::amp::device_vector<int, concurrency::array_view> dumpV(empty, empty + WAVEFRNT_SIZE);
    bolt::amp::device_vector<int, concurrency::array_view> dumpCheckV(empty, empty + WAVEFRNT_SIZE);

    bolt::amp::transform( dumpCheckV.begin( ),
                          dumpCheckV.end( ),
                          dumpCheckV.begin( ),
                          bolt::amp::negate<int>( ) );

    auto dvebegin = dve.begin( );
    auto dveend = dve.end( );

    bolt::amp::control ctl;
    concurrency::accelerator_view av = ctl.getAccelerator( ).default_view;

    typedef bolt::amp::transform_iterator<my_negate, bolt::amp::device_vector<int>::iterator> transf_iter;

    transf_iter tbegin(dvebegin, my_negate( ));
    transf_iter tend(dveend, my_negate( ));

    auto dumpAV = dumpV.begin( ).getContainer( ).getBuffer( );

    concurrency::extent< 1 > inputExtent( WAVEFRNT_SIZE );
    concurrency::parallel_for_each(av, inputExtent, [=](concurrency::index<1> idx)restrict(amp)
    {
      int gidx = idx[0];

      dumpAV[gidx] = tbegin[gidx];
       
    });
    dumpAV.synchronize( );

    cmpArrays(dumpCheckV,dumpV);

}



int main(int argc, char* argv[])
{
    ::testing::InitGoogleTest( &argc, &argv[ 0 ] );

    //    Set the standard OpenCL wait behavior to help debugging
    bolt::amp::control& myControl = bolt::amp::control::getDefault( );
    myControl.setWaitMode( bolt::amp::control::NiceWait );
    myControl.setForceRunMode( bolt::amp::control::Automatic );

    int retVal = RUN_ALL_TESTS( );

#ifdef BUILD_TBB

    bolt::amp::control& myControl = bolt::amp::control::getDefault( );
    myControl.setWaitMode( bolt::amp::control::NiceWait );
    myControl.setForceRunMode( bolt::amp::control::MultiCoreCpu );  // choose tbb


    int retVal = RUN_ALL_TESTS( );

#endif
    
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
