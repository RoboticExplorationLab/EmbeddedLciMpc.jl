#include <iostream>
#include <vector> 
#include <julia.h> 
#include "juliaCpp.h" 
JULIA_DEFINE_FAST_TLS()

using namespace std; 
int main() {
    jl_init();
    jl_eval_string("using Pkg");
    jl_eval_string("Pkg.activate(\"../\")"); 
    jl_eval_string("using EmbeddedLciMpc");

    // Important: We have to get a reference to the module in order to get the function
    jl_module_t* EmbeddedLciMpc = (jl_module_t *)jl_eval_string("EmbeddedLciMpc");

    //// Example of calling function with a scalar 
    jl_function_t* add1 = jl_get_function(EmbeddedLciMpc, "add1");
    jl_value_t *argument = jl_box_float64(2.0); 
    jl_value_t *result = jl_call1(add1, argument);
    double r = jl_unbox_float64(result); 
    // std::cout << r << std::endl; 

    //// Example of calling a function with a 1D array 
    // first define an array type, the 1 denotes a 1d array
    jl_value_t* array_type = jl_apply_array_type((jl_value_t*)jl_float64_type, 1); 
    vector<double> x(10);
    for(int i = 0; i < x.size(); ++i) {
        x[i] = i; 
    }

    // if you have an existing array like a vector, you can pass its pointer and 
    // its size to directly convert it to a 2d array 
    jl_array_t *x_j  = jl_ptr_to_array_1d(array_type, x.data(), x.size(), 0); 
    jl_function_t *times2 = jl_get_function(EmbeddedLciMpc, "times2array"); 
    jl_array_t *y_j = (jl_array_t*) jl_call1(times2, (jl_value_t*)x_j); 

    // you can get the returned result using jl_array_data 
    double *yData = (double*)jl_array_data(y_j); 
    vector<double> y(yData, yData + 10); 
    for(int i = 0; i < y.size(); ++i) {
        cout << y[i] << endl; 
    } 


    //// Example of calling a function with a 1D array 
    jl_function_t *findEigs = jl_get_function(EmbeddedLciMpc, "findEigs"); 
    jl_value_t *array2_type = jl_apply_array_type((jl_value_t*)jl_float64_type, 2); 
    jl_array_t *x_j2 = jl_alloc_array_2d(array2_type, 10, 10); 

    double *p = (double*) jl_array_data(x_j2); 
    int ndims = jl_array_ndims(x_j2);
    size_t size0 = jl_array_dim(x_j2, 0);
    size_t size1 = jl_array_dim(x_j2, 1); 

    for(size_t i=0; i < size1; i++) {
        for(size_t j=0; j < size0; j++) {
            p[j + size0*i] = 1; 
        }
    }

    jl_array_t *y_j2 = (jl_array_t*) jl_call1(findEigs, (jl_value_t*)x_j2);
    double *y_out2 = (double*)jl_array_data(y_j2);
    vector<double> y2(y_out2, y_out2 + 10); 

    for(size_t i = 0; i < 10; i++) {
        cout << y2[i] << endl; 
    }

    jl_atexit_hook(0);

    return -1;
}; 