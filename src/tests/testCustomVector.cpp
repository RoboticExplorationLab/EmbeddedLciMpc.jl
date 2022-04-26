#include <iostream>
#include <vector> 
#include "Eigen/Dense"
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

    // Example of calling a function with a 1D array 
    jl_function_t *times2 = jl_get_function(EmbeddedLciMpc, "times2array"); 
    Eigen::MatrixXd test(10, 10); test.setIdentity(); 
    JuliaVector2d jv(test); 
    JuliaVector2d result((jl_array_t*) jl_call1(times2, (jl_value_t*) jv.toJArray())); 
    cout << result.toEigenMatrix() << endl; 

    
    // Example of calling a function that returns a 1D array 
    jl_function_t *findEigs = jl_get_function(EmbeddedLciMpc, "findEigs"); 
    jl_array_t *j_result = (jl_array_t*) jl_call1(findEigs, (jl_value_t*) jv.toJArray());
    JuliaVector2d result2(j_result); 
    cout << result2.toEigenMatrix() << endl; 

    jl_atexit_hook(0);
    return -1; 
}