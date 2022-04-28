
#include <vector> 
#include "Eigen/Dense"
#include "julia.h"

using namespace std; 
class JuliaVector1d {
    public:
        JuliaVector1d(jl_array_t *j_arr) {
            julia_data_ = j_arr; 
            size_ = jl_array_dim(j_arr, 0); 
            double *data = (double*)jl_array_data(julia_data_); 
            data_ = vector<double>(data, data+size_); 
            array_type_ = jl_apply_array_type((jl_value_t*)jl_float64_type, 1);
        }

        // Convert a vector to julia vector 
        JuliaVector1d(std::vector<double> vec) {
            data_ = vec; 
            size_ = vec.size(); 
            array_type_ = jl_apply_array_type((jl_value_t*)jl_float64_type, 1); 
            julia_data_ = jl_ptr_to_array_1d(array_type_, data_.data(), size_, 0); 
        }

        std::vector<double> toCVector() {
            double *j_data = (double*) jl_array_data(julia_data_); 
            data_ = vector<double>(j_data, j_data + size_); 
            return data_; 
        }

        jl_array_t* toJArray() {
            return julia_data_; 
        }

        size_t size() {
            return size_; 
        }

        double operator[](size_t index) {
            return data_[index];
        }

    private:
        size_t size_; 
        jl_value_t* array_type_;
        jl_array_t* julia_data_; 
        vector<double> data_;
}; 

class JuliaVector2d {
    // Julia 2D array wrapper 
    public:
        JuliaVector2d(Eigen::MatrixXd vec) {
            // Get the dimension of the matrix and load it into relevant 2D 
            // julia_array_t. Loading is accomplished by putting data into data_,
            // which is a pointer to the values hold by the julia vector
            vec_ = vec; 
            n_cols_ = vec_.cols(); 
            n_rows_ = vec_.rows(); 
            array_type_ = jl_apply_array_type((jl_value_t*)jl_float64_type, 2); 
            julia_data_ = jl_alloc_array_2d(array_type_, n_rows_, n_cols_); 
            data_ = (double *) jl_array_data(julia_data_); 

            for(size_t c = 0; c < n_cols_; c++) {
                for(size_t r = 0; r < n_rows_; r++) {
                    data_[r + n_rows_*c] = vec_(r, c); 
                }
            } 
        }

        JuliaVector2d(jl_array_t *j_arr) {
            julia_data_ = j_arr; 
            array_type_ = jl_apply_array_type((jl_value_t*)jl_float64_type, 2); 
            data_ = (double*) jl_array_data(julia_data_);

            // If it is dimension 1, we default to a column vector 
            int n_dims = jl_array_ndims(j_arr); 
            if(n_dims == 1) {
                n_rows_ = jl_array_dim(j_arr, 0);
                n_cols_ = 1; 
            } else {
                n_rows_ = jl_array_dim(j_arr, 0); 
                n_cols_ = jl_array_dim(j_arr, 1); 
            }
            vec_.resize(n_rows_, n_cols_);
            for(size_t c = 0; c < n_cols_; c++) {
                for(size_t r = 0; r < n_rows_; r++) {
                    vec_(r, c) = data_[r + n_rows_*c]; 
                }
            }
        }

        jl_array_t* toJArray() {
            return julia_data_; 
        }

        Eigen::MatrixXd toEigenMatrix() {
            for(size_t c = 0; c < n_cols_; c++) {
                for(size_t r = 0; r < n_rows_; r++) {
                    vec_(r, c) = data_[r + n_rows_*c]; 
                }
            }
            return vec_; 
        }

    private: 
        size_t n_cols_; 
        size_t n_rows_; 
        jl_value_t* array_type_; 
        jl_array_t* julia_data_; 
        double* data_; 
        Eigen::MatrixXd vec_; 

};
