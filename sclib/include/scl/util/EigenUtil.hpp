/**
 * @file EigenUtil.hpp
 * <a href="https://qiita.com/r9y9/items/c5fc39b5d51a20b7d18c">参考サイト</a>
 */

#ifndef SCL_EIGEN_UTIL_HPP
#define SCL_EIGEN_UTIL_HPP

#include <Eigen/Core>
#include <Eigen/LU>

#include <iterator>
#include <memory>

namespace scl
{
    namespace internal {
        /**
           @brief Serialize Matrix to 1D array
           @param matrix
           @param array1d: pointer of 1D array
        */
        template <class Matrix, class T>
        inline void matrixToArray1d(Matrix& matrix, T* array1d)
        {
            const int rows = matrix.size();
            const int cols = matrix[0].size();
            
            for (int i = 0 ; i < rows; ++i) {
                std::copy(matrix[i].begin(), matrix[i].end(), array1d);
                array1d += cols;
            }
        }
        
    } // end namespace internal

    
    /**
       @brief Eigen::Matrix from matrix
       @param matrix
       @return Eigen::Matrix
    */
    template <class ValueType=double, class Matrix>
    Eigen::Matrix<ValueType, Eigen::Dynamic, Eigen::Dynamic>
    toEigenMatrix(Matrix& matrix)
    {
        const int rows = matrix.size();
        const int cols = matrix[0].size();
        
        // copy to 1d array
        std::unique_ptr<ValueType[]> array1d(new ValueType [rows*cols]);
        internal::matrixToArray1d(matrix, array1d.get());
        
        // eigen matrix from 1darray
        return Eigen::Map<Eigen::Matrix<ValueType, Eigen::Dynamic, Eigen::Dynamic> >(array1d.get(), rows, cols);
    }

    
    /**
       @brief Eigen::Vector from vector
       @param vector
       @return Eigen::Vector
    */
    template <class Vector>
    Eigen::Matrix<typename Vector::value_type, Eigen::Dynamic, 1>
    toEigenVector(Vector& vector)
    {
        typedef typename Vector::value_type value_type;
        return Eigen::Map<Eigen::Matrix<value_type, Eigen::Dynamic, 1> >(&vector[0], vector.size(), 1);
    }


    //-------------------------------------------------
    // やっつけ自作
    //-------------------------------------------------

    /**
     * @brief convert matrix data : STL -> Eigen
     */
    template<class DataType>
    Eigen::MatrixXd toEigenMatrix(const std::size_t dim, const std::vector<DataType> & stl_mat)
    {
        // copy
        const std::size_t num( stl_mat.size() );
        Eigen::MatrixXd eigen_mat(num, dim);
        for (std::size_t i = 0; i < num; ++i)
        {
            for (std::size_t j = 0; j < dim; ++j)
            {
                eigen_mat(i,j) = static_cast<double>(stl_mat[i][j]);
            }
        }
        return eigen_mat;
    }


    /**
     * @brief convert matrix data : STL -> Eigen
     */
    template<class DataType>
    Eigen::MatrixXd toEigenMatrix(const std::size_t dim, const std::vector<DataType> & stl_mat, const std::vector<std::size_t> &indices)
    {
        // copy
        const std::size_t num( indices.size() );
        Eigen::MatrixXd eigen_mat(num, dim);
        for (std::size_t i = 0; i < num; ++i)
        {
            std::size_t index(indices[i]);
            for (std::size_t j = 0; j < dim; ++j)
            {
                eigen_mat(i,j) = static_cast<double>(stl_mat[index][j]);
            }
        }
        return eigen_mat;
    }


    /** @brief convert vector data : DataType -> Eigen */
    template<class DataType>
    Eigen::VectorXd toEigenVector(const std::size_t dim, const DataType data)
    {
        Eigen::VectorXd vec(dim);
        for (std::size_t i = 0; i < dim; ++i)
        {
            vec(i) = static_cast<double>(data[i]);
        }
        return vec;
    }
}

#endif
