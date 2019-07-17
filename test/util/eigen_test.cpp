#include <scl/util/EigenUtil.hpp>
#include <scl/util/Statistics.hpp>

#include <vector>
#include <iostream>

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

int main ()
{
    std::vector< std::vector<double> > stl_mat({{0, 1}, {4, 2}, {2, 2}});

    Eigen::MatrixXd mat1, mat2, mat3;
    mat1 = scl::toEigenMatrix(stl_mat);
    PRINT_MAT(mat1);
    
    mat2 = scl::toEigenMatrix(2, stl_mat);
    PRINT_MAT(mat2);

    std::vector<std::size_t> indices({0, 2});
    mat3 = scl::toEigenMatrix(2, stl_mat, indices);
    PRINT_MAT(mat3);
    
    return 0;
}
