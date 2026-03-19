#pragma once
#include <array>
#include <stdexcept>
#include <cmath>
#include <initializer_list>

namespace defensekit {

// 고정 크기 행렬 (N x M)
template<int N, int M>
class Matrix {
public:
    std::array<std::array<double, M>, N> data{};

    Matrix() {
        for (auto& row : data)
            row.fill(0.0);
    }

    Matrix(std::initializer_list<std::initializer_list<double>> init) {
        int i = 0;
        for (auto& row : init) {
            int j = 0;
            for (double val : row) {
                data[i][j++] = val;
            }
            ++i;
        }
    }

    double& operator()(int row, int col) { return data[row][col]; }
    double  operator()(int row, int col) const { return data[row][col]; }

    static Matrix<N, N> identity() {
        static_assert(N == M, "Identity requires square matrix");
        Matrix<N, N> result;
        for (int i = 0; i < N; ++i)
            result(i, i) = 1.0;
        return result;
    }

    Matrix<N, M> operator+(const Matrix<N, M>& rhs) const {
        Matrix<N, M> result;
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < M; ++j)
                result(i, j) = data[i][j] + rhs(i, j);
        return result;
    }

    Matrix<N, M> operator-(const Matrix<N, M>& rhs) const {
        Matrix<N, M> result;
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < M; ++j)
                result(i, j) = data[i][j] - rhs(i, j);
        return result;
    }

    template<int K>
    Matrix<N, K> operator*(const Matrix<M, K>& rhs) const {
        Matrix<N, K> result;
        for (int i = 0; i < N; ++i)
            for (int k = 0; k < K; ++k)
                for (int j = 0; j < M; ++j)
                    result(i, k) += data[i][j] * rhs(j, k);
        return result;
    }

    Matrix<N, M> operator*(double scalar) const {
        Matrix<N, M> result;
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < M; ++j)
                result(i, j) = data[i][j] * scalar;
        return result;
    }

    // 전치 행렬
    Matrix<M, N> transpose() const {
        Matrix<M, N> result;
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < M; ++j)
                result(j, i) = data[i][j];
        return result;
    }
};

// 2x2 역행렬 (칼만 필터 게인 계산용)
inline Matrix<2, 2> inverse2x2(const Matrix<2, 2>& m) {
    double det = m(0,0)*m(1,1) - m(0,1)*m(1,0);
    if (std::abs(det) < 1e-12)
        throw std::runtime_error("Matrix is singular");
    Matrix<2, 2> inv;
    inv(0,0) =  m(1,1) / det;
    inv(0,1) = -m(0,1) / det;
    inv(1,0) = -m(1,0) / det;
    inv(1,1) =  m(0,0) / det;
    return inv;
}

// 4x4 역행렬 (가우스-조르당 소거법)
inline Matrix<4, 4> inverse4x4(const Matrix<4, 4>& m) {
    // 증강 행렬 [M | I] 구성
    double aug[4][8]{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j)
            aug[i][j] = m(i, j);
        aug[i][i + 4] = 1.0;
    }
    for (int col = 0; col < 4; ++col) {
        // 피벗 찾기
        int pivot = col;
        for (int row = col + 1; row < 4; ++row)
            if (std::abs(aug[row][col]) > std::abs(aug[pivot][col]))
                pivot = row;
        if (std::abs(aug[pivot][col]) < 1e-12)
            throw std::runtime_error("Matrix is singular");
        std::swap(aug[col], aug[pivot]);
        double scale = aug[col][col];
        for (int j = 0; j < 8; ++j)
            aug[col][j] /= scale;
        for (int row = 0; row < 4; ++row) {
            if (row == col) continue;
            double factor = aug[row][col];
            for (int j = 0; j < 8; ++j)
                aug[row][j] -= factor * aug[col][j];
        }
    }
    Matrix<4, 4> inv;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            inv(i, j) = aug[i][j + 4];
    return inv;
}

} // namespace defensekit
