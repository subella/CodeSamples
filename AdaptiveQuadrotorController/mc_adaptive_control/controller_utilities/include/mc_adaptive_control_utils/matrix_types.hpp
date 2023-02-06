/**
 * @file matrix_types.hpp
 * Matrix typedefs
 *
 * @author Nathan Hughes <nathan.h.hughes@gmail.com>
 *
 */

#pragma once
#include <matrix/matrix/math.hpp>

namespace matrix {
typedef Vector<float, 1> Vector1f;
typedef Vector<float, 4> Vector4f;
typedef Vector<float, 5> Vector5f;
typedef Vector<float, 7> Vector7f;
typedef Vector<float, 10> Vector10f;
typedef Vector<float, 11> Vector11f;
typedef Vector<float, 12> Vector12f;
typedef Vector<float, 13> Vector13f;
typedef Vector<float, 14> Vector14f;
typedef Matrix<float, 7, 3> Matrix73f;
typedef Matrix<float, 4, 3> Matrix43f;
typedef Matrix<float, 10, 14> Matrix10_14f;
typedef Matrix<float, 10, 12> Matrix10_12f;
typedef Matrix<float, 10, 10> Matrix10_10f;
typedef Matrix<float, 1, 10> Matrix1_10f;

typedef Matrix<float, 5, 3> Matrix5_3f;
typedef Matrix<float, 5, 7> Matrix5_7f;
typedef Matrix<float, 5, 11> Matrix5_11f;
typedef Matrix<float, 5, 13> Matrix5_13f;
typedef Matrix<float, 5, 5> Matrix5_5f;
typedef Matrix<float, 1, 5> Matrix1_5f;

template <typename Type, size_t M, size_t N>
auto operator*(const Vector<Type, M>& first, const Matrix<Type, 1, N>& second)
    -> Matrix<Type, M, N> {
  Matrix<Type, M, N> result;
  for (size_t i = 0; i < M; ++i) {
    for (size_t j = 0; j < N; ++j) {
      result(i, j) = first(i) * second(0, j);
    }
  }
  return result;
}

inline auto randFloat(float fMin, float fMax) -> float {
  float f = static_cast<float>(rand()) / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

template <typename Type, size_t M, size_t N>
void randomMatrix(matrix::Matrix<Type, M, N>& mat, float min, float max) {
  for (size_t r = 0; r < M; ++r) {
    for (size_t c = 0; c < N; ++c) {
      mat(r, c) = randFloat(min, max);
    }
  }
}

}  // namespace matrix
