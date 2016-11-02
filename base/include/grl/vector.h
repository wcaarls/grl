/** \file vector.h
 * \brief Utility functions to make std::vector more useful.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
 *
 * \copyright \verbatim
 * Copyright (c) 2015, Wouter Caarls
 * All rights reserved.
 *
 * This file is part of GRL, the Generic Reinforcement Learning library.
 *
 * GRL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * \endverbatim
 */

#ifndef GRL_VECTOR_H_
#define GRL_VECTOR_H_

#define GRL_EIGEN_VECTOR

#ifdef GRL_EIGEN_VECTOR
#include <eigen3/Eigen/Dense>
#endif 

#include <vector>
#include <iostream>

#include <math.h>
#include <string.h>

#include <grl/compat.h>

template < class T >
inline std::ostream& operator << (std::ostream& os, const std::vector<T>& v)
{
  os << "[ ";
  for (size_t ii=0; ii < v.size(); ++ii)
  {
    os << v[ii];
    if (ii < v.size()-1)
      os << ", ";
  }
  os << " ]";
  return os;
}

template < class T >
inline std::istream& operator >> (std::istream& is, std::vector<T>& v)
{
  char c=0;
  v.clear();

  // Skip whitespace
  while (is.good() && ((c = is.get()) == ' ' || c == '\t'));

  if (!is.good())
    return is;

  // Check for single element
  if (c != '[')
  {
    is.putback(c);

    T e;
    is >> e;

    if (!is.fail())
      v.push_back(e);

    return is;
  }

  // Read vector
  while (is.good())
  {
    // Hack to read empty vectors fully
    while (is.good() && ((c = is.get()) == ' ' || c == '\t'));
    if (c == ']')
      break;
    else
      is.putback(c);
  
    T e;
    is >> e;

    if (is.good())
      v.push_back(e);

    while (is.good() && (c = is.get()) != ']' && c != ',');
    if (c == ']')
      break;
  }

  return is;
}

namespace grl {

using ::operator >>;
using ::operator <<;

typedef std::vector<size_t> IndexVector;

#ifdef GRL_EIGEN_VECTOR
typedef Eigen::Array<double,1,Eigen::Dynamic> Vector;

typedef Eigen::Matrix<double,1,Eigen::Dynamic> RowVector;
typedef Eigen::Matrix<double,Eigen::Dynamic,1> ColumnVector;

inline Vector ConstantVector(size_t sz, double d)
{
  return Vector::Constant(sz, d);
}
inline Vector VectorConstructor(double a)
{
  Vector v(1); v << a; return v;
}
inline Vector VectorConstructor(double a, double b)
{
  Vector v(2); v << a, b; return v;
}
inline Vector VectorConstructor(double a, double b, double c)
{
  Vector v(3); v << a, b, c; return v;
}
inline Vector VectorConstructor(double a, double b, double c, double d)
{
  Vector v(4); v << a, b, c, d; return v;
}
inline Vector VectorConstructor(double a, double b, double c, double d, double e)
{
  Vector v(5); v << a, b, c, d, e; return v;
}
inline Vector VectorConstructor(double a, double b, double c, double d, double e, double f)
{
  Vector v(6); v << a, b, c, d ,e, f; return v;
}
inline Vector VectorConstructor(double a, double b, double c, double d, double e, double f, double g)
{
  Vector v(7); v << a, b, c, d ,e, f, g; return v;
}

using ::pow;
using ::log;

#else
typedef std::vector<double> Vector;

inline Vector ConstantVector(size_t sz, double d)
{
  return Vector(sz, d);
}
inline Vector VectorConstructor(double a)
{
  Vector v; v.push_back(a); return v;
}
inline Vector VectorConstructor(double a, double b)
{
  Vector v = VectorConstructor(a); v.push_back(b); return v;
}
inline Vector VectorConstructor(double a, double b, double c)
{
  Vector v = VectorConstructor(a, b); v.push_back(c); return v;
}
inline Vector VectorConstructor(double a, double b, double c, double d)
{
  Vector v = VectorConstructor(a, b, c); v.push_back(d); return v;
}
inline Vector VectorConstructor(double a, double b, double c, double d, double e)
{
  Vector v = VectorConstructor(a, b, c, d); v.push_back(e); return v;
}
inline Vector VectorConstructor(double a, double b, double c, double d, double e, double f)
{
  Vector v = VectorConstructor(a, b, c, d, e); v.push_back(f); return v;
}

inline Vector operator+ (const Vector &a, const Vector &b)
{
  Vector c;
  c.resize(std::min(a.size(), b.size()));
  for (size_t ii=0; ii < c.size(); ++ii)
  c[ii] = a[ii] + b[ii];
  return c;
}

inline Vector operator+ (const Vector &a, double b)
{
  Vector c;
  c.resize(a.size());
  for (size_t ii=0; ii < c.size(); ++ii)
  c[ii] = a[ii] + b;
  return c;
}

inline Vector operator- (const Vector &a, const Vector &b)
{
  Vector c;
  c.resize(std::min(a.size(), b.size()));
  for (size_t ii=0; ii < c.size(); ++ii)
  c[ii] = a[ii] - b[ii];
  return c;
}

inline Vector operator- (const Vector &a, double b)
{
  Vector c;
  c.resize(a.size());
  for (size_t ii=0; ii < c.size(); ++ii)
  c[ii] = a[ii] - b;
  return c;
}

inline Vector operator/ (const Vector &a, double b)
{
  Vector c;
  c.resize(a.size());
  for (size_t ii=0; ii < c.size(); ++ii)
  c[ii] = a[ii] / b;
  return c;
}

inline Vector operator/ (double a, const Vector &b)
{
  Vector c;
  c.resize(b.size());
  for (size_t ii=0; ii < c.size(); ++ii)
  c[ii] = a / b[ii];
  return c;
}

inline Vector operator/ (const Vector &a, const Vector &b)
{
  Vector c;
  c.resize(std::min(a.size(), b.size()));
  for (size_t ii=0; ii < c.size(); ++ii)
  c[ii] = a[ii] / b[ii];
  return c;
}

inline Vector operator* (const Vector &a, double b)
{
  Vector c;
  c.resize(a.size());
  for (size_t ii=0; ii < c.size(); ++ii)
  c[ii] = a[ii] * b;
  return c;
}

inline Vector operator* (double a, const Vector &b)
{
  Vector c;
  c.resize(b.size());
  for (size_t ii=0; ii < c.size(); ++ii)
  c[ii] = a * b[ii];
  return c;
}

inline Vector operator* (const Vector &a, const Vector &b)
{
  Vector c;
  c.resize(std::min(a.size(), b.size()));
  for (size_t ii=0; ii < c.size(); ++ii)
  c[ii] = a[ii] * b[ii];
  return c;
}

using ::pow;

inline Vector pow(const Vector &a, const Vector &b)
{
  Vector c;
  c.resize(std::min(a.size(), b.size()));
  for (size_t ii=0; ii < c.size(); ++ii)
    c[ii] = pow(a[ii], b[ii]);
  return c;
}

using ::log;

inline Vector log(const Vector &a)
{
  Vector c(a.size());
  for (size_t ii=0; ii < c.size(); ++ii)
    c[ii] = log(a[ii]);
  return c;
}
#endif

inline double sum (const Vector &a)
{
  double e = 0;
  
  for (size_t ii=0; ii < a.size(); ++ii)
    e += a[ii];
  return e;
}

inline double prod (const Vector &a)
{
  double e = 1;
  
  for (size_t ii=0; ii < a.size(); ++ii)
    e *= a[ii];
  return e;
}

inline double dot (const Vector &a, const Vector &b)
{
  return sum(a*b);
}

inline Vector extend(const Vector &a, const Vector &b)
{
  Vector c(a.size()+b.size());
  
  for (size_t ii=0; ii < a.size(); ++ii)
    c[ii] = a[ii];
  for (size_t ii=0; ii < b.size(); ++ii)
    c[a.size()+ii] = b[ii];

  return c;
}

template <class T>
inline void fromVector(const Vector &vector, T &to)
{
  to.resize(vector.size());
  for (size_t ii = 0; ii < vector.size(); ++ii)
    to[ii] = vector[ii];
}

template <class T>
inline void toVector(const T &from, Vector &vector)
{
  vector.resize(from.size());
  for (size_t ii = 0; ii < from.size(); ++ii)
    vector[ii] = from[ii];
}

#ifdef GRL_EIGEN_VECTOR
typedef Eigen::MatrixXd Matrix;

inline Matrix diagonal(const Vector &v)
{
  return v.matrix().asDiagonal();
}
#else
class Matrix
{
  protected:
    size_t rows_, cols_;
    double *data_;
    
  public:
    Matrix(size_t rows=0, size_t cols=0) : rows_(rows), cols_(cols), data_(NULL)
    {
      if (rows_*cols_ > 0)
        data_ = new double[rows_*cols_];
    }
    
    Matrix(const Matrix &obj) : rows_(obj.rows_), cols_(obj.cols_), data_(NULL)
    {
      if (rows_*cols_ > 0)
      {
        data_ = new double[rows_*cols_];
        memcpy(data_, obj.data_, rows_*cols_*sizeof(double));
      }
    }
    
    ~Matrix()
    {
      if (data_)
        delete[] data_;
    }
    
    double operator()(size_t row, size_t col) const
    {
      return data_[row*cols_+col];
    }
    
    double &operator()(size_t row, size_t col)
    {
      return data_[row*cols_+col];
    }
    
    size_t size() const
    {
      return rows_*cols_;
    }

    size_t rows() const
    {
      return rows_;
    }
    
    size_t cols() const
    {
      return cols_;
    }
    
    static Matrix Zero(size_t rows, size_t cols)
    {
      Matrix m(rows, cols);
      memset(m.data_, 0, rows*cols*sizeof(double));
      return m;
    }
    
    static Matrix Identity(size_t rows, size_t cols)
    {
      Matrix m = Zero(rows, cols);
      for (size_t ii=0; ii < rows && ii < cols; ++ii)
        m(ii, ii) = 1.;
      return m;
    }
    
    Matrix operator*(const Matrix &rhs) const
    {
      if (!size() || !rhs.size())
        return Matrix();
    
      grl_assert(cols_ == rhs.rows_);
      
      Matrix m = Zero(rows_, rhs.cols_);

      for (size_t rr=0; rr < rows_; ++rr)
        for (size_t cc=0; cc < rhs.cols_; ++cc)
          for (size_t kk=0; kk < cols_; ++kk)
            m(rr, cc) += (*this)(rr, kk)*rhs(kk, cc);
            
      return m;
    }
    
    friend std::ostream& operator<<(std::ostream& os, const Matrix& m)
    {
      os << "[";
      for (size_t rr=0; rr < m.rows(); ++rr)
      {
        if (rr)
          os << " ";
        os << "[ ";
        for (size_t cc=0; cc < m.cols(); ++cc)
        {
          os << m(rr, cc);
          if (cc < m.cols()-1)
            os << ", ";
        }
        os << " ]";
        if (rr < m.rows()-1)
          os << "," << std::endl;
      }
      os << "]";
      return os;
    }
};

inline Matrix diagonal(const Vector &v)
{
  size_t sz = v.size();
  Matrix m = Matrix::Zero(sz, sz);
  for (size_t ii=0; ii < sz; ++ii)
    m(ii, ii) = v[ii];
  return m;
}
#endif

}

#endif /* GRL_VECTOR_H_ */
