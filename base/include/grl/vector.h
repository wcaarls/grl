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

#include <vector>
#include <iostream>

#include <math.h>

template<class T>
inline std::vector<T> VectorConstructor(T a)
{
  std::vector<T> v; v.push_back(a); return v;
}
template<class T>
inline std::vector<T> VectorConstructor(T a, T b)
{
  std::vector<T> v = VectorConstructor(a); v.push_back(b); return v;
}
template<class T>
inline std::vector<T> VectorConstructor(T a, T b, T c)
{
  std::vector<T> v = VectorConstructor(a, b); v.push_back(c); return v;
}
template<class T>
inline std::vector<T> VectorConstructor(T a, T b, T c, T d)
{
  std::vector<T> v = VectorConstructor(a, b, c); v.push_back(d); return v;
}
template<class T>
inline std::vector<T> VectorConstructor(T a, T b, T c, T d, T e)
{
  std::vector<T> v = VectorConstructor(a, b, c, d); v.push_back(e); return v;
}
template<class T>
inline std::vector<T> VectorConstructor(T a, T b, T c, T d, T e, T f)
{
  std::vector<T> v = VectorConstructor(a, b, c, d, e); v.push_back(f); return v;
}

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

typedef std::vector<double> Vector;

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

inline double sum (const Vector &a)
{
  double e = 0;
  
  for (size_t ii=0; ii < a.size(); ++ii)
    e += a[ii];
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

using ::pow;

inline Vector pow(const Vector &a, const Vector &b)
{
  Vector c;
  c.resize(std::min(a.size(), b.size()));
  for (size_t ii=0; ii < c.size(); ++ii)
    c[ii] = pow(a[ii], b[ii]);
  return c;
}

}

#endif /* GRL_VECTOR_H_ */
